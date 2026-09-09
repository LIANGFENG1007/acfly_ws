#include "exploration_planner/coverage_planner.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {
void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

GridSnapshot explored_grid()
{
    GridSnapshot grid;
    grid.cfg = GridConfig{0, -5, 7.5, 5, .5, .05, .9, 100, 3};
    grid.bnx = 15; grid.bny = 20;
    grid.big.assign(static_cast<size_t>(grid.bnx) * grid.bny, 1);
    return grid;
}

void unknown(GridSnapshot& grid, int x, int y)
{
    grid.big[static_cast<size_t>(x) * grid.bny + y] = 0;
}

FrontierConfig config()
{
    return FrontierConfig{0, -5, 7.5, 5, .5, 1, 1.4, 1.7, 1.8,
                          6, 8, 1, 4, .4, 1, 6};
}

void prefer_regions_without_losing_fragments()
{
    auto grid = explored_grid();
    for (int x = 2; x <= 4; ++x)
        for (int y = 9; y <= 11; ++y)
            if (!(x == 2 && y != 10)) unknown(grid, x, y);
    for (int x = 12; x <= 14; ++x)
        for (int y = 9; y <= 12; ++y) unknown(grid, x, y);
    const auto before = grid.big;
    int band = 1;
    auto selection = select_observation_target(grid, config(), {2.25, .25}, 0, band, {});
    require(selection.valid && selection.region_size == 12,
            "a nearby small fragment displaced the broad forward region");
    require(!selection.cleanup && !selection.requires_turn,
            "a normal forward region unnecessarily requested cleanup turning");
    require(grid.big == before, "selection changed observed coverage");
    for (int x = 12; x <= 14; ++x)
        for (int y = 9; y <= 12; ++y)
            grid.big[static_cast<size_t>(x) * grid.bny + y] = 1;
    const auto fragments = grid.big;
    selection = select_observation_target(grid, config(), {2.25, .25}, 0, band, {});
    require(selection.valid && selection.region_size == 7 && selection.cleanup,
            "the last small region was lost instead of being observed");
    require(grid.big == fragments, "cleanup fabricated observations");
}

void observe_narrow_wall_pocket_from_outside()
{
    auto grid = explored_grid();
    unknown(grid, 1, 10);  // (0.75, 0.25), field clamp previously requested (1,0.25).
    const Obstacles obstacles{{2.05, .25, .35}};
    int band = 1;
    const Vec2 current{3, 2.5};
    const auto selection = select_observation_target(grid, config(), current, -2.4, band, obstacles);
    require(selection.valid, "a visible pocket was declared inaccessible");
    require(selection.clearance >= .60 && selection.point.y > .7,
            "the aircraft was sent into the tight wall-column pocket: point=" +
            std::to_string(selection.point.x) + "," + std::to_string(selection.point.y) +
            " clearance=" + std::to_string(selection.clearance));
    require(std::hypot(selection.point.x - current.x, selection.point.y - current.y) >= .8,
            "a tiny waypoint defeated the minimum observation travel distance");
    require(visible_unknown_count(grid, selection.point,
        std::atan2(selection.look_at.y - selection.point.y, selection.look_at.x - selection.point.x),
        obstacles, config()) > 0, "the selected outside observation cannot see its region");
}

void visibility_uses_physical_occlusion_and_real_unscanned_cells()
{
    auto grid = explored_grid();
    unknown(grid, 9, 10);  // (4.75,0.25).
    const Obstacles obstacles{{4, 0, .4}};
    require(grid.has_gain_within({4.75, .25}, 1), "the occluded old-gain fixture is invalid");
    require(visible_unknown_count(grid, {3, 0}, 0, obstacles, config()) == 0,
            "an unknown cell behind a solid column kept an observation alive");
    require(visible_unknown_count(grid, {5.75, .25}, M_PI, obstacles, config()) == 1,
            "a genuine view from the other side was rejected");
    require(visible_unknown_count(grid, {5.75, .25}, 0, obstacles, config()) == 0,
            "a cell behind the camera counted as forward information");

    grid = explored_grid();
    unknown(grid, 4, 10);
    grid.snx = 150; grid.sny = 200; grid.cpb = 10;
    grid.small.assign(static_cast<size_t>(grid.snx) * grid.sny, 1);
    grid.small[static_cast<size_t>(40) * grid.sny + 100] = 0;  // (2.025,0.025).
    require(visible_unknown_count(grid, {2.25, .25}, 0, {}, config()) == 0,
            "already scanned portions of an unfinished cell created forward gain");
    require(visible_unknown_count(grid, {2.25, .25}, -2.4, {}, config()) == 1,
            "the real remaining small-cell observation was missed");
}

void final_near_fragment_gets_an_explicit_observation_turn()
{
    auto grid = explored_grid();
    unknown(grid, 6, 10);  // The unknown point is almost underneath the aircraft.
    int band = 1;
    const Vec2 current{3.25, .25};
    const auto selection = select_observation_target(grid, config(), current, 0, band, {});
    require(selection.valid && selection.cleanup && selection.requires_turn,
            "the last nearby cell needs an explicit look-back observation, not silent exclusion");
    require(selection.gain == 1 &&
        std::hypot(selection.point.x - current.x, selection.point.y - current.y) >= .8,
        "cleanup ignored the real information or the minimum movement constraint");
    require(visible_unknown_count(grid, selection.point,
        std::atan2(selection.look_at.y - selection.point.y, selection.look_at.x - selection.point.x),
        {}, config()) == 1, "cleanup's final observation direction does not reveal the cell");
}

void blocked_outputs_do_not_erase_regions_or_advance_the_band()
{
    auto grid = explored_grid();
    unknown(grid, 6, 10);
    const auto before = grid.big;
    int band = 1;
    const std::vector<Vec2> blocked{{3.75, 0}};
    const auto selection = select_observation_target(grid, config(), {2, 0}, 0, band, {}, &blocked, 100);
    require(!selection.valid && grid.big == before && band == 1,
            "a failed observation erased coverage or committed candidate band state");
}

void small_heading_changes_keep_forward_observations()
{
    auto grid = explored_grid();
    for (int x = 7; x <= 13; ++x)
        for (int y = 7; y <= 14; ++y) unknown(grid, x, y);
    int first_band = 1, second_band = 1;
    const auto first = select_observation_target(grid, config(), {3, .25}, -.01, first_band, {});
    const auto second = select_observation_target(grid, config(), {3, .25}, .01, second_band, {});
    require(first.valid && second.valid && first.point.x > 3 && second.point.x > 3,
            "small yaw changes flipped a broad forward observation behind the aircraft");
    require(std::hypot(first.point.x - second.point.x, first.point.y - second.point.y) < .6,
            "near-identical headings selected widely separated observation positions");
}

void coverage_completion_requires_real_scans()
{
    const auto cfg = config();
    GridMap map(GridConfig{0, -5, 7.5, 5, .5, .05, .9, 100, 3});
    Vec2 position{3, 0};
    double yaw = 0;
    int band = 1;
    map.mark_scan(position.x, position.y, yaw);
    for (int step = 0; step < 40 && map.coverage_ratio() < .9; ++step) {
        const double before = map.coverage_ratio();
        const auto selection = select_observation_target(map.snapshot(), cfg, position, yaw, band, {});
        require(map.coverage_ratio() == before, "candidate selection fabricated coverage progress");
        require(selection.valid, "genuine remaining coverage lost every observation waypoint");
        yaw = std::atan2(selection.point.y - position.y, selection.point.x - position.x);
        position = selection.point;
        map.mark_scan(position.x, position.y, yaw);
        if (selection.requires_turn) {
            yaw = std::atan2(selection.look_at.y - position.y, selection.look_at.x - position.x);
            map.mark_scan(position.x, position.y, yaw);
        }
    }
    require(map.coverage_ratio() >= .9,
            "actual FOV observations did not finish 90 percent coverage within the bounded viewpoint sequence");
}

void band_parity_cannot_reverse_an_equivalent_forward_choice()
{
    auto grid = explored_grid();
    for (int x : {1, 2, 3, 4, 5, 9, 10, 11, 12, 13})
        for (int y = 9; y <= 10; ++y) unknown(grid, x, y);
    const auto before = grid.big;
    auto cfg = config();
    cfg.band_clear_cnt = 0;
    const Vec2 current{3.75, -1.0};
    int first_band = 0, second_band = 1;
    const auto first = select_observation_target(grid, cfg, current, 1.4, first_band, {});
    const auto second = select_observation_target(grid, cfg, current, 1.4, second_band, {});
    require(first.valid && second.valid && first.point.x > current.x && second.point.x > current.x,
            "band parity flipped the aircraft away from an equally useful forward region");
    require(std::hypot(first.point.x - second.point.x, first.point.y - second.point.y) < .01,
            "the same visible candidate set changed sides solely because of band numbering");

    // A planned left bend remains the preferred continuation while the body
    // is still completing its previous heading. This is a preference, not a ban.
    const auto left = select_observation_target(grid, cfg, current, 1.4, second_band,
                                                {}, nullptr, 0, 2.45);
    require(left.valid && left.point.x < current.x,
            "a safe planned bend was ignored in favor of the instantaneous body heading");
    require(grid.big == before, "continuity preference changed real coverage");

    grid = explored_grid();
    for (int x = 2; x <= 4; ++x)
        for (int y = 9; y <= 12; ++y) unknown(grid, x, y);
    const auto behind = select_observation_target(grid, cfg, {6, .25}, 0, second_band,
                                                  {}, nullptr, 0, 0);
    require(behind.valid && behind.gain > 0 && behind.point.x < 6,
            "route continuity prevented observing the remaining region behind the aircraft");
}

void selected_gain_keeps_its_observation_heading()
{
    auto grid = explored_grid();
    std::fill(grid.big.begin(), grid.big.end(), 0);
    for (double yaw : {-2.4, -.7, .8, 2.3}) {
        int band = 1;
        const Vec2 current{3, 0};
        const auto selection = select_observation_target(grid, config(), current, yaw, band, {});
        require(selection.valid && selection.gain == visible_unknown_count(
            grid, selection.point, selection.view_heading, {}, config()),
            "the stored observation heading does not reproduce the selected information gain");
        if (!selection.requires_turn)
            require(std::abs(std::atan2(std::sin(selection.view_heading -
                std::atan2(selection.point.y - current.y, selection.point.x - current.x)),
                std::cos(selection.view_heading -
                std::atan2(selection.point.y - current.y, selection.point.x - current.x)))) < 1e-9,
                "a normal forward viewpoint silently changed to a different look-at heading");
    }
}
}  // namespace

int main()
{
    prefer_regions_without_losing_fragments();
    observe_narrow_wall_pocket_from_outside();
    visibility_uses_physical_occlusion_and_real_unscanned_cells();
    final_near_fragment_gets_an_explicit_observation_turn();
    blocked_outputs_do_not_erase_regions_or_advance_the_band();
    small_heading_changes_keep_forward_observations();
    coverage_completion_requires_real_scans();
    band_parity_cannot_reverse_an_equivalent_forward_choice();
    selected_gain_keeps_its_observation_heading();
    auto full_grid = explored_grid();
    std::fill(full_grid.big.begin(), full_grid.big.end(), 0);
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i) {
        int band = 1;
        require(select_observation_target(full_grid, config(), {3, 0}, 0, band, {}).valid,
                "a fully unknown field lost every observation candidate");
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count() / 10;
    std::cout << "frontier_observation_test: all checks passed; full-grid selection "
              << elapsed << " ms/call\n";
}
