#include "exploration_planner/coverage_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

GridSnapshot explored_grid()
{
    GridSnapshot grid;
    grid.cfg = GridConfig{0, -5, 7.5, 5, .5, .05, .9, 100, 3};
    grid.bnx = 15;
    grid.bny = 20;
    grid.big.assign(static_cast<size_t>(grid.bnx) * grid.bny, 1);
    return grid;
}

void unscanned(GridSnapshot& grid, int x, int y)
{
    grid.big[static_cast<size_t>(x) * grid.bny + y] = 0;
}

FrontierConfig frontier()
{
    return FrontierConfig{0, -5, 7.5, 5, .5, 1.0, 1.0, 0.0, 0.0,
                          6, 0.1, 0.1, 20.0, 0.0, 0.0, 0};
}

Path2 select(const GridSnapshot& grid, const FrontierConfig& cfg, const Vec2& current,
             const std::vector<Vec2>* blocked, double radius, bool& all_explored)
{
    int band = -1;
    return plan_explore(grid, cfg, current, 0.0, band, all_explored, blocked, radius);
}

void four_corner_preimages()
{
    const std::array<Vec2, 4> corners{{{1, -4}, {1, 4}, {6.5, -4}, {6.5, 4}}};
    for (size_t corner = 0; corner < corners.size(); ++corner) {
        auto grid = explored_grid();
        const int first_x = corner < 2 ? 0 : 13;
        const int first_y = corner % 2 == 0 ? 0 : 18;
        for (int dx = 0; dx < 2; ++dx)
            for (int dy = 0; dy < 2; ++dy) unscanned(grid, first_x + dx, first_y + dy);
        const std::vector<Vec2> blocked{corners[corner]};
        bool all_explored = true;
        const auto path = select(grid, frontier(), {3.75, 0}, &blocked, .8, all_explored);
        require(path.size() == 1, "a corner preimage selected its blacklisted clamped waypoint");
        require(!all_explored, "unreachable cells must not be reported as explored");
    }
}

void ordinary_candidate_survives()
{
    auto grid = explored_grid();
    for (const int x : {0, 1, 13, 14})
        for (const int y : {0, 1, 18, 19}) unscanned(grid, x, y);
    unscanned(grid, 6, 10);  // (3.25, 0.25), unchanged by the field clamp.
    const std::vector<Vec2> blocked{{1, -4}, {1, 4}, {6.5, -4}, {6.5, 4}};
    bool all_explored = true;
    const auto path = select(grid, frontier(), {6.49, 3.99}, &blocked, .8, all_explored);
    require(path.size() == 2 && std::hypot(path.back().x - 3.25, path.back().y - .25) < 1e-12,
            "blocking corner outputs suppressed an ordinary available candidate");
    require(!all_explored, "ordinary unscanned candidate was lost");
}

void fallback_uses_the_same_coordinates()
{
    auto grid = explored_grid();
    for (const int x : {13, 14})
        for (const int y : {18, 19}) unscanned(grid, x, y);
    unscanned(grid, 6, 10);
    auto cfg = frontier();
    cfg.horizon = 0.0;  // Exercise the whole-field fallback independently of the chain.
    const std::vector<Vec2> blocked{{6.5, 4}};
    bool all_explored = false;
    const auto path = select(grid, cfg, {6.49, 3.99}, &blocked, .8, all_explored);
    require(path.size() == 2 && std::hypot(path.back().x - 3.25, path.back().y - .25) < 1e-12,
            "fallback selected a blacklisted clamped corner instead of the available point");
    grid.big[static_cast<size_t>(6) * grid.bny + 10] = 1;
    require(select(grid, cfg, {6.49, 3.99}, &blocked, .8, all_explored).size() == 1,
            "fallback must not resurrect a blacklisted target when every candidate is blocked");
}

void unblocked_selection_is_unchanged()
{
    auto grid = explored_grid();
    unscanned(grid, 14, 19);  // Raw (7.25,4.75), output (6.5,4.0).
    unscanned(grid, 12, 17);  // Raw and output (6.25,3.75).
    const Vec2 current{6.4, 4.0};
    // The clamped corner is closer, but existing scoring intentionally uses raw
    // cell centers: preserve the ordinary candidate as the first choice.
    bool all_explored = true;
    const std::vector<Vec2> empty;
    const std::vector<Vec2> unrelated{{100, 100}};
    const std::vector<Vec2> disabled{{6.25, 3.75}};
    for (const auto* blocked : {static_cast<const std::vector<Vec2>*>(nullptr), &empty, &unrelated}) {
        const auto path = select(grid, frontier(), current, blocked, .8, all_explored);
        require(path.size() == 2 && std::hypot(path.back().x - 6.25, path.back().y - 3.75) < 1e-12,
                "the blacklist change altered unblocked raw-cell scoring");
    }
    const auto path = select(grid, frontier(), current, &disabled, 0.0, all_explored);
    require(path.size() == 2 && std::hypot(path.back().x - 6.25, path.back().y - 3.75) < 1e-12,
            "zero blacklist radius no longer disables filtering");
}
}  // namespace

int main()
{
    four_corner_preimages();
    ordinary_candidate_survives();
    fallback_uses_the_same_coordinates();
    unblocked_selection_is_unchanged();
    std::cout << "coverage_blacklist_test: all checks passed\n";
}
