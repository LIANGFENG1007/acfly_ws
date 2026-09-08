#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "exploration_planner/visualizer.hpp"

namespace {

void check(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void add_wall(exploration::CorridorVisualState& corridor,
              exploration::Vec2 from, exploration::Vec2 to)
{
    const int count = std::max(1, static_cast<int>(std::ceil(
        std::hypot(to.x - from.x, to.y - from.y) / 0.025)));
    for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / count;
        corridor.points.push_back({from.x + t * (to.x - from.x),
                                   from.y + t * (to.y - from.y)});
    }
}

cv::Mat draw(exploration::Visualizer& visualizer,
             const exploration::GridSnapshot& grid,
             const exploration::CorridorVisualState& corridor,
             const exploration::Trajectory& trajectory = {},
             bool pose_valid = true)
{
    const exploration::Vec2 position = corridor.active
        ? exploration::Vec2{8.6, 2.45} : exploration::Vec2{3.0, 1.0};
    return visualizer.render(grid, trajectory, {7.0, 4.25}, true,
                             position.x, position.y, -M_PI / 2.0, pose_valid,
                             {8.6, 1.8}, false,
                             {}, {}, false, 1, false, {}, corridor);
}

int color_count(const cv::Mat& image, cv::Scalar color)
{
    cv::Mat mask;
    cv::inRange(image, color, color, mask);
    return cv::countNonZero(mask);
}

bool contains_color(const cv::Mat& image, cv::Scalar color)
{
    return color_count(image, color) > 0;
}

cv::Rect color_bounds(const cv::Mat& image, cv::Scalar color)
{
    cv::Mat mask;
    cv::inRange(image, color, color, mask);
    return cv::boundingRect(mask);
}

void check_status_layout(const cv::Mat& image)
{
    check(image.at<cv::Vec3b>(65, 0) == cv::Vec3b(28, 28, 28),
          "Status panel height changed between mission stages");
    check(image.at<cv::Vec3b>(66, 0) == cv::Vec3b(40, 40, 40),
          "Map margin or status panel background changed");
    for (const cv::Rect& row : {cv::Rect(8, 5, image.cols - 16, 15),
                               cv::Rect(8, 25, image.cols - 16, 15),
                               cv::Rect(8, 45, image.cols - 16, 15)}) {
        check(color_count(image(row), cv::Scalar(28, 28, 28)) < row.area(),
              "A mission stage is missing a status row");
    }
}

}  // namespace

int main(int argc, char** argv)
{
    using namespace exploration;
    const GridConfig cfg{-1.0, -5.0, 7.5, 5.0, 0.5, 0.05, 0.7, 100.0, 3.0};
    GridSnapshot grid;
    grid.cfg = cfg;
    grid.snx = 170;
    grid.sny = 200;
    grid.bnx = 17;
    grid.bny = 20;
    grid.cpb = 10;
    grid.cov = 0.25;
    grid.small.resize(grid.snx * grid.sny, 0);
    grid.big.resize(grid.bnx * grid.bny, 0);
    for (int i = 0; i < grid.bnx; ++i) {
        for (int j = 0; j < 5; ++j) grid.big[i * grid.bny + j] = 1;
    }

    CorridorVisualState corridor;
    corridor.configured = corridor.active = true;
    corridor.entry = {8.25, 4.25};
    corridor.h = {8.25, -4.25};
    corridor.width = 1.5;
    corridor.phase = "Crossing door";
    corridor.gate_valid = true;
    corridor.gate_left = {9.0, 1.8};
    corridor.gate_right = {8.2, 1.8};
    corridor.gate_center = {8.6, 1.8};
    corridor.route = {{8.6, 2.45}, {8.6, 1.8}, {8.6, 1.2}};
    add_wall(corridor, {7.5, 4.75}, {9.0, 4.75});
    add_wall(corridor, {9.0, 4.75}, {9.0, -4.75});
    add_wall(corridor, {9.0, -4.75}, {7.5, -4.75});
    add_wall(corridor, {7.5, 4.75}, {7.5, 4.6});
    add_wall(corridor, {7.5, 3.9}, {7.5, -4.75});
    // Synthetic gate positions for this preview only; both openings are 0.8 m.
    add_wall(corridor, {7.5, 1.8}, {8.2, 1.8});
    add_wall(corridor, {8.3, -1.3}, {9.0, -1.3});

    Visualizer visualizer(cfg, 640);
    const cv::Mat image = draw(visualizer, grid, corridor);
    check(image.cols <= 640 && image.rows <= 640, "Canvas exceeds configured maximum");
    check(image.cols < image.rows && image.rows == 640,
          "Vertical corridor did not preserve portrait canvas bounds");
    check(contains_color(image, cv::Scalar(0, 0, 255)), "Original endpoint is clipped");
    check(contains_color(image, cv::Scalar(0, 210, 140)), "Entry marker is clipped");
    check(contains_color(image, cv::Scalar(245, 245, 245)), "H marker is clipped");
    check(contains_color(image, cv::Scalar(225, 90, 240)), "Gate opening is missing");
    check(contains_color(image, cv::Scalar(60, 180, 70)), "Coverage grid disappeared");
    check(contains_color(image, cv::Scalar(225, 225, 225)), "Measured walls are missing");
    check_status_layout(image);

    const Trajectory exploration_trajectory = {{{1.0, 1.0}, 0, 0, 0},
        {{3.0, 1.0}, 0, 0, 2}, {{3.0, 3.0}, 0, 0, 4},
        {{5.5, 3.0}, 0, 0, 6.5}, {{7.0, 4.25}, 0, 0, 8.5}};
    CorridorVisualState preview = corridor;
    preview.active = false;
    preview.gate_valid = false;
    preview.phase.clear();
    preview.points.clear();
    preview.route.clear();
    const cv::Mat exploring = draw(visualizer, grid, preview, exploration_trajectory);
    const cv::Mat waiting = draw(visualizer, grid, preview, {}, false);
    for (const cv::Mat& stage_image : {waiting, exploring, image}) {
        check_status_layout(stage_image);
        check(stage_image.size() == image.size(),
              "Waiting, exploration, and corridor use different canvas dimensions");
        check(color_bounds(stage_image, cv::Scalar(0, 210, 140)) ==
              color_bounds(image, cv::Scalar(0, 210, 140)),
              "Entry coordinates or map scale changed at corridor handover");
        check(color_bounds(stage_image, cv::Scalar(245, 245, 245)) ==
              color_bounds(image, cv::Scalar(245, 245, 245)),
              "H coordinates or map scale changed at corridor handover");
    }
    check(contains_color(exploring, cv::Scalar(255, 185, 80)) &&
          contains_color(image, cv::Scalar(255, 185, 80)),
          "Exploration and corridor trajectories do not share the same visual style");
    check(color_count(waiting, cv::Scalar(225, 225, 225)) < 20 &&
          !contains_color(waiting, cv::Scalar(225, 90, 240)),
          "Route preview invented measured walls or a door opening");
    const cv::Mat stale_trajectory = draw(visualizer, grid, corridor, exploration_trajectory);
    check(cv::norm(stale_trajectory, image, cv::NORM_INF) == 0,
          "Completed exploration trajectory overlaps the active corridor trajectory");
    for (const std::string phase : {"Turning to corridor heading", "Moving to entry",
            "Searching for door", "Aligning with door center", "Crossing door",
            "Approaching H", "H reached", "Waiting for fresh odometry"}) {
        CorridorVisualState stage = corridor;
        stage.phase = phase;
        const cv::Mat stage_image = draw(visualizer, grid, stage);
        check_status_layout(stage_image);
        check(stage_image.size() == image.size(), "Corridor phase resized the map");
        check(cv::norm(stage_image.rowRange(66, stage_image.rows),
                       image.rowRange(66, image.rows), cv::NORM_INF) == 0,
              "Corridor phase moved world coordinates on screen");
    }

    CorridorVisualState no_cloud = corridor;
    no_cloud.points.clear();
    const cv::Mat no_cloud_image = draw(visualizer, grid, no_cloud);
    check(color_count(image, cv::Scalar(225, 225, 225)) >
          color_count(no_cloud_image, cv::Scalar(225, 225, 225)) + 100,
          "Measured walls do not follow supplied cloud points");
    check(no_cloud_image.size() == image.size(), "Cloud updates change map dimensions");

    CorridorVisualState invalid_cloud = corridor;
    invalid_cloud.points.push_back({std::numeric_limits<double>::quiet_NaN(), 0.0});
    invalid_cloud.points.push_back({1e6, 1e6});
    const cv::Mat invalid_cloud_image = draw(visualizer, grid, invalid_cloud);
    check(invalid_cloud_image.size() == image.size(), "Cloud outlier resized the map");

    CorridorVisualState horizontal = corridor;
    horizontal.h = {22.0, 4.25};
    horizontal.gate_valid = false;
    horizontal.points.clear();
    horizontal.route.clear();
    const cv::Mat horizontal_image = draw(visualizer, grid, horizontal);
    check(horizontal_image.cols == 640 && horizontal_image.rows <= 640,
          "Horizontal corridor exceeds canvas bounds");
    check(contains_color(horizontal_image, cv::Scalar(245, 245, 245)),
          "Horizontal H marker is clipped");

    const cv::Mat unconfigured = visualizer.render(grid, {}, {}, false, 0, 0, 0, false,
                                                   {}, false, {}, {}, false, 1, false, {});
    check_status_layout(unconfigured);
    check(unconfigured.cols <= 640 && unconfigured.rows == 640,
          "Unconfigured exploration canvas exceeds maximum size");
    check(contains_color(unconfigured.rowRange(66, unconfigured.rows), cv::Scalar(60, 180, 70)),
          "Unconfigured view lost the coverage grid below the status bar");
    for (const bool active : {false, true}) {
        CorridorVisualState no_route;
        no_route.active = active;
        no_route.phase = "Waiting for route";
        const cv::Mat stage_image = draw(visualizer, grid, no_route, exploration_trajectory);
        check_status_layout(stage_image);
        check(stage_image.size() == unconfigured.size(),
              "Unconfigured mission stages change canvas dimensions");
    }
    const std::string output = argc > 1 ? argv[1] : "/tmp/acfly-corridor-preview.png";
    check(cv::imwrite(output, image), "Could not write corridor preview");
    check(cv::imwrite("/tmp/acfly-unified-exploration.png", exploring),
          "Could not write unified exploration preview");
    check(cv::imwrite("/tmp/acfly-unified-corridor.png", image),
          "Could not write unified corridor preview");
    std::cout << "Corridor visualizer checks passed; preview: " << output << '\n';
    return 0;
}
