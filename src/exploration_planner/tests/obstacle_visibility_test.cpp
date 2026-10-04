#include "exploration_planner/obstacle_map.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace exploration;

void check(bool value,const char* message) {
    if(!value) throw std::runtime_error(message);
}

int main() {
    try {
        const ObstacleConfig cfg{-3,-3,3,3,.1,.05,2,2,4,1,1,.01,1,0,
                                 1,.4,0,120,4,3};
        for(double angle:{0.0,M_PI/2,-M_PI/2,M_PI-.01,-M_PI+.01}) {
            const Vec2 cell{2.05*std::cos(angle),2.05*std::sin(angle)};
            const auto contains=[&](const ObstacleMap& map) {
                for(const auto& p:map.occupied_points())
                    if(std::floor((p.x-cfg.min_x)/cfg.cell)==std::floor((cell.x-cfg.min_x)/cfg.cell) &&
                       std::floor((p.y-cfg.min_y)/cfg.cell)==std::floor((cell.y-cfg.min_y)/cfg.cell)) return true;
                return false;
            };
            // Dense background rays supply positive visibility, including
            // returns outside the map; those are not inserted as obstacles.
            std::vector<Vec2> background;
            for(int i=-100;i<=100;++i) {
                const double a=angle+i*.002;
                background.push_back({4.8*std::cos(a),4.8*std::sin(a)});
            }
            ObstacleMap map(cfg);
            map.integrate({cell});
            check(contains(map),"test cell did not latch");
            for(int frame=0;frame<12;++frame)
                map.integrate({{4.8*std::cos(angle+.5),4.8*std::sin(angle+.5)}},0,0,angle,true);
            check(contains(map),"unrelated returns erased a cell just because it was inside FOV");
            for(int frame=0;frame<12;++frame) map.integrate({},0,0,angle,true);
            check(contains(map),"empty observations erased unseen static obstacle");
            for(int frame=0;frame<12;++frame)
                map.integrate({{4.8*std::cos(angle),4.8*std::sin(angle)}},0,0,angle,true);
            check(contains(map),"one sparse ray incorrectly cleared an entire occupied cell");

            auto occluded=background;
            occluded.push_back({std::cos(angle),std::sin(angle)});
            for(int frame=0;frame<12;++frame) map.integrate(occluded,0,0,angle,true);
            check(contains(map),"far returns bypassed a foreground occluder");
            for(int frame=0;frame<12;++frame) map.integrate(background,0,0,angle+M_PI,true);
            check(contains(map),"out-of-FOV cell was cleared");
            const double nan=std::numeric_limits<double>::quiet_NaN();
            const double inf=std::numeric_limits<double>::infinity();
            for(int frame=0;frame<12;++frame) map.integrate({{nan,0},{inf,0}},0,0,angle,true);
            check(contains(map),"invalid returns supplied clearing evidence");

            // Real hits reset all prior contradictory observations. A brief
            // dropout may not accumulate across repeated valid detections.
            auto live=background;live.push_back(cell);
            for(int cycle=0;cycle<5;++cycle) {
                map.integrate(background,0,0,angle,true);
                map.integrate(background,0,0,angle,true);
                map.integrate(live,0,0,angle,true);
            }
            check(contains(map),"live obstacle accumulated ghost misses across hits");
            for(int frame=0;frame<cfg.ghost_clear_frames;++frame)
                map.integrate(background,0,0,angle,true);
            check(contains(map),"cell cleared before configured number of free observations");
            map.integrate(background,0,0,angle,true);
            check(!contains(map),"confirmed empty cell was never reclaimed");
        }
        std::cout<<"occlusion, sparse/empty cloud, nearest-return visibility, free-space clearing and angle wrap passed\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<e.what()<<'\n';return 1;
    }
}
