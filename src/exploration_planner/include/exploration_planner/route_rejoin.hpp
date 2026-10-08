#pragma once

#include <optional>
#include <algorithm>
#include <limits>
#include "exploration_planner/global_planner.hpp"

namespace exploration {

inline double route_distance(Vec2 point, const Path2& route)
{
    double best=std::numeric_limits<double>::infinity();
    for(size_t i=1;i<route.size();++i) {
        const Vec2 a=route[i-1],b=route[i];
        const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
        const double u=length2>1e-12 ? std::clamp(((point.x-a.x)*dx+(point.y-a.y)*dy)/length2,0.0,1.0) : 0.0;
        best=std::min(best,std::hypot(point.x-a.x-u*dx,point.y-a.y-u*dy));
    }
    return best;
}

inline bool outside_obstacle_rings(Vec2 point, const Obstacles& obstacles, const GlobalConfig& cfg)
{
    for(const auto& obstacle:obstacles)
        if(std::hypot(point.x-obstacle.cx,point.y-obstacle.cy)<
           obstacle.r+cfg.robot_radius+cfg.inflate-1e-9) return false;
    return true;
}

// Recover to the nearest reachable point on the retained route. Every segment
// must move strictly out of every ring containing the aircraft and must not
// enter any other ring. The perpendicular foot is preferred when it is safe;
// sampled alternatives handle an unsafe foot or overlapping obstacles.
inline std::optional<Vec2> nearest_safe_rejoin(Vec2 current, const Path2& route,
    const Obstacles& obstacles, const GlobalConfig& cfg, double max_distance=2.0)
{
    std::optional<Vec2> best;
    double best_distance=max_distance;
    const auto consider=[&](Vec2 point) {
        const double distance=std::hypot(point.x-current.x,point.y-current.y);
        if(distance<1e-6 || distance>best_distance+1e-9 ||
           !outside_obstacle_rings(point,obstacles,cfg) ||
           !obstacle_escape_clear(current,point,obstacles,cfg) ||
           !field_motion_clear(current,point,cfg)) return;
        if(!best || distance<best_distance-1e-9) {best=point;best_distance=distance;}
    };
    for(size_t i=1;i<route.size();++i) {
        const Vec2 a=route[i-1],b=route[i];
        const double dx=b.x-a.x,dy=b.y-a.y,len2=dx*dx+dy*dy;
        if(len2<1e-12) {consider(a);continue;}
        const double u=std::clamp(((current.x-a.x)*dx+(current.y-a.y)*dy)/len2,0.0,1.0);
        consider({a.x+u*dx,a.y+u*dy});
        const int count=std::max(1,static_cast<int>(std::ceil(std::sqrt(len2)/.05)));
        for(int k=0;k<=count;++k) {
            const double t=static_cast<double>(k)/count;
            consider({a.x+t*dx,a.y+t*dy});
        }
    }
    return best;
}

} // namespace exploration
