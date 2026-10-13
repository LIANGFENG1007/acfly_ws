#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <queue>
#include <vector>
#include "exploration_planner/global_planner.hpp"

namespace exploration {

struct DropApproach {
    bool ok = false;
    Vec2 entry{};
    Path2 path;
    const char* reason = "no_reachable_entry";
    double arrival_heading = std::numeric_limits<double>::quiet_NaN();
};

// 严格验证整条斜降的 XY 包络。两端安全不代表线段安全；所有障碍一起检查。
// cfg.inflate 是额外余量，障碍自身的拟合充气已经包含在 obs.r 中。
inline bool drop_descent_clear(Vec2 a, Vec2 b, const Obstacles& obs, const GlobalConfig& cfg,
                               double start_allowance = 0, double endpoint_inflate = -1)
{
    if (!std::isfinite(start_allowance) || start_allowance < 0) return false;
    if (!std::isfinite(endpoint_inflate) || (endpoint_inflate != -1 &&
        (endpoint_inflate < 0 || endpoint_inflate > cfg.inflate))) return false;
    const double inset = required_field_margin(cfg);
    int endpoint = 0;
    for (const Vec2 p : {a, b}) {
        const double margin = inset + (endpoint++ == 0 ? start_allowance : 0);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            p.x < cfg.min_x + margin || p.x > cfg.max_x - margin ||
            p.y < cfg.min_y + margin || p.y > cfg.max_y - margin) return false;
    }
    const double dx = b.x-a.x, dy = b.y-a.y, length2 = dx*dx+dy*dy;
    for (const auto& ob : obs) {
        if (!std::isfinite(ob.cx) || !std::isfinite(ob.cy) || !std::isfinite(ob.r) || ob.r < 0) return false;
        // 普通巡航的外层规划余量不能把可容纳机体的投放位置判死。
        // 仅对“最终对准点进入外层余量”的障碍，沿斜降线逐渐收小额外余量，
        // 终点仍保留执行余量+完整机体半径+拟合障碍轮廓；其他障碍保持原余量。
        const bool terminal_margin = endpoint_inflate >= 0 &&
            std::hypot(b.x-ob.cx,b.y-ob.cy) < ob.r+cfg.robot_radius+cfg.inflate;
        const double final_inflate = terminal_margin ? endpoint_inflate : cfg.inflate;
        const double r = ob.r + cfg.robot_radius + final_inflate;
        const double allowance = start_allowance + cfg.inflate-final_inflate;
        // 入口可能有到点误差：检查入口误差圆到精确终点的整束直线。
        // 半径余量随进度线性缩小，而非给最终目标也硬加一个无法到达的余量。
        // |a+u*d-center|²-(r+allowance*(1-u))² 是二次式；最小值只在端点或凸顶点。
        if (dx == 0.0 && dy == 0.0 && allowance == 0.0) {
            if (std::hypot(ob.cx-a.x,ob.cy-a.y) < r-1e-9) return false;
            continue;
        }
        const double qa = length2-allowance*allowance;
        const double qb = 2*((a.x-ob.cx)*dx+(a.y-ob.cy)*dy+(r+allowance)*allowance);
        const double vertex = qa > 1e-12 ? std::clamp(-qb/(2*qa),0.0,1.0) : 0.0;
        for(double u : {0.0,vertex,1.0})
            if (std::hypot(ob.cx-a.x-u*dx,ob.cy-a.y-u*dy) < r+allowance*(1-u)-1e-9) return false;
    }
    return true;
}

// 在以视觉物体为圆心的 entry_radius 圆周上选入口；包裹偏移只改变下降终点，
// 不改变这个圆的圆心。安全是准入条件，比较的是飞机到入口的可行路程。
// 圆周候选作为同一次 Dijkstra 的终点，不能先选一个远处可下降点再接第二段。
inline DropApproach plan_drop_approach(Vec2 start, Vec2 object, Vec2 lidar_goal,
                                      const Obstacles& obs, GlobalConfig transit,
                                      const GlobalConfig& descent, double entry_radius,
                                      double entry_tolerance = 0, double endpoint_inflate = -1,
                                      const std::function<Vec2(Vec2,Vec2)>& landing_at = {})
{
    DropApproach result;
    result.reason = "no_reachable_entry_circle";
    const double inset = required_field_margin(transit);
    const double xmin = transit.min_x+inset, ymin = transit.min_y+inset;
    const double width = transit.max_x-inset-xmin, height = transit.max_y-inset-ymin;
    if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
        !std::isfinite(object.x) || !std::isfinite(object.y) ||
        !std::isfinite(entry_radius) || entry_radius <= 0 ||
        !std::isfinite(transit.cell) || transit.cell <= 0 || !std::isfinite(width) ||
        !std::isfinite(height) || width < 0 || height < 0 ||
        (width/transit.cell+1)*(height/transit.cell+1) > 250000) {
        result.reason = "invalid_search_geometry"; return result;
    }
    auto endpoint = descent;
    if (endpoint_inflate >= 0) endpoint.inflate = endpoint_inflate;
    if (!landing_at && !drop_descent_clear(lidar_goal,lidar_goal,obs,endpoint)) {
        result.reason = "alignment_point_blocked"; return result;
    }
    const auto acceptable = [&](Vec2 p, Vec2 from) {
        return drop_descent_clear(p,p,obs,transit) &&
            drop_descent_clear(p,landing_at ? landing_at(p,from) : lidar_goal,obs,descent,entry_tolerance,endpoint_inflate);
    };
    const auto first_clear = [&](Vec2 p) {
        return field_motion_clear(start,p,transit) && obstacle_escape_clear(start,p,obs,transit);
    };
    const double object_distance = std::hypot(start.x-object.x,start.y-object.y);
    // 已经在1米以内且整条下降线可用时，就在当前位置采样，不为凑半径反向退出。
    if (object_distance <= entry_radius && acceptable(start,start))
        return {true,start,{start,start},"current_point_inside_circle"};
    const double angle0 = std::atan2(start.y-object.y,start.x-object.x);
    const Vec2 nearest{object.x+entry_radius*std::cos(angle0),object.y+entry_radius*std::sin(angle0)};
    if (acceptable(nearest,start) && first_clear(nearest))
        return {true,nearest,{start,nearest},"direct_circle_entry",std::atan2(nearest.y-start.y,nearest.x-start.x)};

    const double arc_step = std::max(.005,std::min(.05,transit.cell));
    const double count = std::ceil(2*M_PI*entry_radius/arc_step);
    if (count > 4096) { result.reason = "invalid_search_geometry"; return result; }
    const int samples = std::max(16,static_cast<int>(count));
    std::vector<Vec2> candidates;
    for (int i=0;i<samples;++i) {
        const double angle = angle0+2*M_PI*i/samples;
        const Vec2 p{object.x+entry_radius*std::cos(angle),object.y+entry_radius*std::sin(angle)};
        // 到达航向来自连接到入口的最后一条边。不能预设必须正对物体，
        // 也不能在尚未知道最后一条边时排除该入口。
        if (landing_at ? drop_descent_clear(p,p,obs,transit) : acceptable(p,start)) candidates.push_back(p);
    }
    if (candidates.empty()) { result.reason = "no_safe_point_on_entry_circle"; return result; }
    const int nx = static_cast<int>(std::floor(width/transit.cell))+1;
    const int ny = static_cast<int>(std::floor(height/transit.cell))+1;
    const int n = nx*ny, total = n+static_cast<int>(candidates.size());
    const auto point = [&](int id) {
        return id<n ? Vec2{xmin+(id%nx)*transit.cell,ymin+(id/nx)*transit.cell} : candidates[id-n];
    };
    std::vector<unsigned char> free(total,1);
    std::vector<double> distance(total,std::numeric_limits<double>::infinity());
    std::vector<int> parent(total,-2);
    using Item = std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    // 起点直接连到所有可见格点/圆周点；不能把起点强吸到障碍另一侧的格子。
    for (int id=0;id<total;++id) {
        const Vec2 p=point(id);
        if (id<n) free[id]=drop_descent_clear(p,p,obs,transit);
        if (free[id] && first_clear(p) && (id<n || acceptable(p,start))) {
            distance[id]=std::hypot(p.x-start.x,p.y-start.y);
            parent[id]=-1;queue.emplace(distance[id],id);
        }
    }
    // 精确圆周点不落在格子中心：显式连接附近格点，保留1米半径而非吸到格子上。
    std::vector<std::vector<int>> terminal_links(n);
    for (int id=n;id<total;++id) {
        const Vec2 p=point(id);
        // 可直达的入口只能使用真实直达方向评估包裹。不能人为添加最后几厘米
        // 的折线来“凑”一个安全yaw；这种理论捷径会在实飞时变成第二段横移。
        if (landing_at && first_clear(p)) continue;
        const int x=static_cast<int>(std::floor((p.x-xmin)/transit.cell));
        const int y=static_cast<int>(std::floor((p.y-ymin)/transit.cell));
        for (int dy=-1;dy<=2;++dy) for (int dx=-1;dx<=2;++dx) {
            if (x+dx<0 || x+dx>=nx || y+dy<0 || y+dy>=ny) continue;
            const int grid=(y+dy)*nx+x+dx;
            if (free[grid] && drop_descent_clear(point(grid),p,obs,transit))
                terminal_links[grid].push_back(id);
        }
    }
    const auto relax = [&](int id,int next) {
        if (!free[next]) return;
        const Vec2 p=point(id),q=point(next);
        int previous=id;
        double cost=distance[id]+std::hypot(q.x-p.x,q.y-p.y);
        // Ordinary-grid cost cannot change below; an already dominated edge
        // cannot update distance/parent even when its geometry is clear.
        if (next<n && cost>=distance[next]-1e-9) return;
        if (!drop_descent_clear(p,q,obs,transit)) return;
        if (next>=n) {
            // 终点的父节点就是核准包裹朝向所用的入场点。必须一起存入图，
            // 不能用远处父节点验算yaw，却把圆周旁几厘米的格子边作为实际末段。
            Vec2 from=p;
            for(int ancestor=parent[id];ancestor>=0;ancestor=parent[ancestor]) {
                if(drop_descent_clear(point(ancestor),q,obs,transit)) {
                    from=point(ancestor);previous=ancestor;
                }
            }
            if(first_clear(q)) { from=start;previous=-1; }
            if(previous>=0) {
                // 还没到入场点就已同时落进入口到点范围，不能把这样短的边
                // 当成必须转向的新航段来证明包裹位置安全。
                if(std::hypot(q.x-from.x,q.y-from.y)<=entry_tolerance) return;
                Vec2 before=start;
                for(int ancestor=parent[previous];ancestor>=0;ancestor=parent[ancestor]) {
                    if(drop_descent_clear(point(ancestor),from,obs,transit)) before=point(ancestor);
                }
                if(first_clear(from)) before=start;
                // 不允许为凑包裹yaw，在入口前先越过它再用一条反向短边回来。
                if((from.x-before.x)*(q.x-from.x)+(from.y-before.y)*(q.y-from.y)<-1e-9) return;
            }
            if(!acceptable(q,from)) return;
            cost=(previous<0 ? 0.0 : distance[previous])+std::hypot(q.x-from.x,q.y-from.y);
        }
        if(cost>=distance[next]-1e-9) return;
        distance[next]=cost;parent[next]=previous;queue.emplace(cost,next);
    };
    int found=-1;
    while (!queue.empty()) {
        const auto [cost,id]=queue.top();queue.pop();
        if (cost > distance[id]+1e-9) continue;
        if (id>=n) { found=id;break; }
        const int x=id%nx,y=id/nx;
        for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx) {
            if ((!dx && !dy) || x+dx<0 || x+dx>=nx || y+dy<0 || y+dy>=ny) continue;
            relax(id,(y+dy)*nx+x+dx);
        }
        for (const int next:terminal_links[id]) relax(id,next);
    }
    if (found<0) return result;
    Path2 reverse;
    for (int id=found;id>=0;id=parent[id]) reverse.push_back(point(id));
    reverse.push_back(start);
    Path2 route(reverse.rbegin(),reverse.rend());
    // 只简化入场点之前的路线；最后一条边已连同包裹偏移核准，不能再次
    // 简化成另一条入场方向，也不能在旧方向不通过时偷偷保留一个极短折返边。
    const size_t last_anchor=route.size()-2;
    Path2 shortened{start};
    for (size_t at=0;at<last_anchor;) {
        size_t next=at+1;
        for (size_t j=last_anchor+1;j-->at+1;) {
            if (at==0 ? first_clear(route[j]) : drop_descent_clear(route[at],route[j],obs,transit)) {
                next=j;break;
            }
        }
        shortened.push_back(route[next]);at=next;
    }
    if(!acceptable(route.back(),shortened.back())) {
        result.reason="invalid_arrival_connector";return result;
    }
    shortened.push_back(route.back());
    result.ok=true;result.path=std::move(shortened);result.entry=result.path.back();
    result.reason="nearest_reachable_circle";
    const Vec2 incoming=result.path[result.path.size()-2];
    result.arrival_heading=std::atan2(result.entry.y-incoming.y,result.entry.x-incoming.x);
    return result;
}

}  // namespace exploration
