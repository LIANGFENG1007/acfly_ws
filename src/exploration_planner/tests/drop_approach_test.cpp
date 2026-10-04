#include "exploration_planner/drop_approach.hpp"
#include <iostream>
#include <stdexcept>

using namespace exploration;
void check(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
double length(const Path2& path) {
    double n = 0;
    for (size_t i=1;i<path.size();++i) n += std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    return n;
}
// 独立于线段投影实现的密集采样 oracle，检查整条通路，而非只检查返回点。
void verify_line(Vec2 a, Vec2 b, const Obstacles& obs, double body, double margin) {
    const int samples = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/.003)));
    for (int k=0;k<=samples;++k) {
        const double u=double(k)/samples;
        for (const auto& o:obs) check(std::hypot(a.x+u*(b.x-a.x)-o.cx,a.y+u*(b.y-a.y)-o.cy)
            >= o.r+body+margin-1e-8,"returned route crosses an obstacle");
    }
}
int main() {
    try {
        GlobalConfig descent{-1,-3,8,3,.10,.4,.1,.4};
        auto transit=descent;
        const Vec2 start{0,0}, goal{4,0};
        auto direct=plan_drop_approach(start,goal,goal,{},transit,descent,1.0);
        check(direct.ok && std::hypot(direct.entry.x-3,direct.entry.y)<1e-12,
              "empty scene did not fly directly to the nearest point on the 1m circle");
        const auto already_near=plan_drop_approach({3.5,0},goal,goal,{},transit,descent,1.0);
        check(already_near.ok && std::abs(already_near.entry.x-3.5)<1e-12,
              "already-near target caused an unnecessary retreat to the 1m circle");
        Obstacles sealed_circle;
        for(int i=0;i<8;++i) sealed_circle.push_back({4+std::cos(i*M_PI/4),std::sin(i*M_PI/4),.05});
        const auto sealed=plan_drop_approach(start,goal,goal,sealed_circle,transit,descent,1.0);
        check(!sealed.ok && std::string(sealed.reason)=="no_safe_point_on_entry_circle",
              "fully blocked 1m circle silently expanded the sampling radius");
        const Obstacles one{{2,0,.35}};
        check(drop_descent_clear({1,0},goal,{{4,.61,.1}},descent,.1),
              "entry uncertainty unnecessarily enlarged the precise destination");
        check(!drop_descent_clear({1,0},goal,{{1,.65,.1}},descent,.1),
              "entry uncertainty was ignored near the sampling point");
        check(drop_descent_clear(start,start,one,transit) &&
              drop_descent_clear(goal,goal,one,transit) &&
              !drop_descent_clear(start,goal,one,descent),"unsafe-endpoints-only fixture is invalid");
        auto detour=plan_drop_approach(start,goal,goal,one,transit,descent,1.0);
        check(detour.ok,"target behind single obstacle has no entry");
        verify_line(detour.entry,goal,one,.4,.1);
        for(size_t i=1;i<detour.path.size();++i) verify_line(detour.path[i-1],detour.path[i],one,.4,.1);
        // A known nearby entry is enough. A farther, wider point must not win a safety score.
        const Vec2 nearby{1.5,1.25};
        check(drop_descent_clear(start,nearby,one,transit) &&
              drop_descent_clear(nearby,goal,one,descent),"nearest-point comparison fixture");
        const Vec2 known_circle_entry{3.2,.6};
        verify_line(nearby,known_circle_entry,one,.4,.1);
        const double reference=std::hypot(nearby.x,nearby.y)+std::hypot(nearby.x-3.2,nearby.y-.6);
        check(std::abs(std::hypot(detour.entry.x-goal.x,detour.entry.y-goal.y)-1)<1e-9,
              "obstacle case selected a distant descent entry outside the 1m circle");
        check(length(detour.path)<=reference+.1,
              "chose a far safe region despite an already-safe nearby entry");
        const Obstacles two{{2,0,.35},{detour.entry.x,detour.entry.y,.18}};
        auto other_side=plan_drop_approach(start,goal,goal,two,transit,descent,1.0);
        check(other_side.ok,"second obstacle prevented the valid opposite-side entry");
        verify_line(other_side.entry,goal,two,.4,.1);
        for(size_t i=1;i<other_side.path.size();++i) verify_line(other_side.path[i-1],other_side.path[i],two,.4,.1);
        check(!plan_drop_approach(start,goal,goal,{{4,0,.2}},transit,descent,1.0).ok,
              "occupied alignment point fell back to an unchecked object point");
        check(!plan_drop_approach(start,goal,goal,{{4,0,.2}},transit,descent,1.0,.1,0).ok,
              "terminal margin relaxation removed the aircraft footprint");
        check(drop_descent_clear(start,goal,{{4,.55,.1}},descent,.1,0),
              "physically clear endpoint inside only the outer margin was rejected");
        check(!drop_descent_clear(start,goal,{{2,.55,.1}},descent,.1,0),
              "terminal exception relaxed an unrelated obstacle in the middle of descent");
        // 实际编辑场景第三次投放：规划圈比机体可用空间多压入约3cm，旧版直接封死目标。
        GlobalConfig scene{-.5,-3.8,9,4.6,.04,.4,.1,.6};
        auto scene_transit=scene;scene_transit.required_goal_field_margin=.5;scene_transit.wall_margin=.5;
        const Vec2 scene_start{5.097,3.7299},scene_goal{3.76334815524,2.33038765984};
        const Obstacles scene_obs{{2.975,2.55833333333,.353885312596},
                                 {5.11600252960,2.79416524759,.358003347095}};
        check(!plan_drop_approach(scene_start,{3.75,2.56},scene_goal,scene_obs,scene_transit,scene,1.0,.1).ok,
              "recorded blocked-endpoint fixture no longer exercises the outer ring");
        const auto scene_route=plan_drop_approach(scene_start,{3.75,2.56},scene_goal,scene_obs,scene_transit,scene,1.0,.1,0);
        check(scene_route.ok && scene_route.entry.x<scene_start.x && scene_route.entry.y>scene_goal.y,
              "recorded scene missed the nearby approach from above the target");
        check(length(scene_route.path)<1.6,"recorded scene went around the far side unnecessarily");
        verify_line(scene_route.entry,scene_goal,scene_obs,.4,0);
        for(int perturb=0;perturb<24;++perturb) {
            const double a=perturb*M_PI/12;
            verify_line({scene_route.entry.x+.1*std::cos(a),scene_route.entry.y+.1*std::sin(a)},
                        scene_goal,scene_obs,.4,0);
        }
        for(size_t i=1;i<scene_route.path.size();++i)
            verify_line(scene_route.path[i-1],scene_route.path[i],scene_obs,.4,.1);
        check(!plan_drop_approach(start,{7.8,0},{7.8,0},{},transit,descent,1.0).ok,
              "package-compensated endpoint outside body-clearance boundary accepted");
        const Obstacles barrier{{2,-2.4,.5},{2,-1.6,.5},{2,-.8,.5},{2,0,.5},
                                {2,.8,.5},{2,1.6,.5},{2,2.4,.5}};
        check(!plan_drop_approach(start,goal,goal,barrier,transit,descent,1.0).ok,"sealed wall produced a descent route");
        // Package direction and yaw can put the endpoint on different sides of an obstacle.
        size_t checked=0;
        for(int package=0;package<3;++package) for(int angle=0;angle<12;++angle) {
            const double yaw=angle*M_PI/6, c=std::cos(yaw),s=std::sin(yaw);
            const double f=package==0?.23:0, l=package==1?.23:package==2?-.23:0;
            const Vec2 lidar{goal.x-c*f+s*l,goal.y-s*f-c*l};
            const auto plan=plan_drop_approach(start,goal,lidar,one,transit,descent,1.0,.1);
            check(plan.ok,"reachable package/yaw case lost its safe entry");
            check(std::abs(std::hypot(plan.entry.x-goal.x,plan.entry.y-goal.y)-1)<1e-9,
                  "package offset moved the entry circle away from the visual object");
            verify_line(plan.entry,lidar,one,.4,.1);
            for(int perturb=0;perturb<12;++perturb) {
                const double a=perturb*M_PI/6;
                verify_line({plan.entry.x+.1*std::cos(a),plan.entry.y+.1*std::sin(a)},lidar,one,.4,.1);
            }
            for(size_t i=1;i<plan.path.size();++i) verify_line(plan.path[i-1],plan.path[i],one,.4,.1);
            ++checked;
        }
        // 入场方向必须来自路线：第三包裹从右下侧接近，有近侧直达入口，不能
        // 因假设到点要面对物体而排除它、绕到上侧再折返。
        const Vec2 incoming_start{6.257,1.625}, object{3.75,2.56};
        const Obstacles fitted{{2.975,2.558333333333,.353885312596},{5.116,2.794166666667,.358005}};
        const auto compensated=[&](Vec2 entry,Vec2 from) {
            const double yaw=std::atan2(entry.y-from.y,entry.x-from.x);
            return Vec2{object.x-.23*std::sin(yaw),object.y+.23*std::cos(yaw)};
        };
        const auto joint=plan_drop_approach(incoming_start,object,object,fitted,scene_transit,scene,1,.1,0,compensated);
        check(joint.ok && joint.entry.x>object.x && joint.entry.y<object.y && length(joint.path)<2.0,
              "route/heading coupling missed the near-side descent and went around the target");
        const Vec2 from=joint.path[joint.path.size()-2], landing=compensated(joint.entry,from);
        check(std::abs(std::remainder(joint.arrival_heading-std::atan2(joint.entry.y-from.y,joint.entry.x-from.x),2*M_PI))<1e-9,
              "returned heading does not belong to the checked final approach");
        verify_line(joint.entry,landing,fitted,.4,0);
        for(size_t i=1;i<joint.path.size();++i) verify_line(joint.path[i-1],joint.path[i],fitted,.4,.1);
        // Recorded false terminal: overshoot an entry,
        // then use a few-centimetre reverse edge to manufacture a payload yaw.
        // Check both handednesses, including the payload's left/right mirror.
        for(int side:{-1,1}) {
            GlobalConfig cfg{-.5,-4.6,9,4.6,.04,.4,.1,.6};
            auto transit=cfg;transit.required_goal_field_margin=.6;transit.wall_margin=.5;
            const Vec2 cur{3.289294140841054,side*1.3229144165042386},target{3.75,side*2.56};
            const Obstacles obs{{2.9345650013708715,side*2.468140002744494,.3011013798455387},
                                 {5.049109761363637,side*2.783991738636363,.3687324865802285}};
            const auto landing=[&](Vec2 p,Vec2 from) {
                const double yaw=std::atan2(p.y-from.y,p.x-from.x);
                return Vec2{target.x-side*.23*std::sin(yaw),target.y+side*.23*std::cos(yaw)};
            };
            for(double terminal_margin:{-1.0,0.0}) {
                const auto route=plan_drop_approach(cur,target,target,obs,transit,cfg,1,.1,terminal_margin,landing);
                // The strict-only search may reject the former fictitious yaw
                // route; the near-side route under the actual terminal rule must exist.
                if(!route.ok && terminal_margin<0) continue;
                check(route.ok,"recorded terminal has a reachable entry");
                for(size_t i=1;i<route.path.size();++i) {
                    verify_line(route.path[i-1],route.path[i],obs,.4,.1);

                }
                const auto from=route.path[route.path.size()-2];
                if(route.path.size()>2) {
                    const auto before=route.path[route.path.size()-3];
                    check((from.x-before.x)*(route.entry.x-from.x)+(from.y-before.y)*(route.entry.y-from.y)>=-1e-9,
                          "terminal inserted a reverse edge to fabricate payload heading");
                }
                check(drop_descent_clear(route.entry,landing(route.entry,from),obs,cfg,.1,terminal_margin),
                      "returned connector uses a different payload heading than the validated descent");
                if(terminal_margin==0) check(route.path.size()==2 && length(route.path)<.7,
                    "near legal entry lost to the longer fully padded route");
            }
        }
        auto zero=transit;zero.cell=0;
        check(!plan_drop_approach(start,goal,goal,one,zero,descent,1.0).ok,"invalid grid size accepted");
        std::cout<<"Nearest 1m circle entry, full descent segment, multiple obstacles, blocked/boundary targets and "
                 <<checked<<" package/yaw cases passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
