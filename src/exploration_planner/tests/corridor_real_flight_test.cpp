#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"
#define private public
#include "exploration_planner/corridor_controller.hpp"
#undef private
using namespace exploration;
namespace {
void require(bool ok,const std::string& text){if(!ok)throw std::runtime_error(text);}
double norm(Vec2 p){return std::hypot(p.x,p.y);}
double distance(Vec2 a,Vec2 b){return norm({a.x-b.x,a.y-b.y});}
CorridorConfig config() {
    CorridorConfig c;c.position_control=true;c.descend_before_entry=true;
    c.perception.robot_width=.64;c.perception.corridor_width=1.50;
    c.entry_speed=.40;c.cruise_speed=.45;c.gate_speed=.40;c.align_speed=.30;
    c.point_tolerance=.25;c.center_tolerance=.15;c.approach_distance=.80;
    c.exit_distance=.50;c.center_prediction_time=.40;c.cloud_window=5;c.settle_time=0;
    return c;
}
CorridorController seed() {
    auto cfg=config();CorridorController c(cfg);c.configure({0,0},{0,-6});c.start({0,0},0);
    c.phase_=CorridorPhase::Entry;c.heading_selected_=true;
    c.position_active_=c.position_ready_=true;c.position_waiting_=false;
    c.position_reference_={-.10,0};c.position_reference_velocity_={.05,0};
    for(int i=-100;i<=100;++i)c.points_.push_back({.64,i*.02});
    return c;
}
void heightIndependence() {
    auto a=seed();a.phase_=CorridorPhase::Search;a.position_height_admitted_=true;
    a.position_reference_={0,-.59};a.position_reference_velocity_={0,-.35};
    a.evidence_time_=1.;auto b=a;
    a.set_altitude_ready(true);b.set_altitude_ready(false);
    auto ca=a.update({0,0},0,0,0,1,true),cb=b.update({0,0},0,0,0,1,true);
    require(distance(ca.target,cb.target)<1e-12 && ca.status==cb.status,"in-flight Z error changes XY target");
    require(distance(cb.target,{0,0})>.58,"in-flight Z error withdrew forward target");
    auto first=seed();first.set_altitude_ready(false);
    require(first.update({0,0},0,0,0,1,true).status.find("initial corridor height")!=std::string::npos,
        "first handover skipped height check");
    b.positionFeedback(false,{0,0});
    require(b.update({0,0},0,0,0,1.02,true).status.find("main handover")!=std::string::npos,
        "height fix bypassed real control handover");
    std::cout<<"initial height still required; admitted XYZ flight independent of Z excursion\n";
}
void measuredBrakingAndRecovery() {
    auto a=seed(),b=a;a.filtered_velocity_={0,0};b.filtered_velocity_={.48,0};
    auto ca=a.positionCommand({-.30,0},{0,0},.4,.02),cb=b.positionCommand({-.30,0},{0,0},.4,.02);
    require(cb.target.x<ca.target.x,"real outward velocity did not brake entry target");
    auto r=seed();r.position_reference_={0,0};r.position_reference_velocity_={};
    const auto cr=r.positionCommand({.33,0},{0,0},.4,.02);
    require(r.positionRecovering() && .33-cr.target.x>.25,"overlap discarded safe 33cm inward pull");
    require(CorridorPerception::pathRecovering(r.points_,{.33,0},cr.target,.32),"recovery target not monotonically clear");
    std::cout<<"actual-velocity entry braking and retained safe recovery pull passed\n";
}
void voxelWindow() {
    auto cfg=config();cfg.cloud_window=.3;CorridorController c(cfg);
    c.observe({{0,0},{.001,0},{.04,0}},1);
    require(c.points().size()==2 && c.points()[0].x==.001,"latest sample not retained");
    c.observe({{.005,0}},1.2);c.observe({},1.31);
    require(c.points().size()==1 && c.points()[0].x==.005,"old support not expired independently");
    c.observe({},1.51);require(c.points().empty(),"window kept stale wall");
    c.observe({{2,2}},1.52);c.observe({{3,3}},.5);
    require(c.points().size()==1 && c.points()[0].x==3,"clock rollback retained previous wall");
    std::cout<<"incremental cloud window matches newest-sample/expiry/clock-reset contract\n";
}
void wall(Path2& cloud,Vec2 a,Vec2 b) {
    int n=std::ceil(distance(a,b)/.025);
    for(int i=0;i<=n;++i)cloud.push_back({a.x+(b.x-a.x)*i/n,a.y+(b.y-a.y)*i/n});
}
void delayedActualFlight(double mirror,double delay,double tau,double gain,double wind) {
    auto cfg=config();cfg.wall_entry=true;
    CorridorController c(cfg);
    // Real flight geometry: configured1.50m vs measured1.37m; alternate ~80cm doors.
    Path2 cloud;wall(cloud,{0,0},{6,0});wall(cloud,{-1.5,1.37},{6,1.37});
    wall(cloud,{1.83,0},{1.83,.57});wall(cloud,{4.19,.82},{4.19,1.37});
    for(auto& q:cloud)q.y*=mirror;
    Vec2 p{-.60,-1.25*mirror},v{},measured=p;
    c.configure({100,100},{5.4,.75*mirror});c.start(p,0);
    std::deque<Vec2> queue(static_cast<size_t>(std::round(delay/.02)),p);
    bool finished=false;double minimum=10,entry_max=-100;int resets=0;Vec2 prior=p;
    std::ofstream trace;
    if(const char* path=std::getenv("ACFLY_FLIGHT_TRACE")) {
        trace.open(path);trace<<"t,x,y,vx,vy,refx,refy,rvx,rvy,filtered_x,filtered_y,status\n";
    }
    for(int tick=1;tick<=10000;++tick) {
        const double t=tick*.02;
        if(tick%5==1)c.observe(cloud,t,p);
        if(tick%3==1)measured=p;
        if(c.positionActive())c.positionFeedback(true,p);
        c.set_altitude_ready(t<2 || std::sin(t*4.1)>0); // recurrent real-flight threshold crossings
        const auto cmd=c.update(measured,0,v.x,v.y,t,true);
        if(trace)trace<<t<<','<<p.x<<','<<p.y<<','<<v.x<<','<<v.y<<','<<cmd.target.x<<','<<cmd.target.y<<','
            <<c.position_reference_velocity_.x<<','<<c.position_reference_velocity_.y<<','
            <<c.filtered_velocity_.x<<','<<c.filtered_velocity_.y<<','<<cmd.status<<'\n';
        require(cmd.position_control,"position scenario changed control interface");
        require(std::isfinite(cmd.target.x)&&std::isfinite(cmd.target.y),"nonfinite target");
        if(distance(cmd.target,prior)>.10) {
            ++resets;
        }
        prior=cmd.target;
        queue.push_back(cmd.target);auto reference=queue.front();queue.pop_front();
        Vec2 desired{gain*(reference.x-p.x),gain*(reference.y-p.y)};
        if(std::abs(desired.x)<.025)desired.x=0;
        if(std::abs(desired.y)<.025)desired.y=0;
        const double speed=norm(desired);if(speed>.6){desired.x*=.6/speed;desired.y*=.6/speed;}
        double breeze=wind*mirror*(1+.5*std::sin(t*1.3));
        const double alpha=1-std::exp(-.02/tau);
        v.x+=alpha*(desired.x-v.x);v.y+=alpha*(desired.y+breeze-v.y);
        p.x+=v.x*.02;p.y+=v.y*.02;
        if(c.phase()==CorridorPhase::Entry)entry_max=std::max(entry_max,p.y*mirror);
        for(auto q:cloud)minimum=std::min(minimum,distance(p,q));
        require(minimum>.32,"collision t="+std::to_string(t)+" p="+std::to_string(p.x)+","+std::to_string(p.y)+
            " delay="+std::to_string(delay)+" tau="+std::to_string(tau)+" gain="+std::to_string(gain)+" status="+cmd.status);
        if(cmd.finished){finished=true;std::cout<<"flight mirror="<<mirror<<" delay="<<delay<<" tau="<<tau<<" gain="<<gain
            <<" wind="<<wind<<" finished="<<t<<" clearance="<<minimum-.32<<" entry_max="<<entry_max<<" jumps="<<resets<<'\n';break;}
    }
    require(finished && c.gatesPassed()==2,"flight stalled p="+std::to_string(p.x)+","+std::to_string(p.y)+" gates="+std::to_string(c.gatesPassed()));
    require(resets==0,"delayed flight generated a target jump above 10cm");
}
}
int main(){try{
    heightIndependence();measuredBrakingAndRecovery();voxelWindow();
    for(double mirror:{-1.,1.}) {
        delayedActualFlight(mirror,.30,.80,1.2,.025);
        delayedActualFlight(mirror,.50,1.20,.8,.02);
        delayedActualFlight(mirror,.50,1.20,1.2,-.02);
    }
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
