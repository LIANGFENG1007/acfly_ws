#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"
#include "exploration_planner/params.hpp"
// Seed the measured failure state without pretending to replay unrecorded LiDAR frames.
#define private public
#include "exploration_planner/corridor_controller.hpp"
#undef private

using namespace exploration;
namespace {
constexpr double dt=.02;
void require(bool v,const std::string& message) {if(!v)throw std::runtime_error(message);}
double norm(Vec2 p){return std::hypot(p.x,p.y);}
CorridorConfig config() {
    CorridorConfig cfg;cfg.position_control=true;cfg.settle_time=0;
    cfg.perception.robot_width=.64;cfg.center_tolerance=.15;cfg.approach_distance=.8;
    cfg.entry_speed=params::CORRIDOR_ENTRY_SPEED;cfg.cruise_speed=params::CORRIDOR_CRUISE_SPEED;
    cfg.gate_speed=params::CORRIDOR_GATE_SPEED;
    cfg.center_prediction_time=.4;cfg.exit_distance=.5;return cfg;
}
void seed(CorridorController& c,Vec2 position,bool gate) {
    c.configure({0,0},{0,-6});c.start(position,0);
    c.position_active_=c.position_ready_=true;c.position_waiting_=false;
    c.position_reference_=position;c.phase_=CorridorPhase::Search;
    c.gate_locked_=gate;c.position_gate_approach_=gate;
    c.locked_gate_.gate_s=2;c.locked_gate_.gate_depth=.04;
    c.locked_gate_.gap_left=-.392767;c.locked_gate_.gap_right=.392767;
    c.locked_gate_.gap_width=.785534;
    for(int i=-20;i<=260;++i) {
        c.points_.push_back({-.75,-i*.025});c.points_.push_back({.75,-i*.025});
    }
    if(gate)for(int i=-30;i<=30;++i)if(std::abs(i*.025)>=.392767)c.points_.push_back({i*.025,-2});
}
void geometricRecovery() {
    const Path2 wall{{0,-1},{0,-.1},{0,0},{0,.1},{0,1}};
    require(CorridorPerception::pathRecovering(wall,{.2,0},{.5,0},.32),"away from overlapping wall rejected");
    require(!CorridorPerception::pathRecovering(wall,{.2,0},{.1,0},.32),"recovery moved into wall");
    auto trapped=wall;trapped.push_back({.7,0});
    require(!CorridorPerception::pathRecovering(trapped,{.2,0},{.5,0},.32),"recovery entered opposite wall");
    require(!CorridorPerception::pathRecovering({}, {.2,0},{.5,0},.32),"empty cloud authorized recovery");
    require(!CorridorPerception::pathRecovering(wall,{.2,0},{.2,0},.32),"stationary overlap marked clear");
    std::cout<<"monotonic escape and opposite-wall veto passed\n";
}
void amplifiedTarget() {
    for(bool gate:{false,true})
    for(double side:{-1.,1.}) {
        CorridorController c(config());const Vec2 p{side*.04,-1.3};seed(c,p,gate);
        Vec2 last=p,last_v{};CorridorCommand cmd;
        for(int i=0;i<400;++i) {
            cmd=c.forwardCorridor(p,0,0,0,dt);
            require(cmd.position_control && cmd.forward==0 && cmd.lateral==0,"not pure position interface");
            Vec2 v{(cmd.target.x-last.x)/dt,(cmd.target.y-last.y)/dt};
            require(norm({v.x-last_v.x,v.y-last_v.y})/dt<=config().position_acceleration+1e-6,"reference jumped during correction");
            if(gate)require(std::abs(cmd.target.x)<=.053,"target placed in opposite door frame");
            last=cmd.target;last_v=v;
        }
        require(side*cmd.target.x<-.025,"position target failed to extend across centerline");
        // Returning velocity must retract the request before the vehicle crosses the centerline.
        c.filtered_velocity_={-side*.15,0};
        for(int i=0;i<300;++i)cmd=c.forwardCorridor(p,0,0,0,dt);
        require(side*cmd.target.x>0,"returning lateral speed did not retract side extension");
    }
    std::cout<<"travel line AND gate: left/right diagonal position extension, retraction and smoothness passed\n";
}
void revokeCrossing() {
    CorridorController c(config());seed(c,{0,-1.3},true);
    c.forwardCorridor({0,-1.3},0,0,0,dt);
    require(c.position_gate_aligned_,"centered gate never admitted crossing");
    CorridorCommand cmd;
    for(int i=0;i<300;++i)cmd=c.forwardCorridor({.12,-1.3},0,0,0,dt);
    require(!c.position_gate_aligned_,"12cm drift kept crossing latched");
    const double staging=2-.02-.32-config().continuous_center_reserve;
    require(-cmd.target.y<=staging+.01,"off-center request advanced past door staging limit");
    std::cout<<"crossing permission revoked before unsafe drift continued forward\n";
}
void recoverDynamics(bool atDoor,double side) {
    const auto cfg=config();CorridorController c(cfg);
    Vec2 p{side*(atDoor?.15:.50),atDoor?-1.86:-.6},v{};seed(c,p,atDoor);
    bool recovered=false,resumed=false;int blocked=0;double start_s=-p.y;
    for(int i=0;i<1800;++i) {
        c.filtered_velocity_=v;
        const auto cmd=c.forwardCorridor(p,0,v.x,v.y,dt);
        require(std::isfinite(cmd.target.x)&&std::isfinite(cmd.target.y),"recovery generated invalid position");
        if(cmd.status.find("blocked by")!=std::string::npos)++blocked;
        // Slow independent position response, not the controller's old velocity PID.
        Vec2 desired{.65*(cmd.target.x-p.x),.65*(cmd.target.y-p.y)};
        v.x+=(1-std::exp(-dt/.4))*(desired.x-v.x);
        v.y+=(1-std::exp(-dt/.4))*(desired.y-v.y);
        p.x+=v.x*dt;p.y+=v.y*dt;
        if(std::abs(p.x)<.03 && CorridorPerception::pathClear(c.points_,p,p,.32))recovered=true;
        if(recovered && -p.y>start_s+.6){resumed=true;break;}
    }
    require(recovered && resumed,"recovery did not recenter and resume: door="+std::to_string(atDoor)+" p="+std::to_string(p.x)+","+std::to_string(p.y)+" blocked="+std::to_string(blocked));
    std::cout<<"overlap recovered and resumed door="<<atDoor<<" side="<<side<<" blocked="<<blocked<<'\n';
}
void weakResponseProgress() {
    CorridorController c(config());Vec2 p{},v{};seed(c,p,false);
    double start_s=0,lead_sum=0;int n=0;
    for(int i=0;i<900;++i) {
        c.filtered_velocity_=v;
        const auto cmd=c.forwardCorridor(p,0,v.x,v.y,dt);
        const Vec2 desired{.5*(cmd.target.x-p.x),.5*(cmd.target.y-p.y)};
        v.x+=(1-std::exp(-dt/.65))*(desired.x-v.x);v.y+=(1-std::exp(-dt/.65))*(desired.y-v.y);
        if(i==250)start_s=-p.y;
        if(i>=250 && i<550){lead_sum+=p.y-cmd.target.y;++n;}
        p.x+=v.x*dt;p.y+=v.y*dt;
        if(i==549)break;
    }
    const double speed=(-p.y-start_s)/(300*dt),lead=lead_sum/n;
    require(lead>.75 && speed>.37,"weak-response aircraft received insufficient forward lead");
    std::cout<<"weak response mean forward lead="<<lead<<" mean speed="<<speed<<'\n';
}
void noisyLeadLimit() {
    CorridorController c(config());seed(c,{0,0},false);
    Vec2 previous{},previous_v{};CorridorCommand cmd;
    for(int i=0;i<650;++i) {
        // Paused aircraft is alternately measured 5 mm forward/back at 20 Hz.
        Vec2 p{0,i>350 ? ((i/3)%2 ? .005:-.005):0};
        cmd=c.forwardCorridor(p,0,0,0,dt);
        const Vec2 v{(cmd.target.x-previous.x)/dt,(cmd.target.y-previous.y)/dt};
        require(norm({v.x-previous_v.x,v.y-previous_v.y})/dt<=config().position_acceleration+1e-6,
                "odometry jitter snapped the reference back at the lead limit");
        require(norm(cmd.target)<config().position_lead+.006,"braking expanded the old lead envelope");
        previous=cmd.target;previous_v=v;
    }
    require(-cmd.target.y>config().position_lead-.05,"lead-limit recovery pulled reference back to the aircraft");
    std::cout<<"5mm pose jitter at lead limit retains continuous reference\n";
}
void delayedWind(double side,double gain) {
    auto cfg=config();cfg.perception.corridor_width=1.5;
    CorridorController c(cfg);c.configure({0,0},{0,-6});c.start({0,0},0);
    Path2 cloud;
    for(int i=20;i<=260;++i){cloud.push_back({-.75,-i*.025});cloud.push_back({.75,-i*.025});}
    for(int door=0;door<2;++door)for(int i=-30;i<=30;++i) {
        double x=i*.025,center=door ? -.15:.15;
        if(std::abs(x-center)>=.40)cloud.push_back({x,door ? -4.:-2.});
    }
    Vec2 p{},v{},measured{};std::deque<Vec2> delayed(15,p);
    double clearance=10;bool done=false;int blocked=0;
    for(int tick=1;tick<=6500;++tick) {
        const double now=tick*dt;
        if(tick%5==1)c.observe(cloud,now,p);
        if(tick%3==1)measured=p;
        if(c.positionActive())c.positionFeedback(true,{0,0});
        const auto cmd=c.update(measured,0,v.x,v.y,now,true);
        require(cmd.position_control,"wind case left position interface");
        if(cmd.status.find("blocked by")!=std::string::npos)++blocked;
        delayed.push_back(cmd.target);const auto reference=delayed.front();delayed.pop_front();
        Vec2 desired{gain*(reference.x-p.x),gain*(reference.y-p.y)};
        // Explicit synthetic low-speed deadband + 0.3 s delay + 0.8 s response.
        if(std::abs(desired.x)<.025)desired.x=0;
        if(std::abs(desired.y)<.025)desired.y=0;
        const double wind=side*.025*(1+.5*std::sin(now*1.3));
        v.x+=(1-std::exp(-dt/.8))*(desired.x+wind-v.x);
        v.y+=(1-std::exp(-dt/.8))*(desired.y-v.y);
        p.x+=v.x*dt;p.y+=v.y*dt;
        for(auto point:cloud)clearance=std::min(clearance,norm({point.x-p.x,point.y-p.y}));
        require(clearance>.32,"delayed wind collided: gain="+std::to_string(gain)+" side="+std::to_string(side)+" now="+std::to_string(now));
        if(cmd.finished){done=true;break;}
    }
    require(done && c.gatesPassed()==2,"delayed wind stalled: gain="+std::to_string(gain)+" side="+std::to_string(side)+" p="+std::to_string(p.x)+","+std::to_string(p.y)+" blocked="+std::to_string(blocked));
    std::cout<<"delayed wind + deadband gain="<<gain<<" side="<<side<<" clearance="<<clearance<<" blocked="<<blocked<<'\n';
}
}
int main(){try{
    geometricRecovery();amplifiedTarget();revokeCrossing();
    for(bool door:{false,true})for(double side:{-1.,1.})recoverDynamics(door,side);
    weakResponseProgress();
    noisyLeadLimit();
    for(double gain:{.5,1.2})for(double side:{-1.,1.})delayedWind(side,gain);
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
