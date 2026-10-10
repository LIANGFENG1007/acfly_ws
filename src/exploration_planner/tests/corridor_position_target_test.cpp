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
    c.locked_gate_.gap_left=.392767;c.locked_gate_.gap_right=-.392767;
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
            const double lateral_accel=2*config().position_lateral_max/
                (config().position_lateral_response*config().position_lateral_response);
            require(norm({v.x-last_v.x,v.y-last_v.y})/dt<=std::hypot(config().position_acceleration,lateral_accel)+1e-6,
                "reference jumped during correction");
            if(gate)require(CorridorPerception::pathClear(c.points_,p,cmd.target,.32),
                "diagonal reference entered wall/door body envelope");
            last=cmd.target;last_v=v;
        }
        require(side*cmd.target.x<-.025,"position target failed to extend across centerline");
        // Returning velocity must retract the request before the vehicle crosses the centerline.
        c.filtered_velocity_={-side*.15,0};
        for(int i=0;i<300;++i)cmd=c.forwardCorridor(p,0,0,0,dt);
        require(std::abs(cmd.target.x)<.005,"returning lateral speed did not retract extension to center");
    }
    std::cout<<"travel line AND gate: left/right diagonal position extension, retraction and smoothness passed\n";
}
void noPrematureSideReversal() {
    CorridorController c(config());
    for(double side:{-1.,1.})for(double error:{.001,.04,.12,.30}) {
        const double stationary=c.lateralPositionExtra(side*error,0,.18);
        require(side*stationary>0,"stationary offset lost cross-center pull");
        double previous=std::abs(stationary);
        for(int i=0;i<=50;++i) {
            const double extra=c.lateralPositionExtra(side*error,-side*i*.01,.18);
            require(side*extra>=0,"inward velocity flipped lateral intent before crossing center");
            require(std::abs(extra)<=previous+1e-12,"returning faster increased extension");
            require(std::abs(extra)<=.18+1e-12,"extension exceeded limit");
            previous=std::abs(extra);
        }
        require(previous<1e-12,"sufficient inward motion did not retract extension");
        require(std::abs(c.lateralPositionExtra(side*error,side*.2,.025))<=.025,
                "door room was ignored");
    }
    require(c.lateralPositionExtra(0,.3,.18)==0,"centered aircraft requested a side reversal");
    std::cout<<"both sides: inward speed retracts pull, never reverses it before crossing\n";
}
void consistentReferenceStopping() {
    for(double yaw:{0.,.7,1.5707963267948966,3.141592653589793}) {
        CorridorController c(config());Vec2 a{std::cos(yaw),std::sin(yaw)},n{-a.y,a.x};
        c.configure({3.1,-2.7},{3.1+6*a.x,-2.7+6*a.y});
        for(double along:{-.4,0.,.4})for(double cross:{-1.03,0.,1.03}) {
            Vec2 v{a.x*along+n.x*cross,a.y*along+n.y*cross};
            const double expected=std::max(std::abs(along)/.6,std::abs(cross)/4.);
            require(std::abs(c.referenceStopTime(v,true,4.)-expected)<1e-9,
                    "reference stopping time disagrees with independent-axis stopping curve");
            require(std::abs(c.referenceStopTime(v,false,4.)-norm(v)/.6)<1e-9,
                    "entry stopping changed to corridor-only model");
        }
    }
    std::cout<<"reference stopping consistent for both axes and rotated corridors\n";
}
void noGateTargetWithdrawal() {
    for(double side:{-1.,1.}) {
        CorridorController c(config());const Vec2 p{side*.07,-1.4};seed(c,p,true);
        // The existing reference is safe through the opening. Small side error exceeds
        // the admission threshold; it must stop advancing, not retreat behind the door.
        c.position_reference_={0,-2.05};
        require(CorridorPerception::pathClear(c.points_,p,c.position_reference_,.32),"invalid test geometry");
        double prior_s=2.05;
        for(int i=0;i<150;++i) {
            const auto cmd=c.forwardCorridor(p,0,0,0,dt);
            const double s=-cmd.target.y;
            require(s>=prior_s-1e-9,"centering gate withdrew existing safe forward reference");
            require(s<=2.05+1e-8,"unadmitted gate reference kept advancing");
            require(CorridorPerception::pathClear(c.points_,p,cmd.target,.32),"retained target crossed door frame");
            prior_s=s;
        }
    }
    std::cout<<"gate admission pauses advancement without withdrawing safe reference\n";
}
void wallNormalBraking() {
    // A wall must not turn a long tangential goal into a large sideways reversal.
    for(double yaw:{0.,.7,1.5707963267948966,3.141592653589793})for(double mirror:{-1.,1.}) {
        CorridorController c(config());const Vec2 a{std::cos(yaw),std::sin(yaw)},n{-a.y,a.x};
        c.configure({3.1,-2.7},{3.1+6*a.x,-2.7+6*a.y});
        const auto world=[&](double s,double t){return c.perception_.toWorld(s,mirror*t);};
        c.phase_=CorridorPhase::Approach;c.position_active_=true;
        c.observation_.walls_observed=true;c.observation_.right_wall=-.75;c.observation_.left_wall=.75;
        for(int i=-100;i<=300;++i)for(double side:{-.75,.75})c.points_.push_back(world(i*.02,side));
        const Vec2 p=world(0,.30);
        c.filtered_velocity_={a.x*.35+n.x*mirror*.20,a.y*.35+n.y*mirror*.20};
        c.position_reference_velocity_={a.x*.45-n.x*mirror*.8,a.y*.45-n.y*mirror*.8};
        c.position_reference_stop_time_=c.referenceStopTime(c.position_reference_velocity_,true,4.);
        const auto short_goal=c.measuredBrakeTarget(p,world(.6,.40),true,4.);
        const auto far_goal=c.measuredBrakeTarget(p,world(2.,.40),true,4.);
        require(std::abs(c.perception_.longitudinal(short_goal)-.6)<1e-9 &&
                std::abs(c.perception_.longitudinal(far_goal)-2.)<1e-9,"side-wall brake changed tangential progress");
        require(std::abs(c.perception_.lateral(short_goal)-c.perception_.lateral(far_goal))<1e-9,
                "long goal amplified side-wall braking");
        // stop=.2*1.2+.2^2/(2*.6), free side room=.75-.30-.34;
        const double expected=.30-(.2*1.2+.2*.2/1.2-(.75-.30-.34));
        require(std::abs(mirror*c.perception_.lateral(short_goal)-expected)<1e-9,"wrong wall-normal stopping clearance");
        c.position_reference_velocity_={};c.position_reference_stop_time_=0;
        const auto no_reference_motion=c.measuredBrakeTarget(p,world(.6,.40),true,4.);
        require(norm({no_reference_motion.x-short_goal.x,no_reference_motion.y-short_goal.y})<1e-9,
                "forward or outward reference inflated wall-normal brake time");
        std::reverse(c.points_.begin(),c.points_.end());
        const auto reversed=c.measuredBrakeTarget(p,world(.6,.40),true,4.);
        require(norm({reversed.x-short_goal.x,reversed.y-short_goal.y})<1e-9,"wall point order changed output");
        // A missing recognition frame retains identity, but still requires real wall samples.
        c.locked_gate_=c.observation_;c.observation_={};
        const auto remembered=c.measuredBrakeTarget(p,world(.6,.40),true,4.);
        require(norm({remembered.x-short_goal.x,remembered.y-short_goal.y})<1e-9,"wall recognition dropout changed braking model");
        c.points_.clear();const auto no_points=c.measuredBrakeTarget(p,world(.6,.40),true,4.);
        require(norm({no_points.x-world(.6,.40).x,no_points.y-world(.6,.40).y})<1e-9,"remembered wall fabricated geometry without points");
    }
    std::cout<<"wall-normal braking invariant to along-wall goal, reference, orientation, mirror and point order\n";
}
void wallBrakeContinuityAndClearance() {
    CorridorController c(config());c.configure({0,0},{6,0});
    c.observation_.walls_observed=true;c.observation_.right_wall=-.75;c.observation_.left_wall=.75;
    for(int i=-50;i<=200;++i)for(double t:{-.75,.75})c.points_.push_back({i*.04,t});
    const Vec2 p{0,.30},goal{1.2,.35};double previous=goal.y;
    for(int i=0;i<=1000;++i) {
        c.filtered_velocity_={.3,i*.001};
        const auto result=c.measuredBrakeTarget(p,goal,true,4.);
        require(std::abs(result.x-goal.x)<1e-9,"side brake changed along-wall destination");
        require(result.y<=previous+1e-9 && previous-result.y<.003,
                "smoothly increasing wall approach triggered a discontinuous side reversal");
        require(result.y>=-.41-1e-9 && result.y<=.41+1e-9,"side braking entered opposite wall margin");
        previous=result.y;
    }
    // Braking a virtual reference toward the wall needs extra time; moving away does not.
    c.filtered_velocity_={.3,.2};c.position_reference_velocity_={0,0};
    const auto stationary=c.measuredBrakeTarget(p,goal,true,4.);
    c.position_reference_velocity_={.8,.8};
    const auto closing=c.measuredBrakeTarget(p,goal,true,4.);
    require(std::abs((stationary.y-closing.y)-.04)<1e-9,"toward-wall reference stopping omitted");
    // Far-away wall samples cannot be extended into a plane alongside the aircraft.
    c.points_={{20,.75},{20,-.75}};c.position_reference_velocity_={};
    const auto unsupported=c.measuredBrakeTarget(p,goal,true,4.);
    require(norm({unsupported.x-goal.x,unsupported.y-goal.y})<1e-9,"distant wall invented local restriction");
    c.observation_.right_wall=-.4;c.observation_.left_wall=.4;
    c.points_={{0,-.4},{0,.4}};c.filtered_velocity_={0,1.};
    const auto narrow=c.measuredBrakeTarget({0,0},{1.2,0},true,4.);
    require(std::abs(narrow.y+.06)<1e-9,"strong brake pushed goal beyond opposite measured wall");
    std::cout<<"wall brake continuity, opposite-wall bound, toward-wall reference and local support passed\n";
}
void gateLimitBeforeBraking() {
    CorridorController c(config());seed(c,{0,-1.3},true);
    c.position_gate_limit_s_=1.6;c.filtered_velocity_={.2,0};
    for(double reference_s:{1.45,1.75}) {
        c.position_reference_={0,-reference_s};c.position_reference_velocity_={};
        c.positionCommand({0,-1.3},{.05,-4.},.4,dt);
        require(std::abs(c.perception_.longitudinal(c.positionBrakeInput())-std::max(1.6,reference_s))<1e-9,
                "unadmitted far goal reached measured braking or existing safe goal was withdrawn");
    }
    // Unclassified obstacles and entry braking keep the measured swept-body fallback.
    c.gate_locked_=false;c.position_gate_approach_=false;c.observation_={};c.candidate_={};c.locked_gate_={};
    c.points_={{.5,0}};c.filtered_velocity_={.3,0};c.position_reference_velocity_={};c.position_reference_stop_time_=0;
    const auto stopped=c.measuredBrakeTarget({0,0},{.3,0},true,4.);
    require(stopped.x<0 && c.positionMeasuredBraking(),"unknown obstacle lost actual-motion braking");
    std::cout<<"gate range applied before braking; unclassified actual obstacles still brake\n";
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
    geometricRecovery();noPrematureSideReversal();consistentReferenceStopping();
    amplifiedTarget();noGateTargetWithdrawal();wallNormalBraking();wallBrakeContinuityAndClearance();
    gateLimitBeforeBraking();revokeCrossing();
    for(bool door:{false,true})for(double side:{-1.,1.})recoverDynamics(door,side);
    weakResponseProgress();
    noisyLeadLimit();
    for(double gain:{.5,1.2})for(double side:{-1.,1.})delayedWind(side,gain);
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
