#include "exploration_planner/corridor_controller.hpp"
#include "exploration_planner/params.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double dt = .02;
double rad(double degrees) { return degrees*pi/180.; }
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
Vec2 world(const CorridorCommand& cmd, double yaw) {
    return {std::cos(yaw)*cmd.forward-std::sin(yaw)*cmd.lateral,
            std::sin(yaw)*cmd.forward+std::cos(yaw)*cmd.lateral};
}

void selection() {
    const std::array<std::array<double,2>,23> cases{{
        {{0,0}}, {{20,0}}, {{-20,0}}, {{44.9,0}}, {{45.1,90}},
        {{70,90}}, {{90,90}}, {{134.9,90}}, {{135.1,180}}, {{160,180}},
        {{179.9,180}}, {{180,180}}, {{-180,180}}, {{-179.9,180}}, {{-160,180}},
        {{-134.9,-90}}, {{-90,-90}}, {{-70,-90}}, {{-45.1,-90}}, {{-44.9,0}},
        {{380,0}}, {{-430,-90}}, {{900,180}}
    }};
    for (const auto& row : cases) {
        CorridorController controller;
        require(controller.configure({1,0},{1,-6}), "valid route rejected");
        controller.start({0,0},0);
        const double yaw=rad(row[0]), expected=rad(row[1]);
        auto cmd=controller.update({0,0},yaw,0,0,.02,true);
        require(controller.headingSelected(), "fresh pose did not select heading");
        require(std::abs(angle(controller.targetYaw()-expected))<1e-12,
                "wrong nearest heading for "+std::to_string(row[0]));
        const double error=angle(expected-yaw);
        require(std::abs(error)<=pi/4+1e-12, "turn exceeds 45 degrees");
        require(cmd.yaw_rate*error>=0, "yaw commanded the long direction");
        // 跨过选角分界、里程计断流后恢复，都必须追原角度，不能重新选。
        controller.update({0,0},expected+rad(70),0,0,.04,true);
        controller.update({0,0},expected+rad(100),0,0,.06,false);
        controller.update({0,0},expected+rad(100),0,0,.08,true);
        require(std::abs(angle(controller.targetYaw()-expected))<1e-12,
                "pose drift or interruption reselected heading");
        controller.start({0,0},1);
        require(!controller.headingSelected(), "start retained heading selection");
        controller.update({0,0},expected+pi/2,0,0,1.02,true);
        require(std::abs(angle(controller.targetYaw()-expected-pi/2))<1e-12,
                "next start failed to reselect heading");
        controller.reset();
        require(!controller.headingSelected() && !controller.configured(), "reset retained mission heading");
    }
    // 等距时只要选择最近的正方向即可，且后续抖动不允许跳到另一个。
    for(double degrees : {-135.,-45.,45.,135.}) {
        CorridorController controller;
        controller.configure({1,0},{1,-6});controller.start({0,0},0);
        controller.update({0,0},rad(degrees),0,0,.02,true);
        const double selected=controller.targetYaw();
        require(std::abs(std::abs(angle(selected-rad(degrees)))-pi/4)<1e-12, "bad tie resolution");
        controller.update({0,0},rad(degrees+.1),0,0,.04,true);
        controller.update({0,0},rad(degrees-.1),0,0,.06,true);
        require(selected==controller.targetYaw(), "tie jitter changed selected heading");
    }
    CorridorController delayed;
    delayed.start({0,0},0);
    delayed.update({0,0},rad(20),0,0,.02,true);
    require(!delayed.headingSelected(), "selected heading before route arrived");
    delayed.configure({1,0},{1,-6});
    delayed.update({0,0},rad(70),0,0,.04,false);
    delayed.update({0,0},std::numeric_limits<double>::quiet_NaN(),0,0,.06,true);
    require(!delayed.headingSelected(), "invalid pose selected heading");
    delayed.update({0,0},rad(70),0,0,.08,true);
    require(std::abs(delayed.targetYaw()-pi/2)<1e-12, "late route did not use current valid pose");
    CorridorConfig fixed;fixed.nearest_cardinal_yaw=false;fixed.initial_yaw=rad(-90);
    CorridorController legacy(fixed);legacy.configure({1,0},{1,-6});legacy.start({0,0},0);
    legacy.update({0,0},rad(20),0,0,.02,true);
    require(std::abs(legacy.targetYaw()-fixed.initial_yaw)<1e-12, "explicit fixed heading ignored");
    CorridorConfig repeat;repeat.descend_before_entry=true;
    CorridorController restarting(repeat);restarting.configure({1,0},{1,-6});
    restarting.start({0,0},0);restarting.set_altitude_ready(false);
    restarting.update({0,0},0,0,0,.02,true);
    restarting.update({0,0},0,0,0,.5,true);
    require(restarting.rotationDone(), "repeat fixture did not finish first turn");
    restarting.start({0,0},1);restarting.set_altitude_ready(true);
    restarting.update({0,0},pi/2,0,0,1.02,true);
    require(!restarting.rotationDone() && restarting.phase()==CorridorPhase::Rotate,
            "new heading inherited old settle timer");
    std::cout << "heading selection, ties, wrapping, latching, freshness and reset passed\n";
}

void movingRotation() {
    for (double degrees : {20.,70.,160.,-70.}) {
        CorridorConfig cfg;cfg.descend_before_entry=true;cfg.settle_time=0;
        CorridorController c(cfg);c.configure({1,0},{1,-6});c.start({0,0},0);c.set_altitude_ready(false);
        const double yaw=rad(degrees);
        const auto command=c.update({.20,.12},yaw,0,0,.02,true);
        const auto velocity=world(command,yaw);
        require(velocity.x<0 && velocity.y<0 && std::abs(command.yaw_rate)>0,
                "turning lost simultaneous world-XY correction");
        c.update({.20,.12},c.targetYaw(),0,0,.04,true);
        require(c.rotationDone() && c.phase()==CorridorPhase::Rotate, "entry ignored altitude condition");
        c.set_altitude_ready(true);
        c.update({.20,.12},c.targetYaw()+rad(12),0,0,.06,true);
        require(c.phase()==CorridorPhase::Rotate, "entry used old heading readiness after drift");
        c.update({.20,.12},c.targetYaw(),0,0,.08,true);
        require(c.phase()==CorridorPhase::Entry, "XY drift incorrectly blocked completed turn/height");
    }
    std::cout << "rotation with XY correction and parallel altitude gate passed\n";
}

// 同一组真实墙点/门点，不同机头朝向下的世界系轨迹必须完全一致。
Path2 fly(double yaw, double corridor_angle, bool position_mode=false) {
    CorridorConfig cfg;
    cfg.nearest_cardinal_yaw=params::CORRIDOR_NEAREST_CARDINAL_YAW;
    cfg.position_control=position_mode;
    cfg.position_acceleration=params::CORRIDOR_POSITION_ACCEL_MPS2;
    cfg.perception.robot_width=params::CORRIDOR_ROBOT_WIDTH;
    cfg.perception.corridor_width=params::CORRIDOR_WIDTH;
    cfg.approach_distance=params::CORRIDOR_APPROACH_M;
    cfg.exit_distance=params::CORRIDOR_EXIT_M;
    cfg.min_forward_speed=params::CORRIDOR_MIN_FORWARD_SPEED;
    cfg.lateral_speed=params::CORRIDOR_LATERAL_SPEED;
    cfg.lateral_acceleration=params::CORRIDOR_LATERAL_ACCEL;
    cfg.point_tolerance=params::CORRIDOR_POINT_TOL;
    cfg.center_tolerance=params::CORRIDOR_CENTER_TOL;
    cfg.cloud_window=params::CORRIDOR_CLOUD_WINDOW_S;
    const double co=std::cos(corridor_angle),si=std::sin(corridor_angle);
    const auto rotate=[&](Vec2 v) {return Vec2{co*v.x-si*v.y,si*v.x+co*v.y};};
    const Vec2 entry=rotate({1,0}),goal=rotate({1,-7});
    CorridorPerception frame(cfg.perception);frame.setFrame(entry,goal);
    Path2 cloud;
    for(int i=20;i<=300;++i) {
        cloud.push_back(frame.toWorld(i*.025,-.75));cloud.push_back(frame.toWorld(i*.025,.75));
    }
    for(int door=0;door<2;++door) for(int i=0;i<=60;++i) {
        const double t=-.75+i*.025;
        const double center=door ? .2 : -.2;
        if(std::abs(t-center)>=.45) cloud.push_back(frame.toWorld(door ? 4.5 : 2.,t));
    }
    CorridorController controller(cfg);controller.configure(entry,goal);controller.start({0,0},0);
    Vec2 position{},velocity{};Path2 track;bool finished=false;
    double minimum_clearance=10,first_position=-1;Vec2 handover_anchor{};
    Vec2 last_reference{},last_reference_velocity{};
    bool have_reference=false;double peak_reference_acceleration=0;
    const double response=1-std::exp(-dt/.15);
    for(int tick=1;tick<=5000;++tick) {
        const double now=tick*dt;
        if(tick%5==1) controller.observe(cloud,now,position);
        const double vf=std::cos(yaw)*velocity.x+std::sin(yaw)*velocity.y;
        const double vl=-std::sin(yaw)*velocity.x+std::cos(yaw)*velocity.y;
        if(controller.positionActive()) controller.positionFeedback(now-first_position>=.4,handover_anchor);
        const auto cmd=controller.update(position,yaw,vf,vl,now,true);
        require(std::abs(angle(controller.targetYaw()-yaw))<1e-12 && std::abs(cmd.yaw_rate)<1e-12,
                "corridor tried to change the selected heading");
        Vec2 desired=world(cmd,yaw);
        if(cmd.position_control) {
            require(cmd.forward==0 && cmd.lateral==0 && cmd.yaw_rate==0,"position mode produced custom velocity command");
            if(first_position<0) {first_position=now;handover_anchor=position;}
            require(std::hypot(cmd.target.x-position.x,cmd.target.y-position.y)<=cfg.position_lead+.02,
                    "position reference ran away from the aircraft");
            if(have_reference) {
                const Vec2 reference_velocity{(cmd.target.x-last_reference.x)/dt,(cmd.target.y-last_reference.y)/dt};
                const double acceleration=std::hypot(reference_velocity.x-last_reference_velocity.x,
                                                      reference_velocity.y-last_reference_velocity.y)/dt;
                peak_reference_acceleration=std::max(peak_reference_acceleration,acceleration);
                const double side_accel=std::max(cfg.position_acceleration,
                    std::max(cfg.cruise_speed,2*cfg.position_lateral_max/cfg.position_lateral_response)/cfg.position_lateral_response);
                require(acceleration<=std::hypot(cfg.position_acceleration,side_accel)+1e-6,
                        "position reference made an abrupt turn/stop across entry or door phases: "+std::to_string(acceleration));
                last_reference_velocity=reference_velocity;
                require(CorridorPerception::pathClear(cloud,last_reference,cmd.target,cfg.perception.robot_width*.5),
                        "smoothed reference cut through a door frame");
                if(cmd.status=="Position: aligning before door")
                    // 实测动量制动可以平滑收回参考；旧的“绝不后退”要求会禁止
                    // 必要制动。仍禁止从门后瞬跳回暂驻点，并检查上面的加速度约束。
                    require(frame.longitudinal(last_reference)-frame.longitudinal(cmd.target)<=cfg.cruise_speed*dt+1e-9,
                            "door alignment jumped back to an old staging point");
            }
            last_reference=cmd.target;have_reference=true;
            // Independent toy FCU response. Production corridor emits only the position, never this velocity.
            desired={1.2*(cmd.target.x-position.x),1.2*(cmd.target.y-position.y)};
        }
        velocity.x+=response*(desired.x-velocity.x);velocity.y+=response*(desired.y-velocity.y);
        position.x+=velocity.x*dt;position.y+=velocity.y*dt;track.push_back(position);
        for(const auto& p : cloud)
            minimum_clearance=std::min(minimum_clearance,std::hypot(p.x-position.x,p.y-position.y));
        if(cmd.finished) {finished=true;break;}
    }
    require(finished && controller.gatesPassed()==2, "two doors and H not completed");
    require(minimum_clearance>cfg.perception.robot_width*.5, "body intersected a door or wall");
    std::cout << "position_mode=" << position_mode << " heading=" << yaw*180/pi << " corridor rotation=" << corridor_angle*180/pi
              << " seconds=" << track.size()*dt << " gates=" << controller.gatesPassed()
              << " clearance=" << minimum_clearance << " reference_accel=" << peak_reference_acceleration << '\n';
    return track;
}
void frameInvariance() {
    for(bool position_mode : {false,true}) for(double rotation : {0.,rad(23)}) {
        const auto reference=fly(-pi/2,rotation,position_mode);
        for(double yaw : {0.,pi/2,pi}) {
            const auto result=fly(yaw,rotation,position_mode);
            require(reference.size()==result.size(), "body heading changed world-route timing");
            for(size_t i=0;i<result.size();++i)
                require(std::hypot(reference[i].x-result[i].x,reference[i].y-result[i].y)<1e-8,
                        "body heading changed world route or inverted correction");
        }
    }
}

void positionHandover() {
    CorridorConfig cfg;cfg.position_control=true;cfg.descend_before_entry=true;cfg.settle_time=0;
    CorridorController c(cfg);c.configure({1,0},{1,-6});c.start({0,0},0);c.set_altitude_ready(false);
    auto cmd=c.update({0,0},rad(20),0,0,.02,true);
    require(!cmd.position_control && cmd.yaw_rate<0,"position interface activated before turning");
    cmd=c.update({0,0},0,0,0,.04,true);
    require(!cmd.position_control && c.rotationDone(),"position interface activated before height readiness");
    c.set_altitude_ready(true);cmd=c.update({.05,.02},0,0,0,.06,true);
    require(cmd.position_control && cmd.target.x==.05 && cmd.target.y==.02,"first requested position jumped to entry");
    cmd=c.update({.10,.02},0,0,0,.08,true);
    require(cmd.target.x==.05,"waiting for acknowledgement followed drift");
    c.positionFeedback(false,{.08,.02});cmd=c.update({.12,.02},0,0,0,.10,true);
    require(cmd.target.x==.08,"planner failed to use the main controller's fixed handover anchor");
    c.positionFeedback(true,{.08,.02});c.set_altitude_ready(false);
    cmd=c.update({.12,.02},0,0,0,.12,true);
    require(cmd.position_control && cmd.forward==0 && cmd.lateral==0,"height recovery reverted to velocity mode");
    c.positionFeedbackTimeout({.13,.02});cmd=c.update({.14,.02},0,0,0,.14,true);
    require(cmd.target.x==.13,"lost feedback did not stop forward reference advancement");
    c.positionFeedbackTimeout({.15,.02});cmd=c.update({.15,.02},0,0,0,.16,true);
    require(cmd.target.x==.13,"lost feedback repeatedly moved the holding anchor");
    c.reset();require(!c.positionActive(),"reset retained position control request");
    std::cout<<"position handover sequencing, drop-height wait, missing ACK and reset passed\n";
}

void positionReferenceLag() {
    CorridorConfig cfg;cfg.position_control=true;cfg.settle_time=0;cfg.position_acceleration=.3;
    CorridorController c(cfg);c.configure({0,0},{0,-6});c.start({-2,0},0);
    Vec2 position{-2,0},previous=position,previous_velocity{};
    Path2 cloud;
    for(int i=0;i<=240;++i) {cloud.push_back({-.75,-.5-i*.025});cloud.push_back({.75,-.5-i*.025});}
    double parked_x=0;bool resumed=false;
    for(int i=1;i<=700;++i) {
        const double now=i*dt;
        if(i%5==1) c.observe(cloud,now,position);
        if(c.positionActive()) c.positionFeedback(true,{-2,0});
        // 停住的飞机不能让参考点在领先上限处走走停停；飞机缓慢跟上后再平滑推进。
        if(i>400) position.x+=.1*dt;
        const auto cmd=c.update(position,0,0,0,now,true);
        require(cmd.position_control,"lag fixture lost position interface");
        const Vec2 v{(cmd.target.x-previous.x)/dt,(cmd.target.y-previous.y)/dt};
        require(std::hypot(v.x-previous_velocity.x,v.y-previous_velocity.y)/dt<=cfg.position_acceleration+1e-6,
                "reference snapped at maximum lead or when aircraft resumed tick="+std::to_string(i)+
                " accel="+std::to_string(std::hypot(v.x-previous_velocity.x,v.y-previous_velocity.y)/dt)+" "+cmd.status);
        require(std::hypot(cmd.target.x-position.x,cmd.target.y-position.y)<=cfg.position_lead+1e-8,
                "reference lead grew beyond configured limit");
        if(i==400) parked_x=cmd.target.x;
        if(i>500 && cmd.target.x>parked_x+.1) resumed=true;
        previous=cmd.target;previous_velocity=v;
    }
    require(parked_x>-2+cfg.position_lead-.01 && parked_x<=-2+cfg.position_lead+1e-8 && resumed,
            "reference failed to stop at lead bound or resume following");
    std::cout<<"smooth reference lead limit and slow-aircraft recovery passed\n";
}

void earlyDiagonalApproach() {
    for(double side : {-1.,1.}) for(bool blocked : {false,true}) {
        CorridorConfig cfg;cfg.position_control=true;cfg.settle_time=0;
        cfg.approach_distance=.8;cfg.center_tolerance=.15;cfg.perception.robot_width=.64;
        CorridorPerception frame(cfg.perception);frame.setFrame({0,0},{0,-6});
        Path2 cloud;
        for(int i=20;i<=260;++i) {
            cloud.push_back(frame.toWorld(i*.025,-.75));cloud.push_back(frame.toWorld(i*.025,.75));
        }
        const double center=side*.2;
        for(int i=0;i<=60;++i) {
            const double lateral=-.75+i*.025;
            if(std::abs(lateral-center)>=.45) cloud.push_back(frame.toWorld(2.,lateral));
        }
        // 中线可走，但提前斜向门中心的整段会擦到这一个实测障碍点。
        if(blocked) cloud.push_back(frame.toWorld(.75,side*.39));
        CorridorController c(cfg);c.configure({0,0},{0,-6});c.start({0,0},0);
        double lateral=0;bool seen=false;
        for(int tick=1;tick<=100;++tick) {
            const double now=tick*dt;
            if(tick%5==1)c.observe(cloud,now,{0,0});
            if(c.positionActive())c.positionFeedback(true,{0,0});
            const auto cmd=c.update({0,0},0,0,0,now,true);
            if(c.gateLocked())seen=true;
            require(cmd.position_control,"diagonal approach lost position interface");
            lateral=frame.lateral(cmd.target);
            require(CorridorPerception::pathClear(cloud,{0,0},cmd.target,.32),
                    "early diagonal reference intersected measured obstacle");
        }
        require(seen,"diagonal fixture did not confirm the door");
        if(blocked)require(std::abs(lateral)<1e-8,"early diagonal ignored a blocking measured point");
        else require(lateral*side>.025,"clear door waited for fixed approach distance instead of approaching diagonally");
    }
    std::cout<<"early left/right diagonal approach and blocked-segment rejection passed\n";
}
}

int main() {
    try {selection();movingRotation();positionHandover();positionReferenceLag();earlyDiagonalApproach();frameInvariance();}
    catch(const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
    std::cout << "corridor nearest-heading tests passed\n";
}
