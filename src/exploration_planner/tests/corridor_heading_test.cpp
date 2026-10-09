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
Path2 fly(double yaw, double corridor_angle) {
    CorridorConfig cfg;
    cfg.nearest_cardinal_yaw=params::CORRIDOR_NEAREST_CARDINAL_YAW;
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
    double minimum_clearance=10;
    const double response=1-std::exp(-dt/.15);
    for(int tick=1;tick<=5000;++tick) {
        const double now=tick*dt;
        if(tick%5==1) controller.observe(cloud,now,position);
        const double vf=std::cos(yaw)*velocity.x+std::sin(yaw)*velocity.y;
        const double vl=-std::sin(yaw)*velocity.x+std::cos(yaw)*velocity.y;
        const auto cmd=controller.update(position,yaw,vf,vl,now,true);
        require(std::abs(angle(controller.targetYaw()-yaw))<1e-12 && std::abs(cmd.yaw_rate)<1e-12,
                "corridor tried to change the selected heading");
        const auto desired=world(cmd,yaw);
        velocity.x+=response*(desired.x-velocity.x);velocity.y+=response*(desired.y-velocity.y);
        position.x+=velocity.x*dt;position.y+=velocity.y*dt;track.push_back(position);
        for(const auto& p : cloud)
            minimum_clearance=std::min(minimum_clearance,std::hypot(p.x-position.x,p.y-position.y));
        if(cmd.finished) {finished=true;break;}
    }
    require(finished && controller.gatesPassed()==2, "two doors and H not completed");
    require(minimum_clearance>cfg.perception.robot_width*.5, "body intersected a door or wall");
    std::cout << "heading=" << yaw*180/pi << " corridor rotation=" << corridor_angle*180/pi
              << " seconds=" << track.size()*dt << " gates=" << controller.gatesPassed()
              << " clearance=" << minimum_clearance << '\n';
    return track;
}
void frameInvariance() {
    for(double rotation : {0.,rad(23)}) {
        const auto reference=fly(-pi/2,rotation);
        for(double yaw : {0.,pi/2,pi}) {
            const auto result=fly(yaw,rotation);
            require(reference.size()==result.size(), "body heading changed world-route timing");
            for(size_t i=0;i<result.size();++i)
                require(std::hypot(reference[i].x-result[i].x,reference[i].y-result[i].y)<1e-8,
                        "body heading changed world route or inverted correction");
        }
    }
}
}

int main() {
    try {selection();movingRotation();frameInvariance();}
    catch(const std::exception& e) {std::cerr << "FAIL: " << e.what() << '\n';return 1;}
    std::cout << "corridor nearest-heading tests passed\n";
}
