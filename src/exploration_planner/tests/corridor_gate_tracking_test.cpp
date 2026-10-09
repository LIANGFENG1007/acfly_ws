#include "exploration_planner/corridor_controller.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace exploration;
namespace {
void require(bool v,const char* text){if(!v)throw std::runtime_error(text);}
CorridorConfig config(){CorridorConfig c;c.position_control=true;c.settle_time=0;c.perception.corridor_width=1.5;
    c.perception.robot_width=.64;c.cloud_window=0;return c;}
Path2 scene(bool rays=true,bool closed=false,double offset=0) {
    Path2 p;
    for(int i=20;i<=(rays?240:81);++i){p.push_back({-.75,-i*.025});p.push_back({.75,-i*.025});}
    for(int i=-30;i<=30;++i)if(closed || std::abs(i*.025)>.40)p.push_back({i*.025+offset,-2});
    return p;
}
void observe(CorridorController& c,const Path2& cloud,double now,double yaw=.7) {
    c.observe(cloud,now,{0,0});c.update({0,0},yaw,0,0,now,true);
}
void intermittentAndWarmup() {
    CorridorController c(config());c.configure({0,0},{0,-6});c.start({0,0},0);
    observe(c,scene(),.02);require(c.gateConfirmations()==1 && c.displayGate().gate_passable,"first measured candidate absent");
    observe(c,scene(false),.07);
    require(!c.observation().gate_passable,"weak ray fixture unexpectedly confirmed door");
    require(c.displayGate().gate_passable && c.gateConfirmations()==1,"one weak scan erased marker/confirmation");
    observe(c,scene(),.12);observe(c,scene(false),.17);observe(c,scene(),.22);
    require(c.gateConfirmations()==3,"rotation did not accumulate independent matching door frames");
    require(!c.gateLocked(),"preconfirmation activated a door before entry");
    c.update({0,0},0,0,0,.24,true);c.positionFeedback(true,{0,0});
    c.update({0,0},0,0,0,.26,true);
    require(c.gateLocked(),"entry restarted door confirmation after warmup");
    const auto center=c.gate().gate_center;
    observe(c,scene(true,false,.025),.30,0);
    require(std::hypot(c.displayGate().gate_center.x-center.x,c.displayGate().gate_center.y-center.y)<1e-12,
        "locked marker followed one noisy raw scan");
    c.update({0,-3.3},0,0,0,.32,true);
    require(c.gatesPassed()==1 && !c.displayGate().gate_passable,"passed doorway reappeared as an old candidate marker");
    std::cout<<"3 matching frames across missed rays confirmed during rotation; locked marker stable\n";
}
void staleAndContradiction() {
    for(bool closed:{false,true}) {
        CorridorController c(config());c.configure({0,0},{0,-6});c.start({0,0},0);
        observe(c,scene(),.02);observe(c,scene(),.12);
        if(closed)observe(c,scene(true,true),.17);
        else c.update({0,0},.7,0,0,.50,true);
        require(!c.displayGate().gate_passable && c.gateConfirmations()==0,"stale/closed door retained passable candidate");
        observe(c,scene(),.52);
        require(c.gateConfirmations()==1,"stale evidence counted toward new confirmation");
        c.update({0,0},.7,0,0,.53,true);
        require(c.gateConfirmations()==1,"same scan counted twice");
    }
    std::cout<<"candidate timeout, closed-door contradiction, independent scan counting passed\n";
}
void filteredGeometry() {
    CorridorController c(config());c.configure({0,0},{0,-6});c.start({0,0},0);
    observe(c,scene(),.02);double before=c.displayGate().gate_center.x;
    observe(c,scene(true,false,.025),.07);
    const auto raw=c.observation(),tracked=c.displayGate();
    require(raw.gate_passable && tracked.gate_passable,"no noisy candidate for filtering test");
    require(std::abs(tracked.gate_center.x-before)<std::abs(raw.gate_center.x-before),"candidate center not smoothed");
    require(tracked.gap_right>=raw.gap_right-1e-9 && tracked.gap_left<=raw.gap_left+1e-9,
        "filtered opening expanded beyond measured free gap");
    require(tracked.gap_width>=config().perception.robot_width-1e-9,"filtered opening smaller than body");
    std::cout<<"candidate center smoothing stays inside current measured gap\n";
}
void unrelatedOutlier() {
    CorridorController c(config());c.configure({0,0},{0,-6});c.start({0,0},0);
    auto other=scene();for(auto& p:other)if(p.y==-2 && std::abs(p.x)<.74)p.y=-2.5;
    observe(c,scene(),.02);const auto first=c.displayGate().gate_center;
    observe(c,other,.07);
    require(c.observation().gate_passable && std::abs(c.observation().gate_s-2.5)<.01,"outlier fixture did not produce a different valid doorway");
    require(c.displayGate().gate_center.y==first.y && c.gateConfirmations()==1,"one unrelated frame jumped the marker or confirmed old door");
    observe(c,scene(),.12);require(c.gateConfirmations()==2,"matching door did not resume confirmation");
    observe(c,other,.50);require(std::abs(c.displayGate().gate_s-2.5)<.01 && c.gateConfirmations()==1,"expired candidate prevented reacquisition");
    std::cout<<"unrelated single-frame candidate cannot replace tracked marker\n";
}
}
int main(){try{intermittentAndWarmup();staleAndContradiction();filteredGeometry();unrelatedOutlier();}
catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
