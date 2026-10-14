#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"
#define private public
#include "exploration_planner/corridor_controller.hpp"
#undef private
using namespace exploration;
namespace {
void require(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
Vec2 rot(Vec2 p,double a){return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a)};}
CorridorConfig config(){CorridorConfig c;c.position_control=true;c.wall_entry=true;c.perception.robot_width=.64;
 c.perception.corridor_width=1.5;c.point_tolerance=.25;c.center_tolerance=.15;return c;}
void directEntry(){
 for(double a:{0.,.47,1.5707963267948966,3.141592653589793}){
  CorridorController c(config());c.entry_=rot({-.6,.75},a);c.h_=rot({6,.75},a);c.perception_.setFrame(c.entry_,c.h_);
  c.configured_=true;c.position_active_=c.position_ready_=true;c.entrance_locked_=true;c.entrance_plan_.valid=true;
  c.entrance_plan_.axis=rot({1,0},a);c.entrance_plan_.inward=rot({0,1},a);c.entrance_plan_.wall_end={0,0};
  const Vec2 p=rot({-.81,-1},a);c.position_reference_=p;
  for(int i=0;i<200;++i){c.points_.push_back(rot({i*.025,0},a));c.points_.push_back(rot({i*.025-1,1.37},a));}
  require(c.canEnterDirect(p),"safe outside entrance still hunts precise staging point");
  auto bad=c;bad.points_.push_back(rot({-.65,-.15},a));require(!bad.canEnterDirect(p),"blocked entrance accepted");
  bad=c;bad.filtered_velocity_=rot({.6,0},a);require(!bad.canEnterDirect(p),"incoming momentum crosses wall end");
  bad=c;bad.position_reference_=rot({-.2,-1},a);require(!bad.canEnterDirect(p),"unsafe reference accepted");
  require(!c.canEnterDirect(rot({-.30,-1},a)),"body overlaps end yet enters directly");
  require(!c.canEnterDirect(rot({-1.1,-1},a)),"direct entry bypasses point tolerance bound");
  bad=c;bad.entrance_locked_=false;require(!bad.canEnterDirect(p),"unconfirmed entrance accepted");
 }
 std::cout<<"direct entry: rotated body/reference/prediction/obstruction/confirmation guards passed\n";
}
void movingAlignment(){
 for(double a:{0.,.47,1.5707963267948966})for(double side:{-1.,1.}){
  auto cfg=config();cfg.wall_entry=false;CorridorController c(cfg);c.configure({0,0},rot({6,0},a));
  c.phase_=CorridorPhase::Approach;c.position_active_=c.position_ready_=c.position_height_admitted_=true;
  c.gate_locked_=c.position_gate_approach_=true;
  c.locked_gate_.gate_s=2;c.locked_gate_.gate_depth=.04;c.locked_gate_.gap_width=.80;
  c.locked_gate_.gap_right=-.4;c.locked_gate_.gap_left=.4;c.locked_gate_.gate_passable=true;
  for(int i=-50;i<=300;++i)for(double t:{-.75,.75})c.points_.push_back(rot({i*.02,t},a));
  for(int i=0;i<=35;++i)for(double side:{-1.,1.})c.points_.push_back(rot({2,side*(.4+i*.01)},a));
  Vec2 p=rot({1.2,.07*side},a);c.position_reference_=p;Vec2 target;
  require(c.continuousGateTarget(p,2.55,0,target),"safe diagonal passage rejected on 7cm initial error");
  require(c.perception_.longitudinal(target)>2.5,"safe passage stopped before door");
  require(CorridorPerception::pathClear(c.points_,p,target,.36),"geometry approval lost one-cell body margin");
  auto cmd=c.forwardCorridor(p,a,0,0,.02);
  require(c.gateGeometryAdmitted()&&c.positionGateAdmission()>0,"safe diagonal centering still locks progress");
  require(CorridorPerception::pathClear(c.points_,p,cmd.target,.32),"unsafe issued target");
  auto momentum=c;momentum.filtered_velocity_=rot({0,.5*side},a);
  require(!momentum.continuousGateTarget(p,2.55,0,target),"unsafe lateral stopping projection accepted");
  auto blocked=c;for(int i=-75;i<=75;++i)blocked.points_.push_back(rot({2,i*.01},a));
  const bool allowed=blocked.continuousGateTarget(p,2.55,0,target);
  require(!allowed || blocked.perception_.longitudinal(target)<1.65,"closed door permits forward crossing");
  blocked.gate_blocked_=true;auto held=blocked.forwardCorridor(p,a,0,0,.02);
  require(held.status.find("blocked")!=std::string::npos && !blocked.gateGeometryAdmitted(),"measured closed door bypassed");
  auto stale=c;stale.evidence_time_=0;stale.last_update_=.98;stale.heading_selected_=true;
  require(stale.update(p,a,0,0,1,true).status=="Waiting for fresh corridor cloud","stale evidence admits moving alignment");
 }
 std::cout<<"moving alignment: rotated/mirrored, delayed drift, blocked and stale evidence guards passed\n";
}
void measuredCenter(){
 auto cfg=config();cfg.wall_entry=false;CorridorController c(cfg);c.configure({0,0},{6,0});
 c.position_active_=true;c.phase_=CorridorPhase::Search;c.position_reference_={0,0};
 c.observation_.walls_observed=true;c.observation_.left_wall=.60;c.observation_.right_wall=-.75;
 for(int i=-20;i<=100;++i)for(double t:{-.75,.60})c.points_.push_back({i*.04,t});
 c.forwardCorridor({0,0},0,0,0,.02);
 require(!c.planned_path_.empty() && std::abs(c.planned_path_.back().y+.075)<1e-9,"travel line ignores measured wall midpoint");
 std::cout<<"measured wall midpoint replaces biased configured travel line\n";
}
}
int main(){try{directEntry();movingAlignment();measuredCenter();}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
