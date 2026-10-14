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
void require(bool b,const char* s){if(!b)throw std::runtime_error(s);}
Vec2 rot(Vec2 p,double a){return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a)};}
double norm(Vec2 p){return std::hypot(p.x,p.y);}
CorridorConfig cfg(){CorridorConfig c;c.position_control=true;c.wall_entry=true;c.point_tolerance=.25;
 c.perception.robot_width=.64;c.perception.corridor_width=1.5;c.settle_time=0;return c;}
CorridorController entry(double angle) {
 CorridorController c(cfg());c.entry_=rot({-.6,.75},angle);c.h_=rot({5,.75},angle);
 c.perception_.setFrame(c.entry_,c.h_);c.configured_=true;c.phase_=CorridorPhase::Entry;
 c.position_active_=c.position_ready_=c.position_height_admitted_=true;
 c.entrance_locked_=c.entrance_staged_=true;c.entrance_plan_.valid=true;
 c.entrance_plan_.wall_end=rot({0,0},angle);c.entrance_plan_.inward=rot({0,1},angle);
 c.entrance_plan_.axis=rot({1,0},angle);c.entrance_plan_.center=c.entry_;
 c.position_reference_=rot({-.60,.54},angle);c.filtered_velocity_=rot({.1,.25},angle);
 c.observation_.walls_observed=true;c.observation_.observed_until=5;
 c.evidence_time_=1;c.last_update_=.98;c.heading_selected_=true;
 for(int i=0;i<=240;++i){c.points_.push_back(rot({i*.025,0},angle));c.points_.push_back(rot({i*.025-1,1.37},angle));}
 return c;
}
void blendGuards(){
 for(double angle:{0.,.37,1.5707963267948966,3.141592653589793}) {
  auto c=entry(angle);auto p=rot({-.43,.56},angle);
  require(c.canBlendEntry(p),"safe inside-wall forward drift still required return to 8cm point");
  auto cmd=c.update(p,0,c.filtered_velocity_.x,c.filtered_velocity_.y,1,true);
  require(c.entryBlended() && c.phase()!=CorridorPhase::Entry,"safe entrance did not hand off");
  require(CorridorPerception::pathClear(c.points_,p,cmd.target,.32),"handoff sent unsafe target");
  auto outside=entry(angle);require(!outside.canBlendEntry(rot({-.6,.30},angle)),"body still across wall accepted");
  auto far=entry(angle);require(!far.canBlendEntry(rot({-.6,.49},angle)),"blended too far from centerline");
  auto unstaged=entry(angle);unstaged.entrance_staged_=false;require(!unstaged.canBlendEntry(p),"bypassed outside staging");
  auto ref=entry(angle);ref.position_reference_=rot({-.6,.2},angle);require(!ref.canBlendEntry(p),"reference still outside wall accepted");
  auto outward=entry(angle);outward.filtered_velocity_=rot({0,-.3},angle);require(!outward.canBlendEntry(p),"outward momentum into wall accepted");
  auto no_wall=entry(angle);no_wall.observation_.walls_observed=false;require(!no_wall.canBlendEntry(p),"unobserved walls accepted");
  auto blocked=entry(angle);blocked.points_.push_back(rot({-.12,.65},angle));require(!blocked.canBlendEntry(p),"blocked turning body sweep accepted");
  auto behind=entry(angle);require(!behind.canBlendEntry(rot({-.72,.65},angle)),"early turn cut outside entrance route");
  auto stale=entry(angle);stale.evidence_time_=0;auto hold=stale.update(p,0,0,0,1,true);
  require(!stale.entryBlended() && hold.status=="Waiting for fresh corridor cloud","stale cloud enabled blend");
  // A recorded-entry-shaped state: one axis has reached, another is already ahead.
  auto brake=entry(angle);auto actual=rot({-.43,.72},angle);auto v=rot({.15,.28},angle);
  brake.filtered_velocity_=v;brake.positionCommand(actual,brake.entry_,.4,.02);
  const auto input=brake.positionBrakeInput();const auto inward=rot({0,1},angle),axis=rot({1,0},angle);
  require((input.x-actual.x)*inward.x+(input.y-actual.y)*inward.y>=-1e-9,"ordinary entry brake moved behind aircraft");
  require(std::abs((input.x-brake.entry_.x)*axis.x+(input.y-brake.entry_.y)*axis.y)<1e-9,
          "entry cross-axis velocity generated backwards drift target");
 }
 std::cout<<"entry blend: rotated body/ref/predicted-clearance guards, staging, fresh evidence and bounded approach braking passed\n";
}
void boundedGuidance(){
 for(double angle:{0.,.37,1.5707963267948966})for(double side:{-1.,1.}) {
  auto config=cfg();config.wall_entry=false;CorridorController c(config);c.configure({0,0},rot({6,0},angle));
  c.phase_=CorridorPhase::Search;c.position_active_=c.position_ready_=true;
  auto p=rot({0,.34*side},angle);c.position_reference_=p;
  for(int i=-40;i<=120;++i)for(double wall:{-.75,.75})c.points_.push_back(rot({i*.025,wall},angle));
  const Vec2 target=rot({1.2,-.18*side},angle),n=rot({0,1},angle);
  for(int i=0;i<150;++i) {
   c.positionCommand(p,target,.45,.02);const auto input=c.positionBrakeInput();
   require(std::abs((input.x-p.x)*n.x+(input.y-p.y)*n.y)<=.250001,"ordinary sideways target exceeded 25cm");
   require(c.positionLateralLimited(),"lateral limiter not visible in diagnostics");
  }
  auto emergency=c;emergency.points_.clear();
  for(int i=-40;i<=40;++i)emergency.points_.push_back(rot({i*.025,.65*side},angle));
  const Vec2 safe=rot({0,0},angle);emergency.position_reference_=safe;
  auto cmd=emergency.positionCommand(p,safe,.45,.02);
  require(emergency.positionRecovering(),"overlap recovery not activated");
  require(norm({cmd.target.x-p.x,cmd.target.y-p.y})>.30,"soft guidance limit weakened existing emergency escape");
  require(CorridorPerception::pathRecovering(emergency.points_,p,cmd.target,.32),"escape entered another obstacle");
 }
 auto invalid=cfg();invalid.position_lateral_lead=0;bool rejected=false;
 try{CorridorController c(invalid);}catch(const std::invalid_argument&){rejected=true;}
 require(rejected,"nonpositive lateral lead accepted");
 std::cout<<"large initial offset: bounded ordinary pull, rotated/mirrored geometry, retained emergency escape passed\n";
}
}
int main(){try{blendGuards();boundedGuidance();}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
