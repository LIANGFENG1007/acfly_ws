#include "exploration_planner/corridor_controller.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace exploration;
namespace {
void require(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
double dist(Vec2 a,Vec2 b){return std::hypot(a.x-b.x,a.y-b.y);}
Vec2 rotate(Vec2 p,double a){return {p.x*std::cos(a)-p.y*std::sin(a),p.x*std::sin(a)+p.y*std::cos(a)};}
void wall(Path2& p,Vec2 a,Vec2 b){int n=std::ceil(dist(a,b)/.025);for(int i=0;i<=n;++i)p.push_back({a.x+(b.x-a.x)*i/n,a.y+(b.y-a.y)*i/n});}
Path2 scene(bool left=false,bool gates=false){Path2 p;wall(p,{0,0},{6,0});wall(p,{-1.5,1.5},{6,1.5});if(left)wall(p,{-1.5,-2},{-1.5,1.5});
 if(gates){wall(p,{2,0},{2,.45});wall(p,{2,1.35},{2,1.5});wall(p,{3.7,0},{3.7,.15});wall(p,{3.7,1.05},{3.7,1.5});}return p;}
void geometry(){CorridorPerceptionConfig cfg;cfg.corridor_width=1.5;cfg.robot_width=.64;CorridorPerception perception(cfg);
 for(double angle:{0.,.37,1.5707963267948966,3.141592653589793})for(bool left:{false,true}){
  auto cloud=scene(left);for(auto& p:cloud)p=rotate(p,angle);auto red=rotate({-.3,-1},angle),h=rotate({5,.75},angle);
  auto e=perception.entrance(cloud,cloud,red,red,h,.6);
  if(!e.valid)std::cerr<<e.reason<<'\n';
  require(e.valid,"horizontal entrance not found without field wall");
  require(dist(e.wall_end,rotate({0,0},angle))<.04,"used field wall instead of horizontal wall end");
  require(dist(e.center,rotate({-.6,.75},angle))<.06,"centerline entrance not 60cm outside wall end");
  require(dist(e.staging,rotate({-.6,-1},angle))<.06,"incorrect outside alignment position");
  auto refined=perception.entrance(cloud,cloud,red,red,h,.6,&e);
  require(refined.valid && dist(refined.center,e.center)<1e-9,"confirmed wall refinement changed geometry");
  auto long_cfg=cfg;long_cfg.wall_min_span=1.5;CorridorPerception long_wall(long_cfg);
  require(long_wall.entrance(cloud,cloud,red,red,h,.6,&e).valid,"confirmed seed shorter than configured wall span");
  require(!perception.entrance(cloud,{},red,red,h,.6,&e).valid,"confirmed wall bypassed fresh rays");
 }
 const auto p=scene();auto no_rays=perception.entrance(p,{}, {-.3,-1},{-.3,-1},{5,.75},.6);
 require(!no_rays.valid,"unseen aperture accepted as free space");
 Path2 blocked=p;wall(blocked,{-.6,-.4},{-.6,.4});require(!perception.entrance(blocked,blocked,{-.3,-1},{-.3,-1},{5,.75},.6).valid,"blocked entrance accepted");
 // Only the near horizontal wall defines the entrance. Returns behind its end may come from an obstacle, not an opposite wall.
 Path2 single;wall(single,{0,0},{6,0});wall(single,{-1,1.5},{-.2,1.5});
 require(perception.entrance(single,single,{-.3,-1},{-.3,-1},{5,.75},.6).valid,"required opposite/field wall to define entrance");
 std::cout<<"wall-end geometry, rotations, missing field wall, blocked/unseen aperture passed\n";
}
void flight(){CorridorConfig cfg;cfg.position_control=true;cfg.wall_entry=true;cfg.perception.corridor_width=1.5;cfg.perception.robot_width=.64;
 cfg.settle_time=0;cfg.point_tolerance=.25;cfg.center_tolerance=.15;cfg.approach_distance=.8;cfg.exit_distance=.5;
 cfg.entry_speed=.4;cfg.cruise_speed=.45;cfg.gate_speed=.4;cfg.cloud_window=5;
 CorridorController c(cfg);const double nan=std::numeric_limits<double>::quiet_NaN();require(c.configure({nan,nan},{5,.75}),"auto entrance still requires old first point");
 Vec2 p{-.3,-1},v{};c.start(p,0);auto cloud=scene(false,true);bool done=false,saw_center=false,saw_alignment=false,saw_entry=false;double clearance=10;
 for(int i=1;i<6000;++i){double t=i*.02;if(i%5==1)c.observe(cloud,t,p);if(c.positionActive())c.positionFeedback(true,p);
  auto cmd=c.update(p,0,v.x,v.y,t,true);if(c.entranceReady())require(dist(c.entry(),{-.6,.75})<.06,"locked entrance moved");
  if(c.phase()==CorridorPhase::Entry && c.entranceReady()) {
   if(dist(p,c.entrancePlan().staging)<=.08)saw_alignment=true;
   if(cmd.target.y>-.98) {require(saw_alignment,"entered before actual XY aligned outside wall");saw_entry=true;}
  }
  require(cmd.forward==0 && cmd.lateral==0,"auto entry changed position interface");
  if(c.phase()!=CorridorPhase::Entry && c.phase()!=CorridorPhase::Rotate){
   if(!saw_center) {
    require(std::abs(p.y-.75)<=cfg.point_tolerance,"entry blended too far from travel line");
    require(p.y>=cfg.perception.robot_width*.5+cfg.perception.cell_size,"turned before body cleared entrance wall");
    require(CorridorPerception::pathClear(cloud,p,{-.25,.75},cfg.perception.robot_width*.5),
            "entry blended through wall-end body envelope");
   }
   saw_center=true;
  }
  Vec2 desired{1.2*(cmd.target.x-p.x),1.2*(cmd.target.y-p.y)};const double a=1-std::exp(-.02/.5);v.x+=a*(desired.x-v.x);v.y+=a*(desired.y-v.y);p.x+=v.x*.02;p.y+=v.y*.02;
  for(auto q:cloud)clearance=std::min(clearance,dist(p,q));
  if(cmd.finished){done=true;std::cout<<"wall entry + two doors complete "<<t<<"s clearance="<<clearance<<'\n';break;}
 }
 if(!done)std::cerr<<"position "<<p.x<<","<<p.y<<" reason "<<c.entrancePlan().reason<<" phase "<<(int)c.phase()<<" gates "<<c.gatesPassed()<<'\n';
 require(saw_alignment && saw_entry,"missing alignment/straight entry stages");
 require(done && c.gatesPassed()==2,"wall entry/two doors stalled");require(clearance>.32,"wall entry hit wall");
}
void confirmation(){
 CorridorConfig cfg;cfg.wall_entry=true;cfg.position_control=true;cfg.settle_time=0;cfg.cloud_window=0;cfg.perception.robot_width=.64;cfg.perception.corridor_width=1.5;
 CorridorController c(cfg);c.configure({100,100},{5,.9});Vec2 red{-.3,-1};c.start(red,0);
 auto cloud=scene();
 for(int i=0;i<2;++i){double t=.02+i*.10;c.observe(cloud,t,red);c.update(red,0,0,0,t,true);}
 require(!c.entranceReady(),"entrance locked before 3 independent frames");
 c.update(red,0,0,0,.2,true);require(!c.entranceReady(),"recounted same cloud for entrance");
 c.update(red,0,0,0,.6,true);
 for(int i=0;i<3;++i){double t=.7+i*.10;c.observe(cloud,t,red);c.update(red,0,0,0,t,true);if(i<2)require(!c.entranceReady(),"expired confirmation survived cloud gap");}
 require(c.entranceReady(),"did not lock recovered entrance");
 require(dist(c.entry(),{-.6,.75})<.04 && dist(c.axis(),{1,0})<.01,"off-center H tilted wall-based centerline");
 auto locked=c.entry();for(auto& p:cloud)p.x+=.2;c.observe(cloud,1,red);c.update(red,0,0,0,1,true);
 require(dist(c.entry(),locked)<1e-12,"new raw endpoint moved locked entrance");
 // A small matching correction uses the same 80/20 weight after confirmation.
 auto nearby=scene();for(auto& p:nearby)p.x+=.04;
 c.observe(nearby,1.1,red);c.update(red,0,0,0,1.1,true);
 require(std::abs(c.entry().x-(locked.x+.008))<.002,"confirmed entrance was not updated by weighted mean");
 const auto filtered=c.entry();
 c.observe({},2,red);c.update(red,0,0,0,2,true);
 require(c.entranceReady() && dist(c.entry(),filtered)<1e-12 && c.entrancePlan().valid,
     "missing wall scan removed entrance or changed coordinates");
 c.observe(nearby,3,red);c.update(red,0,0,0,3,true);
 require(std::abs(c.entry().x-(filtered.x*.8+(locked.x+.04)*.2))<.002,
     "reacquired entrance did not retain old weighted estimate");
 c.start(red,3.1);require(!c.entranceReady() && !c.entrancePlan().valid,"new mission retained previous entrance");
 std::cout<<"independent confirmation, timeout, ignored legacy entry, weighted entrance and persistent memory passed\n";
}
void entranceUpdateKeepsGate() {
 CorridorConfig cfg;cfg.wall_entry=true;cfg.position_control=true;cfg.settle_time=0;cfg.cloud_window=0;
 cfg.perception.robot_width=.64;cfg.perception.corridor_width=1.5;
 CorridorController c(cfg);c.configure({100,100},{5,.75});Vec2 red{-.3,-1};c.start(red,0);
 auto cloud=scene(false,true);
 for(int i=0;i<5;++i){double t=.02+i*.10;c.observe(cloud,t,red);c.update(red,0,0,0,t,true);}
 require(c.entranceReady() && c.displayGate().gate_passable && c.gateConfirmations()==3,
     "fixture did not preconfirm door while holding before entry");
 const auto before=c.displayGate().gate_center;const auto entry=c.entry();
 // Shorten the measured near wall by 4 cm; the doorway itself stays in world coordinates.
 Path2 changed;for(auto p:cloud)if(!(p.x<.04 && std::abs(p.y)<1e-8))changed.push_back(p);
 c.observe(changed,.62,red);c.update(red,0,0,0,.62,true);
 require(c.entry().x>entry.x && c.entry().x<entry.x+.04,"entrance was not refined");
 require(c.displayGate().gate_passable && c.gateConfirmations()==3,"entrance update erased confirmed door");
 const auto raw=c.observation().gate_center;
 const Vec2 expected{before.x*.8+raw.x*.2,before.y*.8+raw.y*.2};
 require(dist(c.displayGate().gate_center,expected)<1e-9,
     "moving corridor origin corrupted weighted door world coordinates");
 std::cout<<"entrance refinement preserves remembered door world coordinates and confirmation\n";
}

}
int main(){try{geometry();confirmation();entranceUpdateKeepsGate();flight();}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
