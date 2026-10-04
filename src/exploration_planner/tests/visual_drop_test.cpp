#include "exploration_planner/visual_drop.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace exploration;
using Phase = VisualDrop::Phase;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
VisionVisitTarget target(unsigned id = 1) { VisionVisitTarget t; t.track_id=id; t.position=t.coarse_position={2,2}; t.confirmed_stamp=9; return t; }
VisionFrame frame(unsigned seq, double stamp, Vec2 p) { VisionFrame f; f.seq=seq; f.stamp=stamp; f.count=1; f.targets[0]={1,p.x,p.y}; return f; }
// Existing payload/timer fixtures explicitly approve their obstacle-free descent.
void advance(VisualDrop& d, Vec2 p, double z, double yaw, double now, double home, double tol) {
    d.tick(p,z,yaw,now,home,tol);
    if (d.phase()==Phase::Validate) d.authorize_descent();
}
void enter(VisualDrop& d, unsigned id=1) {
    d.begin(target(id)); d.start_sample({1,2},1.3,0,10);
    check(d.phase()==Phase::Sample,"approved safe entry did not latch sample phase");
}
void samples_and_fallback() {
    VisualDrop d; enter(d);
    d.sample(frame(2,9.99,{2.1,2}),{1,2},10.01,{});
    check(d.samples()==0,"pre-hover frame counted");
    auto f=frame(4,10.01,{2.1,2}); d.sample(f,{1,2},10.02,{}); d.sample(f,{1,2},10.03,{});
    f=frame(6,10.1,{2,2}); f.count=0; d.sample(f,{1,2},10.11,{});
    d.sample(frame(8,10.7,{1.9,2}),{1,2},10.71,{});
    check(d.samples()==2,"empty frame/gap erased sample or duplicate counted");
    advance(d,{1.1,2.1},1.2,.1,10.99,.1,.1);
    check(d.phase()==Phase::Sample && d.lidar_target(.1).x==1,"hover anchor followed drift or window ended early");
    advance(d,{1.1,2.1},1.2,.1,11,.1,.1);
    const double w1=.25/(1.1*1.1),w2=.25/(.9*.9);
    check(d.phase()==Phase::Descend && !d.used_coarse() &&
          std::abs(d.object().x-(2.1*w1+1.9*w2)/(w1+w2))<1e-10,"weighted mean wrong");
    d.sample(frame(10,11,{2.5,2}),{1,2},11,{});
    check(d.samples()==2,"post-window sample changed locked target");
    VisualDrop empty; enter(empty); advance(empty,{1,2},1.3,0,11,.1,.1);
    check(empty.phase()==Phase::Descend && empty.used_coarse() && empty.object().x==2,"empty window stalled coarse fallback");
    VisualDrop many; enter(many);
    for(unsigned i=1;i<=25;++i) many.sample(frame(i*2,10+i*.02,{2,2}),{1,2},10+i*.02,{});
    check(many.samples()==25,"sampling imposed old ten-frame cap");
    VisualDrop rejected; enter(rejected);
    rejected.sample(frame(2,10.1,{1.6,2}),{1,2},10.1,{});
    advance(rejected,{1.37,2},.6,0,11,.1,.1);
    check(rejected.xy_since()<0,"unapproved refinement started the release deadline");
    rejected.reject_refinement();
    advance(rejected,{1.37,2},.6,0,15,.1,.1);
    check(rejected.xy_since()<0 && rejected.phase()==Phase::Descend,
          "rejected precise position kept arrival timer and released before coarse XY arrival");
    VisualDrop ambiguous; enter(ambiguous);
    ambiguous.sample(frame(2,10.1,{2.2,2}),{1,2},10.1,{{2.21,2}});
    check(ambiguous.samples()==0,"another target's observation leaked into mean");
}
void timers_and_offsets() {
    VisualDrop d;
    for(unsigned index=1;index<=3;++index) {
        enter(d,index); advance(d,{1,2},1.3,0,11,.1,.1);
        auto p=d.lidar_target(0), q=d.lidar_target(M_PI/2);
        const Vec2 expected = index==1?Vec2{1.77,2}: index==2?Vec2{2,1.77}:Vec2{2,2.23};
        check(std::hypot(p.x-expected.x,p.y-expected.y)<1e-9,"front/left/right package compensation wrong");
        const Vec2 rotated = index==1?Vec2{2,1.77}:index==2?Vec2{2.23,2}:Vec2{1.77,2};
        check(std::hypot(q.x-rotated.x,q.y-rotated.y)<1e-9,"offset used latched instead of actual yaw");
        advance(d,p,1.1,0,12,.1,.1);
        check(d.xy_since()==12 && d.phase()==Phase::Descend,"XY alone did not start deadline");
        advance(d,{p.x+.2,p.y},.6,0,13,.1,.1);
        advance(d,p,.6,0,14,.1,.1); d.pose_interrupted();
        advance(d,{p.x+.3,p.y},1,0,15.99,.1,.1);
        check(d.phase()==Phase::Descend,"released before four second deadline");
        advance(d,{p.x+.3,p.y},1,0,16,.1,.1);
        check(d.phase()==Phase::Send && d.forced() && d.release_index()==index,"drift/height restarted irreversible timeout");
        check(!d.serial_finished(index,99,{4,4},16),"wrong target serial result accepted");
        check(d.serial_finished(index,index,{2.4,2.2},16),"current serial result rejected");
        check(!d.serial_finished(index,index,{4,4},16.01) && d.completed()==index,"duplicate result counted twice");
        auto anchor=d.lidar_target(2);
        check(anchor.x==2.4 && anchor.y==2.2 && d.phase()==Phase::Wait && d.release_index()==0,
              "release failed to latch wait XY or kept requesting the package");
        check(std::abs(d.altitude(.1)-.6)<1e-9 && d.vertical_limit(.6,.1,0,0)==.1,
              "release wait abandoned the drop altitude");
        advance(d,{2.5,2.3},1.21,1,17.99,.1,.1); d.pose_interrupted();
        check(d.phase()==Phase::Wait && d.lidar_target(2).x==2.4,
              "climb began before the configured release wait or the anchor moved");
        advance(d,{2.5,2.3},1.19,1,18,.1,.1);
        check(d.phase()==Phase::Climb && std::abs(d.altitude(.1)-1.3)<1e-9 && d.vertical_limit(1.19,.1,0,0)==.6,
              "release wait did not hand over to climb");
        advance(d,{2.5,2.3},1.19,1,18.1,.1,.1);
        check(d.phase()==Phase::Climb,"climb finished too low");
        advance(d,{2.5,2.3},1.21,1,18.2,.1,.1);
        check(d.phase()==Phase::Done,"climb waited for horizontal stability");
        d.finish_visit();
    }
    d.begin(target(4)); check(d.phase()==Phase::Idle && d.completed()==3,"fourth release permitted");
    d.reset(); check(d.completed()==0,"new mission retained counter");
    VisualDropConfig slow_config; slow_config.descend_speed=.05; slow_config.climb_speed=.35; slow_config.release_wait=0;
    VisualDrop configured(slow_config);enter(configured);advance(configured,{1,2},1.3,0,11,.1,.1);
    check(configured.vertical_limit(1.3,.1,.1,.3)==.05,"small configured descent cap inverted clamp bounds");
    auto cp=configured.lidar_target(0);advance(configured,cp,.6,0,12,.1,.1);
    for(int i=1;i<=20;++i) advance(configured,cp,.6,0,12+i*.1,.1,.1);
    check(configured.serial_finished(1,1,cp,14) && configured.phase()==Phase::Climb,
          "zero release wait did not climb immediately");
    advance(configured,cp,1.3,0,15,.1,.1);
    check(configured.phase()==Phase::Done && configured.vertical_limit(1.3,.1,0,0)==.35,
          "done handoff ignored configured climb cap");
    for (double invalid : {-.1,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        VisualDropConfig bad; bad.release_wait=invalid; bool rejected=false;
        try { VisualDrop unused(bad); } catch (const std::invalid_argument&) { rejected=true; }
        check(rejected,"invalid release wait accepted");
    }
    enter(d); advance(d,{1,2},1.3,0,11,.1,.1); auto p=d.lidar_target(0);
    advance(d,p,.6,0,12,.1,.1);
    for(int i=1;i<=19;++i) advance(d,p,.6,0,12+i*.1,.1,.1);
    advance(d,p,.6,0,13.99,.1,.1);
    check(d.phase()==Phase::Descend,"stable dwell shortened");
    advance(d,p,.6,0,14,.1,.1); check(d.phase()==Phase::Send && !d.forced(),"continuous two seconds failed early release");
    VisualDrop uninterrupted; enter(uninterrupted); advance(uninterrupted,{1,2},1.3,0,11,.1,.1);
    p=uninterrupted.lidar_target(0);
    advance(uninterrupted,p,.6,0,12,.1,.1); advance(uninterrupted,{p.x+.2,p.y},.6,0,13,.1,.1);
    advance(uninterrupted,p,.6,0,13.1,.1,.1);
    for(int i=1;i<=11;++i) advance(uninterrupted,p,.6,0,13.1+i*.1,.1,.1);
    check(uninterrupted.phase()==Phase::Descend,"stable timer did not reset after drift");
    for(int i=12;i<=20;++i) advance(uninterrupted,p,.6,0,13.1+i*.1,.1,.1);
    check(uninterrupted.phase()==Phase::Send,"stability failed to recover after drift");
    VisualDrop stalled;enter(stalled);advance(stalled,{1,2},1.3,0,11,.1,.1);
    p=stalled.lidar_target(0);advance(stalled,p,.6,0,12,.1,.1);
    advance(stalled,p,.6,0,14.1,.1,.1);
    check(stalled.phase()==Phase::Descend && stalled.xy_since()==12,
          "unobserved CPU stall counted as continuous stable hover");
    advance(stalled,{p.x+.2,p.y},1.3,0,16,.1,.1);
    check(stalled.phase()==Phase::Send && stalled.forced(),"CPU stall reset four-second deadline");
}
void queue_fallback_and_speed() {
    VisionTargetQueue q; VisionCandidate c; c.track_id=1; c.position={2,2}; c.confirmed=true; c.confirmed_stamp=9;
    q.enqueue({c},{}); check(q.activate_next(10),"queue fixture");
    check(q.complete_active(1,15,true) && q.visited().size()==1,"coarse drop never completed queue");
    VisualDrop d; enter(d); advance(d,{1,2},1.3,0,11,.1,.1);
    check(std::abs(d.altitude(.1)-.6)<1e-9,"height not based on takeoff Z");
    check(d.vertical_limit(1.3,.1,.1,.3)==.4,"close target still forced to old 0.1m/s descent cap");
    check(d.vertical_limit(1.3,.1,1,.3)>.1 && d.vertical_limit(1.3,.1,1,.3)<.4,"descent not coordinated with horizontal travel");
    check(d.vertical_limit(1.3,.1,7,.4)<.1,"long straight descent reached low height prematurely due to old minimum speed");
}
void sampling_approach_limits() {
    VisualDropConfig cfg; cfg.approach_slow_distance=1.5; cfg.entry_speed=.06;
    VisualDrop d(cfg); d.begin(target());
    const double cruise=.4;
    check(std::abs(d.approach_limit_at(3.0,cruise)-cruise)<1e-12,"premature approach braking");
    double previous=cruise;
    for(double distance=2.3;distance>=0;distance-=.01) {
        const double cap=d.approach_limit_at(distance,cruise);
        check(cap>=cfg.entry_speed-1e-12 && cap<=previous+1e-12 && cap<=cruise,"nonmonotonic safe-entry braking");
        previous=cap;
    }
    check(d.approach_limit_at(0,.03)<=.03,"entry cap exceeded cruise");
    d.tick({1.99,2},1.3,0,10,.1,.1);
    check(d.phase()==Phase::Approach,"entering old 1m circle bypassed safe entry approval");
    check(d.start_sample({1,2},1.3,0,10.02),"explicit safe entry rejected");
    d.tick({1,2},1.3,0,11.02,.1,.1);
    check(d.phase()==Phase::Validate && d.altitude(.1)==1.3 && d.release_index()==0,
          "sampling started descent/release before geometry approval");
    d.tick({1.77,2},.6,0,20,.1,.1);
    check(d.phase()==Phase::Validate && d.xy_since()<0,"unapproved descent started a timeout");
    d.retry_approach();
    check(d.phase()==Phase::Approach && d.samples()==0,"unsafe refinement could not reselect an entry");
    for (double invalid : {0.0,-.1,std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
        for (bool distance : {false,true}) {
            auto bad=cfg;
            if (distance) bad.approach_slow_distance=invalid; else bad.entry_speed=invalid;
            bool rejected=false;
            try { VisualDrop unused(bad); } catch (const std::invalid_argument&) { rejected=true; }
            check(rejected,"invalid braking setting permitted a stalled or nonfinite approach");
        }
    }
}
void early_climb_handoff_keeps_return_height() {
    VisualDropConfig cfg;cfg.climb_release=.5;cfg.release_wait=0;
    VisualDrop d(cfg);d.begin(target());
    check(d.start_sample({1,2},.72,0,10,1.3),"resumed visit rejected its safe entry");
    check(d.altitude(.1)==.72 && d.return_z()==1.3,"sample altitude overwrote original return height");
    advance(d,{1,2},.72,0,11,.1,.1);
    const auto p=d.lidar_target(0);advance(d,p,.6,0,12,.1,.1);advance(d,p,.6,0,16,.1,.1);
    check(d.serial_finished(1,1,p,16),"release fixture failed");
    advance(d,p,.72,0,16.02,.1,.1);
    check(d.phase()==Phase::Climb && d.altitude(.1)==1.3,"early handoff skipped climb or ratcheted altitude down");
}
// 投掷高度补偿：只改下降/释放阶段的目标高度与到位判定，按目标位置算一次；回升仍回原采样高度。
void drop_height_compensation() {
    VisualDropConfig cfg; cfg.z_slope_x=-.0338; cfg.z_slope_y=.0016; cfg.z_comp_max=.4;
    VisualDrop d(cfg); d.begin(target()); d.start_sample({1,2},1.3,0,10);
    check(d.phase()==Phase::Sample && std::abs(d.altitude(.1)-1.3)<1e-9,"sampling height was compensated");
    advance(d,{1,2},1.3,0,11,.1,.1);
    const double comp=-.0338*2+.0016*2;
    check(d.phase()==Phase::Descend && std::abs(d.z_compensation()-comp)<1e-12 &&
          std::abs(d.altitude(.1)-(.1+.5+comp))<1e-12,"drop height not compensated at the target position");
    const Vec2 p=d.lidar_target(0);
    for(int i=0;i<=21;++i) advance(d,p,.1+.5+comp,0,11+i*.1,.1,.1);
    check(d.phase()==Phase::Send,"compensated drop height not accepted as on target");
    VisualDropConfig far=cfg; VisualDrop e(far); VisionVisitTarget t=target(); t.position=t.coarse_position={30,0};
    e.begin(t); e.start_sample({29,0},1.3,0,10); advance(e,{29,0},1.3,0,11,.1,.1);
    check(std::abs(e.z_compensation()+.4)<1e-12,"compensation not limited");
}
int main() { try { samples_and_fallback();timers_and_offsets();queue_fallback_and_speed();sampling_approach_limits();drop_height_compensation();early_climb_handoff_keeps_return_height();
    std::cout<<"Visual drop sampling, compensation, timeout/stability, serial dedup, release wait, climb, three-drop limit and fallback passed\n";
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;} }
