#include "exploration_planner/visual_drop.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace exploration;
using Phase = VisualDrop::Phase;
void check(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
VisionVisitTarget target(unsigned id = 1) { VisionVisitTarget t; t.track_id=id; t.position=t.coarse_position={2,2}; t.confirmed_stamp=9; return t; }
VisionFrame frame(unsigned seq, double stamp, Vec2 p) { VisionFrame f; f.seq=seq; f.stamp=stamp; f.count=1; f.targets[0]={1,p.x,p.y}; return f; }
void enter(VisualDrop& d, unsigned id=1) {
    d.begin(target(id)); d.tick({1,2},1.3,0,10, .1,.1);
    check(d.phase()==Phase::Sample,"1m entry did not latch sample phase");
}
void samples_and_fallback() {
    VisualDrop d; enter(d);
    d.sample(frame(2,9.99,{2.1,2}),{1,2},10.01,{});
    check(d.samples()==0,"pre-hover frame counted");
    auto f=frame(4,10.01,{2.1,2}); d.sample(f,{1,2},10.02,{}); d.sample(f,{1,2},10.03,{});
    f=frame(6,10.1,{2,2}); f.count=0; d.sample(f,{1,2},10.11,{});
    d.sample(frame(8,10.7,{1.9,2}),{1,2},10.71,{});
    check(d.samples()==2,"empty frame/gap erased sample or duplicate counted");
    d.tick({1.1,2.1},1.2,.1,10.99,.1,.1);
    check(d.phase()==Phase::Sample && d.lidar_target(.1).x==1,"hover anchor followed drift or window ended early");
    d.tick({1.1,2.1},1.2,.1,11,.1,.1);
    const double w1=.25/(1.1*1.1),w2=.25/(.9*.9);
    check(d.phase()==Phase::Descend && !d.used_coarse() &&
          std::abs(d.object().x-(2.1*w1+1.9*w2)/(w1+w2))<1e-10,"weighted mean wrong");
    d.sample(frame(10,11,{2.5,2}),{1,2},11,{});
    check(d.samples()==2,"post-window sample changed locked target");
    VisualDrop empty; enter(empty); empty.tick({1,2},1.3,0,11,.1,.1);
    check(empty.phase()==Phase::Descend && empty.used_coarse() && empty.object().x==2,"empty window stalled coarse fallback");
    VisualDrop many; enter(many);
    for(unsigned i=1;i<=25;++i) many.sample(frame(i*2,10+i*.02,{2,2}),{1,2},10+i*.02,{});
    check(many.samples()==25,"sampling imposed old ten-frame cap");
    VisualDrop rejected; enter(rejected);
    rejected.sample(frame(2,10.1,{1.6,2}),{1,2},10.1,{});
    rejected.tick({1.37,2},.6,0,11,.1,.1);
    check(rejected.xy_since()==11,"rejected refinement regression fixture");
    rejected.reject_refinement();
    rejected.tick({1.37,2},.6,0,15,.1,.1);
    check(rejected.xy_since()<0 && rejected.phase()==Phase::Descend,
          "rejected precise position kept arrival timer and released before coarse XY arrival");
    VisualDrop ambiguous; enter(ambiguous);
    ambiguous.sample(frame(2,10.1,{2.2,2}),{1,2},10.1,{{2.21,2}});
    check(ambiguous.samples()==0,"another target's observation leaked into mean");
}
void timers_and_offsets() {
    VisualDrop d;
    for(unsigned index=1;index<=3;++index) {
        enter(d,index); d.tick({1,2},1.3,0,11,.1,.1);
        auto p=d.lidar_target(0), q=d.lidar_target(M_PI/2);
        const Vec2 expected = index==1?Vec2{1.77,2}: index==2?Vec2{2,1.77}:Vec2{2,2.23};
        check(std::hypot(p.x-expected.x,p.y-expected.y)<1e-9,"front/left/right package compensation wrong");
        const Vec2 rotated = index==1?Vec2{2,1.77}:index==2?Vec2{2.23,2}:Vec2{1.77,2};
        check(std::hypot(q.x-rotated.x,q.y-rotated.y)<1e-9,"offset used latched instead of actual yaw");
        d.tick(p,1.1,0,12,.1,.1);
        check(d.xy_since()==12 && d.phase()==Phase::Descend,"XY alone did not start deadline");
        d.tick({p.x+.2,p.y},.6,0,13,.1,.1);
        d.tick(p,.6,0,14,.1,.1); d.pose_interrupted();
        d.tick({p.x+.3,p.y},1,0,15.99,.1,.1);
        check(d.phase()==Phase::Descend,"released before four second deadline");
        d.tick({p.x+.3,p.y},1,0,16,.1,.1);
        check(d.phase()==Phase::Send && d.forced() && d.release_index()==index,"drift/height restarted irreversible timeout");
        check(!d.serial_finished(index,99,{4,4}),"wrong target serial result accepted");
        check(d.serial_finished(index,index,{2.4,2.2}),"current serial result rejected");
        check(!d.serial_finished(index,index,{4,4}) && d.completed()==index,"duplicate result counted twice");
        auto anchor=d.lidar_target(2);
        check(anchor.x==2.4 && anchor.y==2.2 && d.phase()==Phase::Climb,"release failed to latch climb XY");
        d.tick({2.5,2.3},1.19,1,16.1,.1,.1);
        check(d.phase()==Phase::Climb,"climb finished too low");
        d.tick({2.5,2.3},1.21,1,16.2,.1,.1);
        check(d.phase()==Phase::Done,"climb waited for horizontal stability");
        d.finish_visit();
    }
    d.begin(target(4)); check(d.phase()==Phase::Idle && d.completed()==3,"fourth release permitted");
    d.reset(); check(d.completed()==0,"new mission retained counter");
    VisualDropConfig slow_config; slow_config.descend_speed=.05; slow_config.climb_speed=.35;
    VisualDrop configured(slow_config);enter(configured);configured.tick({1,2},1.3,0,11,.1,.1);
    check(configured.vertical_limit(1.3,.1,.1,.3)==.05,"small configured descent cap inverted clamp bounds");
    auto cp=configured.lidar_target(0);configured.tick(cp,.6,0,12,.1,.1);
    for(int i=1;i<=20;++i) configured.tick(cp,.6,0,12+i*.1,.1,.1);
    configured.serial_finished(1,1,cp);configured.tick(cp,1.3,0,15,.1,.1);
    check(configured.phase()==Phase::Done && configured.vertical_limit(1.3,.1,0,0)==.35,
          "done handoff ignored configured climb cap");
    enter(d); d.tick({1,2},1.3,0,11,.1,.1); auto p=d.lidar_target(0);
    d.tick(p,.6,0,12,.1,.1);
    for(int i=1;i<=19;++i) d.tick(p,.6,0,12+i*.1,.1,.1);
    d.tick(p,.6,0,13.99,.1,.1);
    check(d.phase()==Phase::Descend,"stable dwell shortened");
    d.tick(p,.6,0,14,.1,.1); check(d.phase()==Phase::Send && !d.forced(),"continuous two seconds failed early release");
    VisualDrop uninterrupted; enter(uninterrupted); uninterrupted.tick({1,2},1.3,0,11,.1,.1);
    p=uninterrupted.lidar_target(0);
    uninterrupted.tick(p,.6,0,12,.1,.1); uninterrupted.tick({p.x+.2,p.y},.6,0,13,.1,.1);
    uninterrupted.tick(p,.6,0,13.1,.1,.1);
    for(int i=1;i<=11;++i) uninterrupted.tick(p,.6,0,13.1+i*.1,.1,.1);
    check(uninterrupted.phase()==Phase::Descend,"stable timer did not reset after drift");
    for(int i=12;i<=20;++i) uninterrupted.tick(p,.6,0,13.1+i*.1,.1,.1);
    check(uninterrupted.phase()==Phase::Send,"stability failed to recover after drift");
    VisualDrop stalled;enter(stalled);stalled.tick({1,2},1.3,0,11,.1,.1);
    p=stalled.lidar_target(0);stalled.tick(p,.6,0,12,.1,.1);
    stalled.tick(p,.6,0,14.1,.1,.1);
    check(stalled.phase()==Phase::Descend && stalled.xy_since()==12,
          "unobserved CPU stall counted as continuous stable hover");
    stalled.tick({p.x+.2,p.y},1.3,0,16,.1,.1);
    check(stalled.phase()==Phase::Send && stalled.forced(),"CPU stall reset four-second deadline");
}
void queue_fallback_and_speed() {
    VisionTargetQueue q; VisionCandidate c; c.track_id=1; c.position={2,2}; c.confirmed=true; c.confirmed_stamp=9;
    q.enqueue({c},{}); check(q.activate_next(10),"queue fixture");
    check(q.complete_active(1,15,true) && q.visited().size()==1,"coarse drop never completed queue");
    VisualDrop d; enter(d); d.tick({1,2},1.3,0,11,.1,.1);
    check(std::abs(d.altitude(.1)-.6)<1e-9,"height not based on takeoff Z");
    check(d.vertical_limit(1.3,.1,.1,.3)==.4,"close target still forced to old 0.1m/s descent cap");
    check(d.vertical_limit(1.3,.1,1,.3)>.1 && d.vertical_limit(1.3,.1,1,.3)<.4,"descent not coordinated with horizontal travel");
}
void sampling_approach_limits() {
    VisualDropConfig cfg; cfg.entry_distance=.8; cfg.approach_slow_distance=1.5; cfg.entry_speed=.06;
    VisualDrop d(cfg); d.begin(target());
    const double cruise=.4;
    check(std::abs(d.approach_speed_limit({2-3.0,2},cruise)-cruise)<1e-12,
          "approach cap slowed cruise outside the configured braking distance");
    double previous=cruise;
    for (double distance=2.3; distance>=.7; distance-=.01) {
        const double cap=d.approach_speed_limit({2-distance,2},cruise);
        check(cap>=cfg.entry_speed-1e-12 && cap<=previous+1e-12 && cap<=cruise,
              "approach cap lost its crossing speed or increased near the sampling circle");
        previous=cap;
    }
    check(std::abs(d.approach_speed_limit({1.2,2},cruise)-cfg.entry_speed)<1e-12 &&
          d.approach_speed_limit({1.2,2},.03)<=.03,
          "sampling entry cap ignored the configured radius or exceeded cruise speed");
    d.tick({1.19,2},1.3,0,10,.1,.1);
    check(d.phase()==Phase::Approach,"braking began the sampling timer outside the circle");
    d.tick({1.21,2},1.3,0,10.02,.1,.1);
    check(d.phase()==Phase::Sample,"configured sampling radius added a velocity arrival gate");
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
int main() { try { samples_and_fallback();timers_and_offsets();queue_fallback_and_speed();sampling_approach_limits();
    std::cout<<"Visual drop sampling, compensation, timeout/stability, serial dedup, climb, three-drop limit and fallback passed\n";
} catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;} }
