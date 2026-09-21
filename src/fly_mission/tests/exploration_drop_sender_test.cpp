#include "fly_mission/exploration_drop_sender.hpp"
#include <pty.h>
#include <iostream>
#include <stdexcept>
#include <algorithm>
using namespace fly_mission;
int serial_fd=-1;
int write_mode=0;
extern "C" ssize_t __real_write(int,const void*,size_t);
extern "C" ssize_t __wrap_write(int fd,const void* data,size_t size) {
    if (fd==serial_fd) {
        if(write_mode==1){errno=EAGAIN;return -1;}
        if(write_mode==2){errno=EINTR;write_mode=0;return -1;}
        if(write_mode==3){errno=EIO;return -1;}
        size=std::min(size,size_t(7));
    }
    return __real_write(fd,data,size);
}
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::string read_all(int fd) {char buf[512];std::string s;ssize_t n;while((n=read(fd,buf,sizeof(buf)))>0)s.append(buf,n);return s;}
int main(){try{
    int master,slave;char name[128];check(openpty(&master,&slave,name,nullptr,nullptr)==0,"pty open");
    close(slave);fcntl(master,F_SETFL,O_NONBLOCK);
    ArduinoSerial serial(name,0);check(serial.send("BEEP",1,0),"open isolated serial");
    read_all(master);
    // Find the isolated slave descriptor, without touching a hardware device.
    for(int fd=3;fd<128;++fd) if(isatty(fd) && fd!=master) {
        char* path=ttyname(fd);if(path && std::string(path)==name) serial_fd=fd;
    }
    check(serial_fd>=0,"isolated slave descriptor");
    ExplorationDropSender sender;
    for(unsigned i=1;i<=3;++i){
        sender.request(i,100+i,10,serial,5);sender.request(i,100+i,10,serial,5);
        write_mode=2;serial.pump();check(serial.ok(),"EINTR closed healthy serial");
        for(int j=0;j<20;++j){serial.pump();sender.poll(10+j*.02,serial);}
        auto result=sender.result(i,100+i);check(result&&result->success,"partial writes failed batch");
        sender.request(i,100+i,11,serial,5);serial.pump();
        std::string expected,line=i==1?"DIANCI\n":i==2?"DIANCI2\n":"DIANCI3\n";
        for(int n=0;n<5;++n) expected+=line;
        check(read_all(master)==expected,"wrong command or duplicate/missing five lines");
    }
    check(sender.result(1,999) && !sender.result(1,999)->success,
          "spent ordinal with new target left restarted planner waiting forever");
    sender.request(4,104,12,serial,5);check(!serial.pending(),"fourth batch allowed");
    sender.reset(serial);sender.request(2,102,13,serial,5);check(!serial.pending(),"out-of-order batch allowed");
    sender.request(1,101,14,serial,5);write_mode=1;serial.pump();sender.poll(15,serial);
    check(sender.result(1,101)&&!sender.result(1,101)->success&&!serial.pending(),"blocked writes hung or left stale batch");
    write_mode=0;sender.request(1,101,16,serial,5);serial.pump();check(read_all(master).empty(),"failed drop replayed");
    sender.reset(serial);sender.request(1,101,17,serial,5);sender.cancel(serial);check(!serial.pending(),"abort left a release queued");
    sender.reset(serial);sender.request(1,101,17,serial,5);
    write_mode=1;sender.pump(17,serial);check(serial.pending()>0,"delayed pump fixture");
    write_mode=0;sender.pump(18.01,serial);
    check(sender.result(1,101)&&!sender.result(1,101)->success&&read_all(master).empty(),
          "delayed control tick transmitted expired release before checking timeout");
    sender.reset(serial);sender.request(1,101,18,serial,5);write_mode=3;serial.pump();sender.poll(18.1,serial);
    check(sender.result(1,101)&&!sender.result(1,101)->success&&!serial.pending(),"fatal write not reported");
    ArduinoSerial absent("/tmp/acfly-no-such-serial",0);sender.reset(absent);sender.request(1,101,20,absent,5);
    check(sender.result(1,101)&&!sender.result(1,101)->success,"closed port counted as success");
    check(absent.queue(std::string(100,'x'),5),"capacity fixture");auto size=absent.pending();
    check(!absent.queue("DIANCI",5)&&absent.pending()==size,"queue capacity failure left partial batch");
    close(master);std::cout<<"Five-line serial batches, partial writes, deduplication, abort, timeout, closed port and failures passed\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
