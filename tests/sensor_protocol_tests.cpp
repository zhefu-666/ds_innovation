#include "rescue/uart_controller.hpp"
#include "rescue/utils.hpp"
#include "../tools/mcu_sensor_packet.h"
#include <cassert>
#include <fcntl.h>
#include <cstdlib>
#include <unistd.h>
using namespace rescue;

int main() {
    // Golden wire bytes are independent of the helper's CRC implementation.
    std::array<uint8_t,4> frame{};
    rescue_pack_sensors(frame.data(), 1);
    assert((frame == std::array<uint8_t,4>{0xA6,0x01,0xBB,0xD0}));
    std::array<uint8_t,4> idle{};
    rescue_pack_sensors(idle.data(), 0);
    assert((idle == std::array<uint8_t,4>{0xA6,0x00,0x7A,0x10}));
    SensorPacketParser parser;
    ActuatorFeedback state;
    auto feed = [&](const auto &bytes, uint64_t time) {
        int count=0;
        for (auto byte:bytes) if(parser.consume(byte,time,state)) ++count;
        return count;
    };
    assert(feed(std::array<uint8_t,2>{0x01,0x55},1000)==0);
    for(size_t i=0;i<3;++i) assert(!parser.consume(frame[i],2000,state));
    assert(state.timestamp_us == 0 && !state.valid);
    assert(parser.consume(frame[3],2001,state));
    assert(state.timestamp_us == 2001 && state.gripper_done == 1 && state.valid);
    assert(feed(idle,3000)==1 && state.gripper_done==0);
    assert(feed(frame,3000)==1 && state.gripper_done==1); // concatenated frames
    auto corrupt=frame; corrupt[3]^=1;
    assert(feed(corrupt,4000)==0 && state.timestamp_us==3000);
    assert(feed(frame,5000)==1);
    auto dropped=std::vector<uint8_t>(frame.begin(),frame.end());
    dropped.erase(dropped.begin()+1);
    assert(feed(dropped,6000)==0 && feed(frame,7000)==1);
    for(int i=0;i<2;++i) assert(!parser.consume(frame[i],8000,state));
    assert(feed(frame,200000)==1); // partial frame timeout
    assert(!parser.consume(0xA6,201000,state));
    assert(feed(frame,1000)==1); // time moves backwards: discard partial bytes
    assert(feed(std::array<uint8_t,3>{0xA6,0xA6,0xA6},2000)==0);
    assert(feed(frame,3000)==1); // repeated header resynchronization
    rescue_pack_sensors(frame.data(),0xA6);
    assert(feed(frame,4000)==1 && state.gripper_done==0xA6); // raw payload retained
    rescue_pack_sensors(frame.data(),1);

    // A virtual terminal exercises real read/write code without opening hardware.
    int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
    assert(master>=0 && grantpt(master)==0 && unlockpt(master)==0);
    UARTController uart;
    uart.initUART(ptsname(master),115200,false,false);
    assert(write(master,frame.data(),2)==2);
    std::this_thread::sleep_for(Ms(20));
    assert(uart.latestActuatorFeedback().timestamp_us==0);
    assert(write(master,frame.data()+2,2)==2);
    auto deadline=Clock::now()+Ms(1000);
    while(uart.latestActuatorFeedback().timestamp_us==0 && Clock::now()<deadline)
        std::this_thread::sleep_for(Ms(5));
    auto received=uart.latestActuatorFeedback();
    assert(received.timestamp_us>0 && received.gripper_done==1 && received.valid);
    ActuatorFeedback older{received.timestamp_us-1,0,true};
    uart.publishActuatorFeedback(older);
    assert(uart.latestActuatorFeedback().gripper_done==1);
    assert(write(master,corrupt.data(),corrupt.size())==4);
    std::this_thread::sleep_for(Ms(30));
    assert(uart.latestActuatorFeedback().timestamp_us==received.timestamp_us);
    assert(uart.getLatestCmd()==0xAA);
    std::this_thread::sleep_for(Ms(220));
    const auto stale=uart.latestActuatorFeedback();
    assert(stale.timestamp_us==received.timestamp_us && !stale.valid);
    assert(write(master,frame.data(),frame.size())==4);
    deadline=Clock::now()+Ms(1000);
    while(uart.latestActuatorFeedback().timestamp_us==received.timestamp_us && Clock::now()<deadline)
        std::this_thread::sleep_for(Ms(5));
    assert(uart.latestActuatorFeedback().valid);

    MotionCommand command;
    command.vx_mps=0.1f; command.wz_rps=-0.25f; command.gripper_closed=1;
    assert(uart.sendMotion(command));
    std::array<uint8_t,10> outgoing{};
    size_t count=0;
    deadline=Clock::now()+Ms(1000);
    while(count<outgoing.size() && Clock::now()<deadline) {
        auto n=read(master,outgoing.data()+count,outgoing.size()-count);
        if(n>0) count+=static_cast<size_t>(n);
        else std::this_thread::sleep_for(Ms(5));
    }
    assert(count==10);
    assert((outgoing==std::array<uint8_t,10>{0x56,0xCD,0xCC,0xCC,0x3D,0,0,0x80,0xBE,1}));
    assert(!uart.execute()); // incompatible 13-byte legacy transport must not transmit
    uint8_t extra=0;
    assert(read(master,&extra,1)<0);
    uart.closePort();
    assert(!uart.latestActuatorFeedback().valid);
    assert(!uart.sendMotion(command));
    close(master);
}
