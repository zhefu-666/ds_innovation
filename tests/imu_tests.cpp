#include "rescue/hipnuc_imu.hpp"
#include "rescue/config.hpp"
#include <cassert>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <limits>
#include <functional>
#include <chrono>
#include <thread>
#include <iostream>
using namespace rescue;
using Bytes = std::vector<uint8_t>;
void checksum(Bytes& frame) {
    uint16_t crc = 0;
    for (size_t j = 0; j < frame.size(); ++j) {
        if (j == 4 || j == 5) continue;
        crc ^= uint16_t(frame[j]) << 8;
        for (int i = 0; i < 8; ++i) crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
    frame[4] = crc & 255; frame[5] = crc >> 8;
}
void timeAt(Bytes& b, uint32_t value) { for (int i=0;i<4;++i) b[14+i] = (value >> (i*8)) & 255; checksum(b); }
void waitUntil(const std::function<bool()>& condition) {
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!condition() && std::chrono::steady_clock::now() < end) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    assert(condition());
}
int main() {
    std::ifstream file(IMU_FIXTURE_PATH, std::ios::binary);
    Bytes frame{std::istreambuf_iterator<char>(file), {}};
    assert(frame.size() == 82 && frame[6] == 0x91);
    Hi91Parser parser; ImuSample sample;
    for (size_t i=0;i<frame.size();++i) assert(parser.consume(frame[i], 1000, sample) == (i==81));
    assert(sample.status == 0x2323 && sample.device_time_ms == 3912659);
    assert(sample.measurements_valid && std::abs(sample.rpy_rad[0] - 3.138964f) < 0.0001f);
    assert(sample.acceleration_mps2[2] < -9.0f && sample.acceleration_mps2[2] > -10.5f);
    assert(std::abs(sample.quaternion_wxyz[2] - 0.99998790f) < 0.00001f);
    // Mounting (roll 180°): the level, inverted fixture reads +1 g up and ~0 tilt in base_link.
    constexpr float deg = 0.017453292519943295f;
    assert(sample.body_acceleration_mps2[2] > 9.0f && sample.body_acceleration_mps2[2] < 10.5f);
    assert(std::abs(sample.body_rpy_rad[0]) < 3 * deg && std::abs(sample.body_rpy_rad[1]) < 3 * deg);
    assert(std::abs(sample.acceleration_mps2[2] + sample.body_acceleration_mps2[2]) < 1e-6f); // raw kept
    {
        // Real frames from the 2026-09-27 hand-motion capture, re-encoded into the fixture layout.
        const auto floats = [](Bytes f, size_t offset, std::initializer_list<float> values) {
            for (float v : values) { uint32_t n; std::memcpy(&n, &v, 4); for (int i=0;i<4;++i) f[offset++] = (n >> (8*i)) & 255; }
            return f;
        };
        const auto decode = [&](Bytes f) {
            checksum(f); Hi91Parser p; ImuSample s; unsigned done = 0;
            for (auto v : f) done += p.consume(v, 1000, s);
            assert(done == 1 && s.measurements_valid);
            return s;
        };
        // Nose raised ~25°: body pitch is negative (REP-103, nose-down positive), roll stays small.
        auto nose = decode(floats(floats(frame, 18, {0.437869f, -0.078395f, -0.896114f}),
                                  66, {-0.153103f, -0.563852f, 0.796467f, -0.158187f}));
        assert(nose.body_rpy_rad[1] < -20 * deg && nose.body_rpy_rad[1] > -30 * deg);
        assert(std::abs(nose.body_rpy_rad[0]) < 8 * deg);
        assert(nose.body_acceleration_mps2[0] > 3.5f && nose.body_acceleration_mps2[2] > 8.0f);
        // Left (counter-clockwise) turn: device gyro z is negative, body yaw rate is positive.
        auto turn = decode(floats(frame, 30, {21.6225f, 5.9366f, -103.7951f}));
        assert(std::abs(turn.body_angular_velocity_rps[2] - 103.7951f * deg) < 1e-4f);
        assert(std::abs(turn.body_angular_velocity_rps[0] - 21.6225f * deg) < 1e-4f);
    }
    // CRC failure and unrelated serial noise cannot refresh a measurement.
    auto bad=frame; bad[30]^=1;
    const auto old=sample.received_us;
    for(auto v:bad) assert(!parser.consume(v,2000,sample));
    assert(sample.received_us==old && parser.crc_errors==1);
    for(auto v:Bytes{0x00,0x5a,0xff}) parser.consume(v,3000,sample);
    unsigned completed=0;
    for(int j=0;j<2;++j)for(auto v:frame)completed+=parser.consume(v,4000,sample);
    assert(completed==2);
    // Partial-frame timeout and unreasonable length recover at next good frame.
    parser.consume(0x5a,5000,sample);parser.consume(0xa5,5000,sample);
    completed=0;for(auto v:frame)completed+=parser.consume(v,200000,sample);assert(completed==1);
    for(auto v:Bytes{0x5a,0xa5,0xff,0xff,0,0})parser.consume(v,201000,sample);
    completed=0;for(auto v:frame)completed+=parser.consume(v,202000,sample);assert(completed==1);
    // Valid CRC is insufficient for usable floating-point data.
    auto nan=frame; uint32_t bits=0x7fc00000;
    for(int i=0;i<4;++i)nan[18+i]=(bits>>(8*i))&255;
    checksum(nan);for(auto v:nan)parser.consume(v,203000,sample);assert(!sample.measurements_valid);
    auto unsupported=frame;unsupported[6]=0x90;checksum(unsupported);
    completed=0;for(auto v:unsupported)completed+=parser.consume(v,204000,sample);assert(completed==0);
    // A closed-loop pseudo terminal exercises actual serial setup, timing and disconnection without hardware.
    int master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);assert(master>=0&&grantpt(master)==0&&unlockpt(master)==0);
    const std::string port=ptsname(master);
    {
        HipnucImu imu(port,115200,60);
        bool locked=false;try{HipnucImu other(port);}catch(...){locked=true;}assert(locked);
        assert(!imu.snapshot().fresh);
        timeAt(frame,1000);
        assert(write(master,frame.data(),17)==17);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));assert(!imu.snapshot().fresh);
        assert(write(master,frame.data()+17,65)==65);
        waitUntil([&]{return imu.snapshot().fresh;});
        auto first=imu.snapshot();assert(first.sample.measurements_valid);
        assert(!imu.snapshotAt(first.sample.received_us-1).fresh); // no future samples
        const auto at_capture=imu.snapshotAt(first.sample.received_us);
        assert(at_capture.fresh&&at_capture.sample.sequence==first.sample.sequence);
        assert(!imu.snapshotAt(first.sample.received_us+60000,50).fresh);
        uint8_t output;assert(read(master,&output,1)<0); // Reader sends no commands.
        assert(write(master,frame.data(),82)==82);
        waitUntil([&]{return imu.snapshot().duplicate_times==1;});
        assert(imu.snapshot().sample.received_us==first.sample.received_us);
        waitUntil([&]{return !imu.snapshot().fresh;});
        timeAt(frame,900);assert(write(master,frame.data(),82)==82);
        waitUntil([&]{return imu.snapshot().backward_times==1;});assert(!imu.snapshot().fresh);
        timeAt(frame,1010);assert(write(master,frame.data(),82)==82);waitUntil([&]{return imu.snapshot().fresh;});
        close(master);waitUntil([&]{return !imu.snapshot().connected;});assert(!imu.snapshot().fresh);
    }
    // Device counter uint32 rollover is accepted; restarting at a smaller time is not.
    master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);assert(master>=0&&grantpt(master)==0&&unlockpt(master)==0);
    {
        HipnucImu imu(ptsname(master));timeAt(frame,0xfffffff8U);assert(write(master,frame.data(),82)==82);
        waitUntil([&]{return imu.snapshot().fresh;});
        timeAt(frame,2);assert(write(master,frame.data(),82)==82);
        waitUntil([&]{return imu.snapshot().sample.device_time_ms==2;});assert(imu.snapshot().backward_times==0);
    }
    close(master);
    const char* args[]={"test","--dry-run","--imu","--imu-port","/dev/ttyUSB0","--imu-timeout-ms","150"};
    auto config=parseArgs(7,const_cast<char**>(args));assert(config.imu&&config.imu_timeout_ms==150);
    const char* wrong[]={"test","--imu-timeout-ms","-1"};bool rejected=false;
    try{parseArgs(3,const_cast<char**>(wrong));}catch(...){rejected=true;}assert(rejected);
    const auto invalidConfig=[](std::vector<const char*> args) {
        bool threw=false;try{parseArgs(int(args.size()),const_cast<char**>(args.data()));}catch(...){threw=true;}
        assert(threw);
    };
    invalidConfig({"test","--geometry-replay","record.json","--imu","--calibration","camera.yaml"});
    invalidConfig({"test","--calibration","camera.yaml","--dry-run"});
    invalidConfig({"test","--pitch-feedback"});
    invalidConfig({"test","--keypoints-file","points.json"});
    invalidConfig({"test","--geometry-replay","record.json","--push-replay","push.json","--calibration","camera.yaml"});
    invalidConfig({"test","--hardware"});                                         // no IMU veto
    invalidConfig({"test","--hardware","--imu","--dry-run"});
    invalidConfig({"test","--hardware","--imu","--pitch-feedback","--calibration","camera.yaml"});
    invalidConfig({"test","--hardware","--imu","--push-replay","push.json"});
    invalidConfig({"test","--hardware","--imu","--port","/dev/ttyUSB0"});         // same port as the IMU
    invalidConfig({"test","--pitch-presets","0,2500"});                           // FAR,TRACK,NEAR
    invalidConfig({"test","--pitch-presets","2500,0,2500"});                      // not ordered
    invalidConfig({"test","--pitch-presets","0,2500,4000"});                      // beyond the servo
    invalidConfig({"test","--pitch-presets","0,25.5,2500"});                      // integers only
    {
        const char* none[]={"test","--dry-run"};
        assert((parseArgs(2,const_cast<char**>(none)).pitch_presets_cdeg==std::array<int16_t,3>{0,2500,2500}));
        const char* p[]={"test","--dry-run","--pitch-presets","1200,2200,3500"};
        assert((parseArgs(4,const_cast<char**>(p)).pitch_presets_cdeg==std::array<int16_t,3>{1200,2200,3500}));
    }
    {
        const char* hw[]={"test","--hardware","--imu","--calibration","camera.yaml"};
        const auto c=parseArgs(5,const_cast<char**>(hw));assert(c.hardware&&!c.dry_run&&!c.pitch_feedback);
    }
    std::cout<<"HI91 real fixture, mounting to base_link, CRC, recovery, SI, PTY freshness, duplicate/backward timestamps, disconnect and configuration passed\n";
}
