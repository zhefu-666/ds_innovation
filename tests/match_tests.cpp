#include "rescue/match_control.hpp"
#include "rescue/match_server.hpp"
#include "rescue/config.hpp"
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
using namespace rescue;
int main(){
    assert(Config{}.match_seconds==180);
    { // Explicit debugging override survives 180s, but STOP and health still latch.
        MatchConfig c;c.duration_us=180000000;c.time_limit_enabled=false;c.require_measured_progress=false;
        MatchControl m(c);m.health(1000000,true,"");assert(m.command(MatchCommand::START,1000000));
        for(uint64_t t=1100000;t<=182000000;t+=100000)m.health(t,true,"");
        assert(m.status(182000000).permit && m.status(182000000).elapsed_us>180000000);
        assert(m.command(MatchCommand::STOP,182000000));assert(!m.status(182000000).permit);
        assert(m.command(MatchCommand::RESUME,182000000));
        m.health(182100000,false,"imu_stale");assert(!m.status(182100000).permit);
        m.health(182200000,true,"");assert(!m.status(182200000).permit);
        assert(m.command(MatchCommand::RESUME,182200000));
        assert(m.command(MatchCommand::FINISH,182200000));assert(!m.status(182200000).permit);
    }

    { // Controlled local-vision tests retain timing/health gates without requiring odometry.
        MatchConfig c;c.duration_us=180000000;c.require_measured_progress=false;
        MatchControl m(c);m.health(1000000,true,"");assert(m.command(MatchCommand::START,1000000));
        for(uint64_t t=1100000;t<=21000000;t+=100000)m.health(t,true,"");
        assert(m.status(21000000).permit);
        m.health(21100000,false,"imu_stale");assert(!m.status(21100000).permit);
    }
    {
        MatchControl m({180000000});m.health(1000000,true,"");
        m.command(MatchCommand::STOP,1000000);assert(m.command(MatchCommand::START,1000000));
        m.command(MatchCommand::STOP,1100000);
        assert(m.status(180999999).state==MatchState::PAUSED);
        assert(m.status(181000000).state==MatchState::FINISHED);
    }

    {
        MatchControl m({});m.health(1000000,true,"");
        assert(!m.command(MatchCommand::START,1000000));
        assert(m.status(1000000).reason=="match_duration_required");
    }
    {
        MatchControl m({30000000});
        assert(!m.command(MatchCommand::START,1000000));
        m.health(1000000,true,"");assert(m.command(MatchCommand::START,1000000));
        assert(!m.command(MatchCommand::START,1050000)); // cannot extend duration
        assert(m.status(1050000).permit);
        assert(m.command(MatchCommand::STOP,1100000));
        m.health(1150000,true,"");assert(!m.status(1150000).permit);
        assert(!m.command(MatchCommand::START,1150000));
        assert(m.command(MatchCommand::RESUME,1150000));
        m.health(1200000,false,"imu_tilt");assert(m.status(1200000).state==MatchState::FAULT);
        m.health(1250000,true,"");assert(!m.status(1250000).permit);
        assert(m.command(MatchCommand::RESUME,1250000));
        assert(!m.status(1500001).permit); // independent health deadline, no main update
        m.health(1550000,true,"");assert(!m.status(1550000).permit);
        assert(m.command(MatchCommand::RESUME,1550000));
        assert(m.command(MatchCommand::STOP,1600000));
        assert(m.status(31000000).state==MatchState::FINISHED); // clock runs through stops
        m.health(31000000,true,"");assert(!m.command(MatchCommand::START,31000000));
        assert(!m.command(MatchCommand::RESUME,31000000));
    }
    {
        MatchControl m({60000000});m.health(1000000,true,"");m.command(MatchCommand::START,1000000);
        // Fresh health and repeated unchanged measured poses do not reset the watchdog.
        for(uint64_t t=1100000;t<=16000000;t+=100000){m.health(t,true,"");m.measuredPosition(t,0,0,"zone-A",0);}
        assert(m.status(16000000).reason=="no_measured_movement");
        assert(!m.command(MatchCommand::RESUME,16000000));
    }
    {
        MatchControl m({60000000});m.health(1000000,true,"");m.command(MatchCommand::START,1000000);
        for(uint64_t t=1100000;t<=21000000;t+=100000) {
            m.health(t,true,"");m.measuredPosition(t,double(t-1000000)*1e-7,0,"zone-A",0);
            assert(m.status(t).permit);
        }
        assert(m.command(MatchCommand::FINISH,21000000));assert(!m.status(21000000).permit);
    }
    {
        MatchControl m({60000000});m.health(1000000,true,"");m.command(MatchCommand::START,1000000);
        // Frame/reference changes and future timestamps are not proof of translation.
        for(uint64_t t=1100000;t<=16000000;t+=100000) {
            m.health(t,true,"");m.measuredPosition(t,100,100,std::to_string(t),0);
            m.measuredPosition(t+100000000,1000,1000,"future",0);
        }
        assert(!m.status(16000000).permit);
    }
    {
        // Real local command round trip, duplicate endpoint refusal and unlink on shutdown.
        char directory[]="/tmp/rescue-match-test-XXXXXX";assert(mkdtemp(directory));
        const std::string server=std::string(directory)+"/server",client=std::string(directory)+"/client";
        MatchControl m({1000000});
        {
            MatchServer s(m,server);
            struct stat st{};assert(stat(server.c_str(),&st)==0 && (st.st_mode&0777)==0600);
            bool refused=false;try{MatchServer duplicate(m,server);}catch(...){refused=true;}assert(refused);
            int fd=socket(AF_UNIX,SOCK_DGRAM,0);assert(fd>=0);
            timeval timeout{1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
            sockaddr_un local{};local.sun_family=AF_UNIX;strcpy(local.sun_path,client.c_str());
            assert(bind(fd,reinterpret_cast<sockaddr*>(&local),sizeof(local))==0);
            sockaddr_un peer{};peer.sun_family=AF_UNIX;strcpy(peer.sun_path,server.c_str());
            const auto request=[&](const char* command){
                assert(sendto(fd,command,strlen(command),0,reinterpret_cast<sockaddr*>(&peer),sizeof(peer))>0);
                char buffer[1024];const auto n=recv(fd,buffer,sizeof(buffer),0);assert(n>0);return std::string(buffer,size_t(n));
            };
            assert(request("status").find("OK WAITING")!=std::string::npos);
            assert(request("start").find("REJECTED")!=std::string::npos);
            const auto now=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            m.health(now,true,"");assert(request("start").find("OK RUNNING")!=std::string::npos);
            assert(request("stop").find("OK PAUSED")!=std::string::npos);
            assert(request("finish").find("OK FINISHED")!=std::string::npos);
            assert(request("resume").find("REJECTED FINISHED")!=std::string::npos);
            close(fd);unlink(client.c_str());
        }
        assert(access(server.c_str(),F_OK)!=0);rmdir(directory);
    }
    std::cout<<"Match deadlines, fault latching, measured-motion watchdog and local control passed\n";
}
