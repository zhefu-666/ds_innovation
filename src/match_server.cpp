#include "rescue/match_server.hpp"
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
namespace rescue {
namespace {
uint64_t clockUs(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
MatchServer::MatchServer(MatchControl& c,const std::string& path):control_(c),path_(path) {
    sockaddr_un address{}; address.sun_family=AF_UNIX;
    if(path.empty() || path[0]!='/' || path.size()>=sizeof(address.sun_path))throw std::runtime_error("Invalid match socket path");
    std::memcpy(address.sun_path,path.c_str(),path.size()+1);
    fd_=::socket(AF_UNIX,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(fd_<0)throw std::runtime_error("Cannot create match socket");
    if(::bind(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0) {
        ::close(fd_);fd_=-1;throw std::runtime_error("Cannot bind match socket (already running or stale path): "+path);
    }
    if(::chmod(path.c_str(),0600)!=0) {::close(fd_);::unlink(path.c_str());throw std::runtime_error("Cannot protect match socket");}
    try {worker_=std::thread(&MatchServer::run,this);}catch(...) {::close(fd_);::unlink(path.c_str());throw;}
}
MatchServer::~MatchServer(){stop_=true;if(worker_.joinable())worker_.join();if(fd_>=0)::close(fd_);::unlink(path_.c_str());}
void MatchServer::run(){
    while(!stop_) {
        pollfd p{fd_,POLLIN,0};::poll(&p,1,40);
        const auto now=clockUs();control_.status(now); // deadline/health checks even with blocked inference
        if(!(p.revents&POLLIN))continue;
        char bytes[64];sockaddr_un peer{};socklen_t length=sizeof(peer);
        const auto n=::recvfrom(fd_,bytes,sizeof(bytes),0,reinterpret_cast<sockaddr*>(&peer),&length);
        if(n<=0)continue;
        const std::string command(bytes,size_t(n));bool accepted=false;
        if(command=="start")accepted=control_.command(MatchCommand::START,clockUs());
        else if(command=="stop")accepted=control_.command(MatchCommand::STOP,clockUs());
        else if(command=="resume")accepted=control_.command(MatchCommand::RESUME,clockUs());
        else if(command=="finish")accepted=control_.command(MatchCommand::FINISH,clockUs());
        else if(command=="status")accepted=true;
        const auto s=control_.status(clockUs());
        const std::string response=std::string(accepted?"OK ":"REJECTED ")+MatchControl::name(s.state)+
            " reason="+s.reason+" elapsed_ms="+std::to_string(s.elapsed_us/1000)+
            " remaining_ms="+std::to_string(s.remaining_us/1000)+" permit="+(s.permit?"1":"0");
        ::sendto(fd_,response.data(),response.size(),MSG_DONTWAIT,reinterpret_cast<sockaddr*>(&peer),length);
    }
}
}
