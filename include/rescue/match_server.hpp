#pragma once
#include "rescue/match_control.hpp"
#include <atomic>
#include <thread>
namespace rescue {
// Local Unix datagram endpoint, mode 0600. A second process cannot replace an
// existing endpoint. Commands are exact words: start, stop, resume, finish, status.
class MatchServer {
public:
    MatchServer(MatchControl& control,const std::string& path);
    ~MatchServer();
    MatchServer(const MatchServer&)=delete;
    MatchServer& operator=(const MatchServer&)=delete;
private:
    void run();
    MatchControl& control_;
    std::string path_;
    int fd_=-1;
    std::atomic<bool> stop_{false};
    std::thread worker_;
};
}
