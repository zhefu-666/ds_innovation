#include "rescue/control/match_control.hpp"
#include "rescue/control/match_server.hpp"
#include "rescue/config/config.hpp"
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
using namespace rescue;
void buttonRestartsCreateFreshRounds() {
    MatchConfig c;
    c.duration_us = 1000000;
    c.require_measured_progress = false;
    MatchControl m(c);
    assert(!m.restartFromButton(1000000, 1000001, true));
    m.health(1100000, true, "");
    assert(!m.restartFromButton(1050000, 1100001, true));
    assert(!m.restartFromButton(1100003, 1100002, true));
    assert(m.restartFromButton(1100002, 1100003, true));
    auto status = m.status(1100004);
    assert(status.state == MatchState::RESTARTING && !status.permit && !status.started_us);
    assert(!m.completeButtonRestart(1100005)); // require new evidence after reset
    m.health(1100010, true, "");
    assert(!m.restartFromButton(1100011, 1100012, true)); // no queued press
    assert(!m.command(MatchCommand::START, 1100013));
    assert(!m.command(MatchCommand::RESUME, 1100014));
    assert(m.completeButtonRestart(1100020));
    status = m.status(1100021);
    assert(status.permit && status.run_id == 1 && status.started_us == 1100020);
    assert(!m.restartFromButton(1100011, 1100022, true));
    uint64_t t = 1200000;
    // RUNNING, PAUSED, FAULT, FINISHED all become new rounds, not resumes.
    for (int state = 0; state < 4; ++state) {
        if (state == 1) assert(m.command(MatchCommand::STOP, t));
        if (state == 2) m.health(t, false, "task_fault");
        if (state == 3) assert(m.command(MatchCommand::FINISH, t));
        if (state != 2) m.health(t + 1, true, "");
        assert(m.restartFromButton(t + 2, t + 3, true));
        assert(!m.status(t + 4).started_us && !m.status(t + 4).permit);
        m.health(t + 5, true, "");
        assert(m.completeButtonRestart(t + 6));
        status = m.status(t + 7);
        assert(status.run_id == uint64_t(state + 2) && status.started_us == t + 6);
        assert(status.elapsed_us == 1 && status.remaining_us == c.duration_us - 1);
        t += 100000;
    }
    m.health(t, false, "mcu_feedback_stale");
    assert(!m.restartFromButton(t + 1, t + 2, false));
    m.health(t + 3, true, "");
    assert(!m.restartFromButton(t + 1, t + 4, true)); // rejected event cannot start later
    assert(!m.restartFromButton(t + 5, t + 400006, true));
    m.health(t + 500000, true, "");
    assert(m.restartFromButton(t + 500001, t + 500002, true));
    assert(m.command(MatchCommand::STOP, t + 500003));
    m.health(t + 500004, true, "");
    assert(!m.completeButtonRestart(t + 500005) && !m.status(t + 500005).permit);
    assert(m.restartFromButton(t + 500006, t + 500007, true));
    assert(m.command(MatchCommand::FINISH, t + 500008));
    assert(!m.completeButtonRestart(t + 500009));
    m.health(t + 500009, true, ""); // every new request needs a current hardware observation
    assert(m.restartFromButton(t + 500010, t + 500011, true));
    m.failButtonRestart(t + 500012, "restart_preparation_timeout");
    m.health(t + 500013, true, "");
    assert(!m.completeButtonRestart(t + 500014));
    assert(m.status(t + 500014).reason == "restart_preparation_timeout");
    assert(m.restartFromButton(t + 500015, t + 500016, true));
    m.health(t + 500017, true, "");
    assert(m.completeButtonRestart(t + 500018));
    assert(m.status(t + 1500018).state == MatchState::FINISHED);
    m.health(t + 1500020, true, "");
    assert(m.restartFromButton(t + 1500021, t + 1500022, true));
    m.health(t + 1500023, true, "");
    assert(m.completeButtonRestart(t + 1500024));
    assert(m.status(t + 1500025).elapsed_us == 1);
    std::cout << "PASS A7 new-round timing, faults, replay rejection and cancellation\n";
}
int main() {
    buttonRestartsCreateFreshRounds();
    assert(Config{}.match_seconds == 180);
    {
        MatchControl m({180000000});
        assert(!m.startFromButton(999999, 1000000));
        m.health(1000000, true, "");
        assert(m.status(1000000).ready && !m.status(1000000).permit);
        assert(!m.startFromButton(999999, 1000001));
        // Ten minutes of preparation consume none of the 180-second match.
        for (uint64_t t = 1100000; t <= 601000000; t += 100000)
            m.health(t, true, "");
        const auto waiting = m.status(601000000);
        assert(waiting.state == MatchState::WAITING && waiting.ready && !waiting.permit &&
               waiting.started_us == 0 && waiting.elapsed_us == 0);
        assert(!m.startFromButton(999999, 601000001));
        assert(!m.startFromButton(601000010, 601000001));
        assert(m.startFromButton(601000001, 601000002));
        const auto start = m.status(601000002).started_us;
        assert(!m.startFromButton(601000003, 601000004));
        assert(m.status(601000004).started_us == start);
        assert(m.command(MatchCommand::STOP, 601000005));
        assert(!m.startFromButton(601000006, 601000007));
        assert(!m.status(601000007).permit);
        assert(m.command(MatchCommand::RESUME, 601000008));
        m.health(601000009, false, "camera_pitch_feedback_invalid");
        assert(!m.status(601000009).ready && !m.status(601000009).permit);
        m.health(601000010, true, "");
        assert(!m.startFromButton(601000011, 601000012));
    }
    {
        MatchControl m({180000000});
        m.health(1000000, true, "");
        m.health(1100000, false, "mcu_feedback_stale");
        assert(!m.startFromButton(1100001, 1100002));
        m.health(1200000, true, "");
        assert(!m.startFromButton(1100001, 1200001));
        assert(m.startFromButton(1200001, 1200002));
    }
    {
        MatchConfig c;
        c.time_limit_enabled = false;
        c.require_measured_progress = false;
        MatchControl m(c);
        m.health(1000000, true, "");
        assert(!m.command(MatchCommand::NEXT, 1000000));
        assert(m.command(MatchCommand::START, 1000000));
        assert(m.pauseForPhase("CAPTURE", 1000000));
        assert(m.status(1000000).phase_waiting && !m.status(1000000).permit);
        assert(m.status(1000000).reason == "phase_complete_next_CAPTURE");
        assert(!m.command(MatchCommand::RESUME, 1000000));
        m.health(301000000, false, "imu_stale");
        assert(!m.command(MatchCommand::NEXT, 301000000));
        m.health(301100000, true, "");
        assert(m.command(MatchCommand::NEXT, 301100000));
        assert(!m.status(301100000).phase_waiting && m.status(301100000).permit);
        assert(!m.command(MatchCommand::NEXT, 301100000)); // no queued steps
        assert(m.pauseForPhase("TRANSPORT", 301100000));
        assert(m.command(MatchCommand::FINISH, 301100000));
        assert(!m.command(MatchCommand::NEXT, 301100000));
        assert(!m.status(301100000).permit);
    }
    { // Explicit debugging override survives 180s, but STOP and health still latch.
        MatchConfig c;
        c.duration_us = 180000000;
        c.time_limit_enabled = false;
        c.require_measured_progress = false;
        MatchControl m(c);
        m.health(1000000, true, "");
        assert(m.command(MatchCommand::START, 1000000));
        for (uint64_t t = 1100000; t <= 182000000; t += 100000)
            m.health(t, true, "");
        assert(m.status(182000000).permit && m.status(182000000).elapsed_us > 180000000);
        assert(m.command(MatchCommand::STOP, 182000000));
        assert(!m.status(182000000).permit);
        assert(m.command(MatchCommand::RESUME, 182000000));
        m.health(182100000, false, "imu_stale");
        assert(!m.status(182100000).permit);
        m.health(182200000, true, "");
        assert(!m.status(182200000).permit);
        assert(m.command(MatchCommand::RESUME, 182200000));
        assert(m.command(MatchCommand::FINISH, 182200000));
        assert(!m.status(182200000).permit);
    }

    { // Controlled local-vision tests retain timing/health gates without requiring odometry.
        MatchConfig c;
        c.duration_us = 180000000;
        c.require_measured_progress = false;
        MatchControl m(c);
        m.health(1000000, true, "");
        assert(m.command(MatchCommand::START, 1000000));
        for (uint64_t t = 1100000; t <= 21000000; t += 100000)
            m.health(t, true, "");
        assert(m.status(21000000).permit);
        m.health(21100000, false, "imu_stale");
        assert(!m.status(21100000).permit);
    }
    {
        MatchControl m({180000000});
        m.health(1000000, true, "");
        m.command(MatchCommand::STOP, 1000000);
        assert(m.command(MatchCommand::START, 1000000));
        m.command(MatchCommand::STOP, 1100000);
        assert(m.status(180999999).state == MatchState::PAUSED);
        assert(m.status(181000000).state == MatchState::FINISHED);
    }

    {
        MatchControl m({});
        m.health(1000000, true, "");
        assert(!m.command(MatchCommand::START, 1000000));
        assert(m.status(1000000).reason == "match_duration_required");
    }
    {
        MatchControl m({30000000});
        assert(!m.command(MatchCommand::START, 1000000));
        m.health(1000000, true, "");
        assert(m.command(MatchCommand::START, 1000000));
        assert(!m.command(MatchCommand::START, 1050000)); // cannot extend duration
        assert(m.status(1050000).permit);
        assert(m.command(MatchCommand::STOP, 1100000));
        m.health(1150000, true, "");
        assert(!m.status(1150000).permit);
        assert(!m.command(MatchCommand::START, 1150000));
        assert(m.command(MatchCommand::RESUME, 1150000));
        m.health(1200000, false, "imu_tilt");
        assert(m.status(1200000).state == MatchState::FAULT);
        m.health(1250000, true, "");
        assert(!m.status(1250000).permit);
        assert(m.command(MatchCommand::RESUME, 1250000));
        assert(m.status(1550000).permit); // 300ms remains valid
        assert(!m.status(1550001).permit); // independent health deadline, no main update
        m.health(1560000, true, "");
        assert(!m.status(1560000).permit);
        assert(m.command(MatchCommand::RESUME, 1560000));
        assert(m.command(MatchCommand::STOP, 1600000));
        assert(m.status(31000000).state == MatchState::FINISHED); // clock runs through stops
        m.health(31000000, true, "");
        assert(!m.command(MatchCommand::START, 31000000));
        assert(!m.command(MatchCommand::RESUME, 31000000));
    }
    {
        MatchControl m({60000000});
        m.health(1000000, true, "");
        m.command(MatchCommand::START, 1000000);
        // Fresh health and repeated unchanged measured poses do not reset the watchdog.
        for (uint64_t t = 1100000; t <= 16000000; t += 100000) {
            m.health(t, true, "");
            m.measuredPosition(t, 0, 0, "zone-A", 0);
        }
        assert(m.status(16000000).reason == "no_measured_movement");
        assert(!m.command(MatchCommand::RESUME, 16000000));
    }
    {
        MatchControl m({60000000});
        m.health(1000000, true, "");
        m.command(MatchCommand::START, 1000000);
        for (uint64_t t = 1100000; t <= 21000000; t += 100000) {
            m.health(t, true, "");
            m.measuredPosition(t, double(t - 1000000) * 1e-7, 0, "zone-A", 0);
            assert(m.status(t).permit);
        }
        assert(m.command(MatchCommand::FINISH, 21000000));
        assert(!m.status(21000000).permit);
    }
    {
        MatchControl m({60000000});
        m.health(1000000, true, "");
        m.command(MatchCommand::START, 1000000);
        // Frame/reference changes and future timestamps are not proof of translation.
        for (uint64_t t = 1100000; t <= 16000000; t += 100000) {
            m.health(t, true, "");
            m.measuredPosition(t, 100, 100, std::to_string(t), 0);
            m.measuredPosition(t + 100000000, 1000, 1000, "future", 0);
        }
        assert(!m.status(16000000).permit);
    }
    {
        // Real local command round trip, duplicate endpoint refusal and unlink on shutdown.
        char directory[] = "/tmp/rescue-match-test-XXXXXX";
        assert(mkdtemp(directory));
        const std::string server = std::string(directory) + "/server",
                          client = std::string(directory) + "/client";
        MatchControl m({1000000});
        {
            MatchServer s(m, server);
            struct stat st {};
            assert(stat(server.c_str(), &st) == 0 && (st.st_mode & 0777) == 0600);
            bool refused = false;
            try {
                MatchServer duplicate(m, server);
            } catch (...) {
                refused = true;
            }
            assert(refused);
            int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
            assert(fd >= 0);
            timeval timeout{1, 0};
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            sockaddr_un local{};
            local.sun_family = AF_UNIX;
            strcpy(local.sun_path, client.c_str());
            assert(bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == 0);
            sockaddr_un peer{};
            peer.sun_family = AF_UNIX;
            strcpy(peer.sun_path, server.c_str());
            const auto request = [&](const char* command) {
                assert(sendto(fd,
                              command,
                              strlen(command),
                              0,
                              reinterpret_cast<sockaddr*>(&peer),
                              sizeof(peer)) > 0);
                char buffer[1024];
                const auto n = recv(fd, buffer, sizeof(buffer), 0);
                assert(n > 0);
                return std::string(buffer, size_t(n));
            };
            const auto waiting = request("status");
            assert(waiting.find("OK WAITING") != std::string::npos);
            assert(waiting.find("ready=0") != std::string::npos);
            assert(request("start").find("REJECTED") != std::string::npos);
            const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
            m.health(now, true, "");
            assert(request("status").find("ready=1") != std::string::npos);
            assert(request("start").find("OK RUNNING") != std::string::npos);
            assert(m.pauseForPhase("SEARCH", now));
            assert(request("status").find("phase_waiting=1") != std::string::npos);
            assert(request("resume").find("REJECTED PAUSED") != std::string::npos);
            assert(request("next").find("OK RUNNING") != std::string::npos);
            assert(request("next").find("REJECTED RUNNING") != std::string::npos);
            const auto fault_now = std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::steady_clock::now().time_since_epoch()).count();
            m.health(fault_now, false, "startup_handoff_identity_uncertain");
            const auto fault = request("status");
            assert(fault.find("OK FAULT reason=startup_handoff_identity_uncertain") != std::string::npos);
            assert(fault.find("ready=0 permit=0") != std::string::npos);
            assert(request("next").find("REJECTED FAULT") != std::string::npos);
            assert(request("resume").find("REJECTED FAULT") != std::string::npos);
            assert(request("stop").find("OK PAUSED") != std::string::npos);
            assert(request("finish").find("OK FINISHED") != std::string::npos);
            assert(request("resume").find("REJECTED FINISHED") != std::string::npos);
            close(fd);
            unlink(client.c_str());
        }
        assert(access(server.c_str(), F_OK) != 0);
        rmdir(directory);
    }
    std::cout
        << "Match deadlines, fault latching, measured-motion watchdog and local control passed\n";
}
