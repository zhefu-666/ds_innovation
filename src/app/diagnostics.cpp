#include "rescue/app/diagnostics.hpp"
#include <opencv2/imgproc.hpp>

namespace rescue {
void report(const PushOutput &out, std::ostream &stream) {
    stream << "state=" << rescue::PushTask::name(out.state) << " batch=" << out.batch_size
              << " delivered=" << out.delivered_total << " first=" << out.first_ordinary_delivered
              << " vx=" << out.motion.vx_mps << " wz=" << out.motion.wz_rps
              << " frame_offset_deg=" << int(out.motion.gripper_offset)
              << " startup_forward_ms=" << out.startup_commanded_us/1000
              << " search_cue_id=" << out.search_cue_id << " capture_target_id=" << out.capture_target_id
              << " cue=" << out.target_is_search_cue << " clear_attempts=" << out.clearing_attempts
              << " pitch_cmd=" << out.motion.camera_pitch_cdeg
              << " camera_protocol_cmd_deg=" << rescue::cameraPitchToWire(out.motion.camera_pitch_cdeg) << " stop="
              << (out.motion.vx_mps == 0.0f && out.motion.wz_rps == 0.0f)
              << " reason=" << (out.reason.empty() ? "-" : out.reason) << "\n";
}

void drawZone(cv::Mat &frame, const std::vector<ZoneKeypoint> &points) {
    static const char *names[] = {"FL", "FD", "FR", "RL", "RD", "RR"};
    for (const auto &k : points) {
        if (k.id < 0 || k.id > 5) continue;
        cv::circle(frame, k.pixel, 5, {255, 0, 255}, -1);
        cv::putText(frame, names[k.id], k.pixel + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, .45, {255, 0, 255}, 1);
    }
}

void printZone(const ZoneHalves &halves, const std::vector<ZoneKeypoint> &points) {
    static const char *half_names[] = {"zone_left", "zone_right"};
    static const char *kpt_names[] = {"far_left", "near_left", "far_right", "near_right"};
    for (int c = 0; c < 2; ++c) {
        const auto &h = halves[c];
        std::cout << "pose " << half_names[c] << " valid=" << h.valid;
        if (h.valid) {
            std::cout << " score=" << h.score << " box=" << h.box.x << "," << h.box.y << "," << h.box.width << "," << h.box.height;
            for (int k = 0; k < 4; ++k)
                std::cout << " " << kpt_names[k] << "=" << h.keypoints[k].x << "," << h.keypoints[k].y << "@" << h.confidence[k];
        }
        std::cout << "\n";
    }
    std::cout << "zone_keypoints=" << points.size();
    for (const auto &k : points) std::cout << " id" << k.id << "=" << k.pixel.x << "," << k.pixel.y << "@" << k.confidence;
    std::cout << "\n";
}
} // namespace rescue
