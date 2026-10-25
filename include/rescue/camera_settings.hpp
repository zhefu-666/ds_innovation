#pragma once

#include <opencv2/videoio.hpp>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <string>

namespace rescue {

// Keep UVC auto exposure enabled, but prevent it lowering a requested 60 FPS.
inline bool disableCameraDynamicFrameRate(int index) {
    const std::string device = "/dev/video" + std::to_string(index);
    const int fd = open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return false;
    v4l2_control control{};
    control.id = V4L2_CID_EXPOSURE_AUTO_PRIORITY;
    control.value = 0;
    const bool set = ioctl(fd, VIDIOC_S_CTRL, &control) == 0;
    const bool verified = set && ioctl(fd, VIDIOC_G_CTRL, &control) == 0 && control.value == 0;
    close(fd);
    return verified;
}

inline void configureCameraCapture(cv::VideoCapture &camera, int index,
                                   int width, int height, int fps) {
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    camera.set(cv::CAP_PROP_FPS, fps);
    if (fps >= 60 && !disableCameraDynamicFrameRate(index)) {
        std::cerr << "[CAMERA] Warning: cannot disable automatic frame-rate reduction; "
                     "actual capture may be slower than requested\n";
    }
    std::cout << "[CAMERA] MJPG requested=" << width << "x" << height << "@" << fps
              << " negotiated=" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << "@"
              << camera.get(cv::CAP_PROP_FPS) << " (not measured throughput)\n";
}

} // namespace rescue
