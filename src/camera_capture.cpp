#include "rescue/stage_audit.hpp"
#include "rescue/camera_capture.hpp"
#include "rescue/camera_settings.hpp"

#include <chrono>
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>

namespace rescue {

namespace {
uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
} // namespace

CameraCapture::CameraCapture(int index, int width, int height, int fps, int model_size)
    : index_(index), width_(width), height_(height), fps_(fps), model_size_(model_size) {}

CameraCapture::~CameraCapture() { stop(); }

bool CameraCapture::start() {
    if (running_.load()) return true;
    stop(); // Join workers left by a failed/disconnected previous session.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = CameraFrame{};
        delivered_frame_ = captured_frames_ = 0;
        dropped_frames_.store(0);
    }
    {
        std::lock_guard<std::mutex> lock(compressed_mutex_);
        compressed_ = CompressedFrame{};
        compressed_read_ = 0;
        decode_requested_ = true;
        last_consume_us_ = 0;
        consumer_period_ms_ = 38.0;
        decode_cost_ms_ = 12.0;
        decode_target_ = std::chrono::steady_clock::now();
    }
    raw_mjpeg_ = false;
    decode_fps_ = std::min(60, fps_);
    if (width_ <= 0 || height_ <= 0 || fps_ <= 0 || model_size_ < 0) return false;
    if (!capture_.open(index_, cv::CAP_V4L2)) return false;
    configureCameraCapture(capture_, index_, width_, height_, fps_);
    const char *raw = std::getenv("RESCUE_CAPTURE_RAW_MJPEG");
    if (!raw || std::string(raw) != "0") {
        raw_mjpeg_ = capture_.get(cv::CAP_PROP_FOURCC) == cv::VideoWriter::fourcc('M','J','P','G')
            && capture_.set(cv::CAP_PROP_CONVERT_RGB, 0);
        if (!raw_mjpeg_) capture_.set(cv::CAP_PROP_CONVERT_RGB, 1);
    }
    if (const char *limit = std::getenv("RESCUE_CAPTURE_DECODE_FPS")) {
        try { decode_fps_ = std::stoi(limit); } catch (...) { capture_.release(); return false; }
        if (decode_fps_ < 1 || decode_fps_ > fps_) { capture_.release(); return false; }
    }
    std::cout << "[CAMERA] raw_latest_mjpeg=" << raw_mjpeg_
              << " demand_scheduled=1 decode_fps_limit=" << decode_fps_
              << " prepared_input=" << (raw_mjpeg_ ? model_size_ : 0)
              << " timestamp=" << (raw_mjpeg_ ? "compressed_read_complete" : "decoded_read_complete") << "\n";
    running_.store(true);
    if (raw_mjpeg_) decode_thread_ = std::thread(&CameraCapture::decodeLoop, this);
    thread_ = std::thread(&CameraCapture::captureLoop, this);
    return true;
}

void CameraCapture::stop() {
    running_.store(false);
    ready_.notify_all();
    compressed_ready_.notify_all();
    if (decode_thread_.joinable()) decode_thread_.join();
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    if (capture_.isOpened()) capture_.release();
}

bool CameraCapture::latest(CameraFrame &out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_.image.empty()) return false;
    out = latest_;
    out.image = latest_.image.clone();
    if (raw_mjpeg_ && running_.load()) {
        // Snapshot-only callers must also keep the consumer-driven decoder moving.
        std::lock_guard<std::mutex> scheduler(compressed_mutex_);
        decode_requested_ = true;
        compressed_ready_.notify_one();
    }
    return true;
}

bool CameraCapture::waitNext(uint64_t after, CameraFrame &out, int timeout_ms) {
    AuditScope audit("capture.wait_copy");
    const uint64_t requested = nowUs();
    std::unique_lock<std::mutex> lock(mutex_);
    if (!ready_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
            return latest_.frame_number > after || !running_.load();
        }) || !running_.load() || latest_.frame_number <= after) return false;
    out = latest_;
    out.image = latest_.image.clone();
    if (raw_mjpeg_ && out.frame_number > delivered_frame_) {
        std::lock_guard<std::mutex> scheduler(compressed_mutex_);
        const uint64_t consumed = nowUs();
        if (last_consume_us_) {
            // Exclude producer wait: otherwise prediction would throttle a faster consumer.
            const double work = requested >= last_consume_us_ ? (requested - last_consume_us_) / 1000.0 : 0;
            if (work <= 200) {
                const double period = std::max(1000.0 / decode_fps_, work);
                consumer_period_ms_ = 0.75 * consumer_period_ms_ + 0.25 * period;
            }
        }
        last_consume_us_ = consumed;
        // Prepare one next frame just before the estimated next consumption.
        // Prediction affects wake-up timing only; no frame bypasses freshness checks.
        decode_target_ = std::chrono::steady_clock::now() + std::chrono::microseconds(
            static_cast<int64_t>(std::max(0.0, consumer_period_ms_ - decode_cost_ms_ - 3.0) * 1000));
        decode_requested_ = true;
        compressed_ready_.notify_one();
    }
    delivered_frame_ = out.frame_number;
    return true;
}

void CameraCapture::captureLoop() {
    try {
        cv::Mat frame;
        while (running_.load()) {
            bool grabbed=false,retrieved=false;
            {AuditScope audit("capture.grab");grabbed=capture_.grab();}
            if(!grabbed)break;
            {AuditScope audit("capture.retrieve_decode");retrieved=capture_.retrieve(frame);}
            if(!retrieved||frame.empty())break;
            if (raw_mjpeg_) {
                if (frame.type() != CV_8UC1 || !frame.isContinuous() || frame.total() < 4
                    || frame.ptr<unsigned char>()[0] != 0xff || frame.ptr<unsigned char>()[1] != 0xd8)
                    throw std::runtime_error("Camera did not return MJPEG bytes");
                CompressedFrame next;
                next.timestamp_us = nowUs();
                next.epoch_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
                { AuditScope audit("capture.compressed_copy"); next.jpeg = frame.clone(); }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    next.number = ++captured_frames_;
                }
                {
                    std::lock_guard<std::mutex> lock(compressed_mutex_);
                    if (compressed_.number > compressed_read_) dropped_frames_.fetch_add(1);
                    compressed_ = std::move(next);
                }
                compressed_ready_.notify_one();
                continue;
            }
            // Timestamp before copying; both models receive this same image and timestamp.
            CameraFrame next;
            next.timestamp_us = nowUs();
            next.epoch_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            {AuditScope audit("capture.worker_clone");next.image = frame.clone();}
            next.frame_number = ++captured_frames_;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (latest_.frame_number > delivered_frame_) dropped_frames_.fetch_add(1);
                latest_ = std::move(next);
            }
            ready_.notify_all();
        }
    } catch (const std::exception &) {
        // Wake the consumer and fail closed; never infer repeatedly on the last frame.
    }
    running_.store(false);
    ready_.notify_all();
    compressed_ready_.notify_all();
}

void CameraCapture::decodeLoop() {
    try {
        uint64_t last = 0;
        auto next_decode = std::chrono::steady_clock::now();
        while (running_.load()) {
            CompressedFrame compressed;
            {
                std::unique_lock<std::mutex> lock(compressed_mutex_);
                compressed_ready_.wait(lock, [&] { return !running_.load() || decode_requested_; });
                if (!running_.load()) break;
                const auto target = std::max(next_decode, decode_target_);
                compressed_ready_.wait_until(lock, target, [&] { return !running_.load(); });
                if (!running_.load()) break;
                compressed_ready_.wait(lock, [&] { return !running_.load() || compressed_.number > last; });
                if (!running_.load()) break;
                compressed = compressed_;
                decode_requested_ = false;
                compressed_read_ = compressed.number; // Own the immutable compressed buffer via its reference count.
            }
            last = compressed.number;
            const auto started = std::chrono::steady_clock::now();
            CameraFrame next;
            { AuditScope audit("capture.jpeg_decode"); next.image = cv::imdecode(compressed.jpeg, cv::IMREAD_COLOR); }
            if (next.image.empty() || next.image.cols != width_ || next.image.rows != height_)
                throw std::runtime_error("Invalid decoded camera frame");
            if (model_size_ > 0) next.prepared = prepareModelImage(next.image, model_size_);
            {
                const double cost = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started).count();
                std::lock_guard<std::mutex> lock(compressed_mutex_);
                decode_cost_ms_ = 0.75 * decode_cost_ms_ + 0.25 * cost;
            }
            next.timestamp_us = compressed.timestamp_us;
            next.epoch_ns = compressed.epoch_ns;
            next.frame_number = compressed.number;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (latest_.frame_number > delivered_frame_) dropped_frames_.fetch_add(1);
                latest_ = std::move(next); // Fresh allocation; no full-size producer clone.
            }
            ready_.notify_all();
            next_decode = started + std::chrono::microseconds(1000000 / decode_fps_);
        }
    } catch (const std::exception &e) {
        std::cerr << "[CAMERA] decode failure: " << e.what() << "\n";
    }
    running_.store(false);
    ready_.notify_all();
    compressed_ready_.notify_all();
}

} // namespace rescue
