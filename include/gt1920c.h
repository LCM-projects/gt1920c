#pragma once

#include <PvApi.h>
#include <atomic>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace gt1920c {

struct CameraSettings {
  // ROI related on pg 38.
  uint32_t height;
  uint32_t width;
  uint32_t roi_x;
  uint32_t roi_y;
  // GainValue on pg 22.
  uint32_t gain;
  // ExposureValue on pg 20, in us.
  uint32_t exposure;
};

struct CameraFrame {
  // GBR color img.
  cv::Mat image;
  // In nano seconds, stamped by camera, if time synced with ptp. this should
  // match
  // ptp master.
  uint64_t timestamp;
  uint64_t frame_id;
};

inline double get_time() {
  struct timespec the_tp;
  clock_gettime(CLOCK_REALTIME, &the_tp);
  return ((double)(the_tp.tv_sec)) + 1.0e-9 * the_tp.tv_nsec;
}

class Gt1920c {
public:
  static constexpr uint64_t kMaxWidth = 1936;
  static constexpr uint64_t kMaxHeight = 1456;

  Gt1920c(const int id, const std::string &ip);
  ~Gt1920c() { Stop(); }

  int get_id() const { return id_; }
  uint64_t get_ip() const { return ip_; }
  void set_synced(bool sync) { ptp_synced_ = sync; }

  CameraFrame GetImage() const;

  void BlockUntilFirstFrame() const {
    while (is_same_frame(0)) {
      // sleep 100 ms.
      usleep(100000);
    }
  }

  void Start(const CameraSettings &settings) {
    quit_ = false;
    handler_thread_ = std::thread(&Gt1920c::HandlerThreadLoop, this, settings);
  }

  void Stop() {
    quit_ = true;
    handler_thread_.join();
  }

  bool is_same_frame(uint64_t frame_id) const {
    std::lock_guard<std::mutex> guard(mutex_);
    return frame_id == frame_.FrameCount;
  }

private:
  void Setup(const CameraSettings &setting);
  bool SetupEvent();
  void HandlerThreadLoop(const CameraSettings &settings);

  std::atomic<bool> quit_{false};

  const int id_{0};
  const uint64_t ip_{0};
  tPvHandle cam_handle_;
  //////////////////////////////////////
  // This is "shared" by HandlerThreadLoop and FrameCaptureCallback,
  // which I think shouldn't have race conditions.
  tPvFrame internal_frame_;
  // frame.ImageBuffer points to img_buf.data()
  std::vector<uint8_t> internal_img_buf_;
  //////////////////////////////////////

  //////////////////////////////////////
  // This is shared by HandlerThreadLoop and GetImage
  mutable std::mutex mutex_;
  tPvFrame frame_;
  std::vector<uint8_t> img_buf_;
  //////////////////////////////////////

  bool ptp_synced_{false};
  std::thread handler_thread_;

  // Just buffer.
  mutable std::vector<uint8_t> buffer_R_;
  mutable std::vector<uint8_t> buffer_G_;
  mutable std::vector<uint8_t> buffer_B_;
};

} // namespace gt1920c
