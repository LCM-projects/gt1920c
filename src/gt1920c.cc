#include "gt1920c.h"

#include <arpa/inet.h>
#include <iostream>

namespace gt1920c {

void CameraEventCallback(void *context, tPvHandle camera_handle,
                         const tPvCameraEvent *event_list,
                         uint64_t event_list_length);

void FrameCaptureCallback(tPvFrame *frame);

std::string get_error_msg(int cam_id, const std::string &func, tPvErr err) {
  return "Camera[" + std::to_string(cam_id) + "]: " + func + ", err: " +
         std::to_string(err);
}

std::string get_error_msg(int cam_id, const std::string &func) {
  return "Camera[" + std::to_string(cam_id) + "]: " + func;
}

Gt1920c::Gt1920c(const int id, const std::string &ip)
    : id_(id), ip_(inet_addr(ip.c_str())) {
  if ((ip_ == INADDR_NONE) || (ip_ == INADDR_ANY)) {
    throw std::logic_error(ip + " is not a valid IP address");
  }

  memset(&cam_handle_, 0, sizeof(tPvHandle));
  memset(&internal_frame_, 0, sizeof(tPvFrame));
  memset(&frame_, 0, sizeof(tPvFrame));
}

CameraFrame Gt1920c::GetImage() const {
  std::lock_guard<std::mutex> guard(mutex_);

  const size_t size = frame_.Height * frame_.Width;
  buffer_R_.resize(size);
  buffer_G_.resize(size);
  buffer_B_.resize(size);

  CameraFrame ret;
  ret.timestamp = (frame_.TimestampHi << 32) | (frame_.TimestampLo);
  ret.frame_id = frame_.FrameCount;
  ret.image = cv::Mat(frame_.Height, frame_.Width, CV_8UC3);

  // Interpolate raw image into RGB channels.
  PvUtilityColorInterpolate(&frame_, buffer_R_.data(), buffer_G_.data(),
                            buffer_B_.data(), 0, 0);

  for (size_t i = 0; i < frame_.Height; i++) {
    for (size_t j = 0; j < frame_.Width; j++) {
      size_t idx = i * frame_.Width + j;
      ret.image.at<cv::Vec3b>(i, j) =
          cv::Vec3b(buffer_B_.at(idx), buffer_G_.at(idx), buffer_R_.at(idx));
    }
  }

  return ret;
}

void Gt1920c::Setup(const CameraSettings &setting) {
  tPvErr err;
  uint64_t frame_size = 0;

  // Set image size and ROI.
  // uint32_t roi_x = (kMaxWidth - setting.width) / 2;
  // uint32_t roi_y = (kMaxHeight - setting.height) / 2;
  if (PvAttrUint32Set(cam_handle_, "Height", setting.height) != ePvErrSuccess ||
      PvAttrUint32Set(cam_handle_, "Width", setting.width) != ePvErrSuccess ||
      PvAttrUint32Set(cam_handle_, "RegionX", setting.roi_x) != ePvErrSuccess ||
      PvAttrUint32Set(cam_handle_, "RegionY", setting.roi_y) != ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "failed to set img size and ROI."));
  }

  // Set Exposure related.
  if (PvAttrEnumSet(cam_handle_, "ExposureMode", "Manual") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to set exposure mode."));
  }
  if ((err = PvAttrUint32Set(cam_handle_, "ExposureValue", setting.exposure)) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to set exposure value.", err));
  }

  // Set Gain related.
  if (PvAttrEnumSet(cam_handle_, "GainMode", "Manual") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to set gain mode."));
  }
  if ((err = PvAttrUint32Set(cam_handle_, "GainValue", setting.gain)) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to set gain value.", err));
  }

  // Set image format.
  if (PvAttrEnumSet(cam_handle_, "PixelFormat", "Bayer8") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to set pixel format."));
  }

  // Calculate frame buffer size
  if ((err = PvAttrUint32Get(cam_handle_, "TotalBytesPerFrame", &frame_size)) !=
      ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "failed to set pixel format.", err));
  }

  // Allocate the frame buffers
  img_buf_.resize(frame_size);
  frame_.ImageBuffer = img_buf_.data();
  frame_.ImageBufferSize = frame_size;

  internal_img_buf_.resize(frame_size);
  internal_frame_.ImageBuffer = internal_img_buf_.data();
  internal_frame_.ImageBufferSize = frame_size;

  // NOTE: This call sets camera PacketSize to largest sized test packet, up to
  // 8228, that doesn't fail
  // on network card. Some MS VISTA network card drivers become unresponsive if
  // test packet fails.
  // Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See
  // network card properties
  // for max allowable PacketSize/MTU/Jumboframe_size.
  if ((err = PvCaptureAdjustPacketSize(cam_handle_, 8228)) != ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "failed to adjust packet size", err));
  }

  if ((PvAttrUint32Set(cam_handle_, "StreamBytesPerSecond", 124000000) !=
       ePvErrSuccess)) {
    throw std::logic_error(
        get_error_msg(id_, "failed to set stream bytes per second", err));
  }

  // start driver capture stream
  if ((err = PvCaptureStart(cam_handle_)) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "failed to start capture", err));
  }

  // set the camera in free run, continuous mode
  if (PvAttrEnumSet(cam_handle_, "FrameStartTriggerMode", "Freerun") !=
      ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "FrameStartTriggerMode to Freerun failed"));
  }

  if (PvAttrEnumSet(cam_handle_, "AcquisitionMode", "Continuous") !=
      ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "AcquisitionMode to Continuous failed"));
  }

  // Set PTP Mode
  if (PvAttrEnumSet(cam_handle_, "PtpMode", "Slave") != ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "failed to set PTP mode to slave"));
  }

  // TODO
  // gain
  // exposure

  printf("CameraSetup done\n");
}

bool Gt1920c::SetupEvent() {
  tPvErr errCode;

  // check if events supported with this camera firmware
  if (PvAttrExists(cam_handle_, "EventsEnable1") == ePvErrNotFound) {
    printf("This camera does not support event notifications.\n");
    return false;
  }

  // Clear all events
  // EventsEnable1 is a bitmask of all events. Bits correspond to last two
  // digits of EventId.
  // e.g: Bit 1 is EventAcquisitionStart, Bit 2 is EventAcquisitionEnd, Bit 10
  // is EventSyncIn1Rise.
  if ((errCode = PvAttrUint32Set(cam_handle_, "EventsEnable1", 0)) !=
      ePvErrSuccess) {
    printf("Set EventsEnable1 err: %u\n", errCode);
    return false;
  }

  // Set events: AcquisitionStart, AcquisitionEnd, PtpSyncLost, PtpSyncLocked
  // In binary this is:  0001100011. In hex: 0x63
  unsigned int mask = 0;
  // AcquisitionStart,
  mask |= (1);
  // AcquisitionEnd,
  mask |= (1 << 1);
  // PtpSyncLost,
  mask |= (1 << 5);
  // PtpSyncLocked,
  mask |= (1 << 6);
  // EventExposureEnd,
  // mask |= (1 << 3);
  if ((errCode = PvAttrUint32Set(cam_handle_, "EventsEnable1", mask)) !=
      ePvErrSuccess) {
    printf("Set EventsEnable1 err: %u\n", errCode);
    return false;
  }

  // register callback function
  if ((errCode = PvCameraEventCallbackRegister(cam_handle_, CameraEventCallback,
                                               this)) != ePvErrSuccess) {
    printf("PvCameraEventCallbackRegister err: %u\n", errCode);
    return false;
  }
  printf("event setup done.\n");
  return true;
}

void Gt1920c::HandlerThreadLoop(const CameraSettings &settings) {
  std::vector<char> ip(128);
  std::vector<char> name(128);
  std::cout << "Starting camera thread for Cam[" << id_ << "]\n";

  // Initialize API.
  tPvErr err;
  if ((err = PvInitialize()) != ePvErrSuccess) {
    throw std::logic_error("PvInitialize failed " + std::to_string(err));
  }

  // Open camera
  if ((err = PvCameraOpenByAddr(ip_, ePvAccessMaster, &(cam_handle_))) !=
      ePvErrSuccess) {
    throw std::logic_error(
        get_error_msg(id_, "PvCameraOpenByAddr failed", err));
  }
  if ((PvAttrStringGet(cam_handle_, "DeviceIPAddress", ip.data(), ip.size(),
                       NULL) != ePvErrSuccess) ||
      (PvAttrStringGet(cam_handle_, "CameraName", name.data(), name.size(),
                       NULL) != ePvErrSuccess)) {
    throw std::logic_error(get_error_msg(id_, "cannot get camera info"));
  }

  printf("Cam[%u]: %s [%s] opened\n", id_, ip.data(), name.data());

  this->Setup(settings);

  this->SetupEvent();

  if (PvCommandRun(cam_handle_, "AcquisitionStart") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(id_, "AcquisitionStart failed"));
  }

  // Loop here.
  const uint64_t time_out = 1000; // milliseconds
  while (!quit_) {
    if (PvCaptureQueueFrame(cam_handle_, &(internal_frame_),
                            FrameCaptureCallback) != ePvErrSuccess) {
      throw std::logic_error(get_error_msg(0, "PvCaptureQueueFrame failed"));
    }

    if ((err = PvCaptureWaitForFrameDone(cam_handle_, &(internal_frame_),
                                         time_out)) != ePvErrSuccess) {
      throw std::logic_error(get_error_msg(id_, "CaptureFrame failed", err));
    }

    std::lock_guard<std::mutex> guard(mutex_);
    // Copy internal_frame_ to frame_
    img_buf_ = internal_img_buf_;
    frame_ = internal_frame_;
    frame_.ImageBuffer = img_buf_.data();
  }

  printf("Camera thread done.\n");
}

// Frame capture callback.
void FrameCaptureCallback(tPvFrame *frame) {
  if (frame->Status == ePvErrSuccess) {
    /*
    printf("Frame: %lu returned successfully\n", frame->FrameCount);
    uint64_t cam_time = (frame->TimestampHi << 32) | (frame->TimestampLo);
    uint64_t pc_time = get_time() * 1e9;
    double diff = ((double)pc_time - (double)cam_time) * 1e-9;
    printf("Frame: [%lu], cam time [%lu], pc time [%lu] diff [%g]\n",
           frame->FrameCount, cam_time, pc_time, diff);
    */
  } else if (frame->Status == ePvErrDataMissing)
    // Possible improper network card settings. See GigE Installation Guide.
    printf("Frame: %lu dropped packets\n", frame->FrameCount);
  else if (frame->Status == ePvErrCancelled)
    printf("Frame cancelled %lu\n", frame->FrameCount);
  else
    printf("Frame: %lu Error: %u\n", frame->FrameCount, frame->Status);
}

// Event callback. This is called by PvApi when camera event(s) occur.
void CameraEventCallback(void *context, tPvHandle camera_handle,
                         const tPvCameraEvent *event_list,
                         uint64_t event_list_length) {
  Gt1920c *camera = static_cast<Gt1920c *>(context);

  // multiple events may have occurred for this one callback
  for (uint64_t i = 0; i < event_list_length; i++) {
    switch (event_list[i].EventId) {
    case 40000:
      printf("Cam[%u]: ***EventAcquisitionStart\n", camera->get_id());
      break;
    case 40001:
      printf("Cam[%u]: ***EventAcquisitionEnd\n", camera->get_id());
      break;
    case 40003:
      printf("Cam[%u]: ***ExposureEndTrigger\n", camera->get_id());
      break;
    case 40005:
      camera->set_synced(false);
      printf("Cam[%u]: ***EventPtpSyncLost\n", camera->get_id());
      break;
    case 40006:
      camera->set_synced(true);
      printf("Cam[%u]: ***EventPtpSyncLocked\n", camera->get_id());
      break;
    case 65534:
      printf("Cam[%u]: ***EventOverflow error\n", camera->get_id());
      break;
    default:
      printf("Cam[%u]: ***Event %lu\n", camera->get_id(),
             event_list[i].EventId);
      break;
    }
  }
}

} // namespace gt1920c
