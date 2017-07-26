#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <stdexcept>

#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <sys/times.h>
#include <unistd.h>

#include <PvApi.h>
// #include <ImageLib.h>

// number of frames to be used
#define FRAMESCOUNT 1

// camera data
typedef struct {
  int ID;
  unsigned long IP;
  tPvHandle Handle;
  tPvFrame Frames[FRAMESCOUNT];
  bool PTPSynced; // Are we in sync?
#ifdef _WINDOWS
  HANDLE ThHandle;
  DWORD ThId;
#else
  pthread_t ThHandle;
#endif

} tCamera;

// session data
typedef struct {
  int Count;
  tCamera *Cameras;
  bool Abort;

} tSession;

///////////////

// global data
tSession GSession;

void SetConsoleCtrlHandler(void (*func)(int), int junk) {
  signal(SIGINT, func);
}

double get_time() {
  struct timespec the_tp;
  clock_gettime( CLOCK_REALTIME, &the_tp );
  return ((double) (the_tp.tv_sec)) + 1.0e-9*the_tp.tv_nsec;
}

std::string get_error_msg(int cam_id, const std::string& func, tPvErr err) {
  return "Camera[" + std::to_string(cam_id) + "]: " + func + ", err: " + std::to_string(err);
}

std::string get_error_msg(int cam_id, const std::string& func) {
  return "Camera[" + std::to_string(cam_id) + "]: " + func;
}

// Frame capture callback. This is queued when an exposure end event has happened.
void F_FrameCaptureCallback(tPvFrame* Frame) {
  if (Frame->Status == ePvErrSuccess) {
		printf("Frame: %lu returned successfully\n", Frame->FrameCount);
    unsigned long cam_time =
        (Frame->TimestampHi << 32) | (Frame->TimestampLo);
    unsigned long pc_time = get_time() * 1e9;
    double diff = ((double)pc_time - (double)cam_time) * 1e-9;
    printf("Frame: [%lu], cam time [%lu], pc time [%lu] diff [%g]\n",
        Frame->FrameCount, cam_time, pc_time, diff);
  }
	else if (Frame->Status == ePvErrDataMissing)
		//Possible improper network card settings. See GigE Installation Guide.
		printf("Frame: %lu dropped packets\n", Frame->FrameCount);
	else if (Frame->Status == ePvErrCancelled)
		printf("Frame cancelled %lu\n", Frame->FrameCount);
	else
		printf("Frame: %lu Error: %u\n", Frame->FrameCount, Frame->Status);

  /*
  if(Frame->Status != ePvErrCancelled) {
    if (PvCaptureQueueFrame(GSession.Cameras[0].Handle, Frame, F_FrameCaptureCallback) != ePvErrSuccess) {
      throw std::logic_error(get_error_msg(0, "PvCaptureQueueFrame failed"));
    }
  }
  */
}

// Event callback.  This is called by PvApi when camera event(s) occur.
void F_CameraEventCallback(void *pContext, tPvHandle CamHandle,
    const tPvCameraEvent *EventList,
    unsigned long EventListLength) {
  tCamera *Camera = (tCamera *)pContext;

  // multiple events may have occurred for this one callback
  for (unsigned long i = 0; i < EventListLength; i++) {
    switch (EventList[i].EventId) {
    case 40000:
      printf("Cam[%u]: ***EventAcquisitionStart\n", Camera->ID);
      break;
    case 40001:
      printf("Cam[%u]: ***EventAcquisitionEnd\n", Camera->ID);
      break;
    case 40003:
      printf("Cam[%u]: ***ExposureEndTrigger\n", Camera->ID);
      break;
    case 40005:
      Camera->PTPSynced = false;
      printf("Cam[%u]: ***EventPtpSyncLost\n", Camera->ID);
      break;
    case 40006:
      Camera->PTPSynced = true;
      printf("Cam[%u]: ***EventPtpSyncLocked\n", Camera->ID);
      break;
    case 65534:
      printf("Cam[%u]: ***EventOverflow error\n", Camera->ID);
      break;
    default:
      printf("Cam[%u]: ***Event %lu\n", Camera->ID, EventList[i].EventId);
      break;
    }
  }
}

// allocate memory
void CameraSetup(tCamera &Camera) {
  tPvErr err;
  unsigned long FrameSize = 0;

  // Set image size and ROI.
  uint32_t width = 640; // 1024;
  uint32_t height = 480; //768;
  uint32_t max_width = 1936;
  uint32_t max_height = 1456;
  uint32_t roi_x = (max_width - width) / 2;
  uint32_t roi_y = (max_height - height) / 2;
  if (PvAttrUint32Set(Camera.Handle, "Height", height) != ePvErrSuccess ||
      PvAttrUint32Set(Camera.Handle, "Width", width) != ePvErrSuccess ||
      PvAttrUint32Set(Camera.Handle, "RegionX", roi_x) != ePvErrSuccess ||
      PvAttrUint32Set(Camera.Handle, "RegionY", roi_y) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to set img size and ROI."));
  }

  if (PvAttrEnumSet(Camera.Handle, "PixelFormat", "Bayer8") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to set pixel format."));
  }

  // Calculate frame buffer size
  if ((err = PvAttrUint32Get(Camera.Handle, "TotalBytesPerFrame",
                                 &FrameSize)) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to set pixel format.", err));
  }

  // allocate the frame buffers
  for (int i = 0; i < FRAMESCOUNT; i++) {
    Camera.Frames[i].ImageBuffer = new char[FrameSize];
    Camera.Frames[i].ImageBufferSize = FrameSize;
  }

  // NOTE: This call sets camera PacketSize to largest sized test packet, up to
  // 8228, that doesn't fail
  // on network card. Some MS VISTA network card drivers become unresponsive if
  // test packet fails.
  // Use PvUint32Set(handle, "PacketSize", MaxAllowablePacketSize) instead. See
  // network card properties
  // for max allowable PacketSize/MTU/JumboFrameSize.
  if ((err = PvCaptureAdjustPacketSize(Camera.Handle, 8228)) !=
      ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to adjust packet size", err));
  }

  if ((PvAttrUint32Set(Camera.Handle, "StreamBytesPerSecond",
                       124000000) != ePvErrSuccess)) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to set stream bytes per second", err));
  }

  // start driver capture stream
  if ((err = PvCaptureStart(Camera.Handle)) != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to start capture", err));
  }

  // queue frames.
  for (int i = 0; i < FRAMESCOUNT; i++) {
    if ((err = PvCaptureQueueFrame(Camera.Handle, &(Camera.Frames[i]),
                                   F_FrameCaptureCallback)) != ePvErrSuccess) {
      throw std::logic_error(get_error_msg(Camera.ID, "PvCaptureQueueFrame failed", err));
    }
  }

  // set the camera in free run, continuous mode
  if (PvAttrEnumSet(Camera.Handle, "FrameStartTriggerMode", "Freerun") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "FrameStartTriggerMode to Freerun failed"));
  }

  if (PvAttrEnumSet(Camera.Handle, "AcquisitionMode", "Continuous") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "AcquisitionMode to Continuous failed"));
  }

  // Set PTP Mode
  if (PvAttrEnumSet(Camera.Handle, "PtpMode", "Slave") != ePvErrSuccess) {
    throw std::logic_error(get_error_msg(Camera.ID, "failed to set PTP mode to slave"));
  }

  // TODO
  // gain
  // exposure

  printf("CameraSetup done\n");
}

// setup event channel
// return value: true == success, false == fail
bool EventSetup(tCamera &Camera) {
  tPvErr errCode;

  // check if events supported with this camera firmware
  if (PvAttrExists(Camera.Handle, "EventsEnable1") == ePvErrNotFound) {
    printf("This camera does not support event notifications.\n");
    return false;
  }

  // Clear all events
  // EventsEnable1 is a bitmask of all events. Bits correspond to last two
  // digits of EventId.
  // e.g: Bit 1 is EventAcquisitionStart, Bit 2 is EventAcquisitionEnd, Bit 10
  // is EventSyncIn1Rise.
  if ((errCode = PvAttrUint32Set(Camera.Handle, "EventsEnable1", 0)) !=
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
  mask |= (1 << 3);
  if ((errCode = PvAttrUint32Set(Camera.Handle, "EventsEnable1", mask)) !=
      ePvErrSuccess) {
    printf("Set EventsEnable1 err: %u\n", errCode);
    return false;
  }

  // register callback function
  if ((errCode = PvCameraEventCallbackRegister(
           Camera.Handle, F_CameraEventCallback, &Camera)) != ePvErrSuccess) {
    printf("PvCameraEventCallbackRegister err: %u\n", errCode);
    return false;
  }
  printf("event setup done.\n");
  return true;
}

void proc_img(const tPvFrame& frame) {
  printf("w: %lu, h: %lu, format %d, img size %lu\n", frame.Width, frame.Height, frame.Format, frame.ImageSize);
}

// main camera thread
void *ThreadFunc(void *pContext) {
  tCamera *Camera = (tCamera *)pContext;
  tPvErr err;

  char IP[128];
  char Name[128];

  if ((err = PvCameraOpenByAddr(Camera->IP, ePvAccessMaster,
                                &(Camera->Handle))) == ePvErrSuccess) {

    if (((err = PvAttrStringGet(Camera->Handle, "DeviceIPAddress", IP, 128,
              NULL)) == ePvErrSuccess) &&
        ((err = PvAttrStringGet(Camera->Handle, "CameraName", Name, 128,
                                    NULL)) == ePvErrSuccess)) {
      printf("Cam[%u]: %s [%s] opened\n", Camera->ID, IP, Name);
      CameraSetup(*Camera);
      EventSetup(*Camera);
      if (PvCommandRun(Camera->Handle, "AcquisitionStart") != ePvErrSuccess) {
        throw std::logic_error(get_error_msg(Camera->ID, "AcquisitionStart failed"));
      }

      while(true) {
        if ((err = PvCaptureWaitForFrameDone(Camera->Handle, &(Camera->Frames[0]), 1000)) != ePvErrSuccess) {
          throw std::logic_error(get_error_msg(Camera->ID, "CaptureFrame failed", err));
        }

        proc_img(Camera->Frames[0]);




        // ImageWriteTiff("stuff.tiff", &(Camera->Frames[0]));
        // exit(-1);

        if(Camera->Frames[0].Status != ePvErrCancelled) {
          if (PvCaptureQueueFrame(Camera->Handle, &(Camera->Frames[0]), F_FrameCaptureCallback) != ePvErrSuccess) {
            throw std::logic_error(get_error_msg(0, "PvCaptureQueueFrame failed"));
          }
        }
      }
    }
  } else {
    printf("Cam[%u]: PvCameraOpenByAddr err: %u\n", Camera->ID, err);
  }
  printf("Camera thread done.\n");
  return NULL;
}

bool init(const std::string& ip_addr) {
  // initialize the PvAPI
  if (PvInitialize() != ePvErrSuccess) {
    throw std::logic_error("Cannot init API");
  }

  // IMPORTANT: Initialize camera structure. See tPvFrame in PvApi.h for more
  // info.
  memset(&GSession, 0, sizeof(tSession));

  GSession.Count = 1;
  GSession.Cameras = new tCamera[GSession.Count];
  memset(GSession.Cameras, 0, sizeof(tCamera) * GSession.Count);

  GSession.Cameras[0].IP = inet_addr(ip_addr.c_str());
  GSession.Cameras[0].ID = 0;
  if ((GSession.Cameras[0].IP == INADDR_NONE) ||
      (GSession.Cameras[0].IP == INADDR_ANY)) {
    GSession.Abort = true;
    throw std::logic_error("A valid IP address must be entered\n");
  }

  printf("%d cameras to be opened\n", GSession.Count);

  // Spawn a thread for each camera
  for (int i = 0; i < GSession.Count; i++) {
    if (GSession.Cameras[i].IP) {
      pthread_create(&GSession.Cameras[i].ThHandle, NULL, ThreadFunc,
          &GSession.Cameras[i]);
    }
  }

  return true;
}

int main() {
  init("192.168.80.100");
  while(1);
}
