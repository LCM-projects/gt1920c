#include "gt1920c.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

using namespace gt1920c;

int main() {
  std::string ip("192.168.80.100");
  CameraSettings setting;
  setting.height = 768;
  setting.width = 1024;
  setting.roi_x = (Gt1920c::kMaxWidth - setting.width) / 2;
  setting.roi_y = (Gt1920c::kMaxHeight - setting.height) / 2;
  setting.exposure = 20000;
  setting.gain = 10;

  Gt1920c driver(0, ip);
  driver.Start(setting);
  driver.BlockUntilFirstFrame();

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

  CameraFrame frame{};

  while (true) {
    if (driver.is_same_frame(frame.frame_id)) {
      usleep(1e4);
      continue;
    }

    frame = driver.GetImage();
    std::cout << "Frame: [" << frame.frame_id
              << "], time diff: " << (get_time() - frame.timestamp / 1e9)
              << "\n";

    cv::imshow("Display window", frame.image);
    cv::waitKey(1);
  }

  return 0;
}
