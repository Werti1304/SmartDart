#include "GeneralTrackbar.h"

#include <thread>

GeneralTrackbar::GeneralTrackbar(std::string name, int values[3], std::string firstValName, std::string secondValName, std::string thirdValName, int width, int height) : ScalarTrackbar(name, &values[0], &values[1], &values[2], "H", "S", "V", 255, width, height)
{
  // Run onChange once to get the ol' engine up n' running
  GeneralTrackbar::onChangeInherit(0);
}

void GeneralTrackbar::setCallback(void(*customCallback)())
{
  this->customCallback = customCallback;
}

void GeneralTrackbar::onChangeInherit(int)
{
  //// https://stackoverflow.com/questions/31329437/trackbar-to-choose-color-using-c-opencv
  //image.setTo(cv::Scalar(*values[0], *values[1], *values[2]));
  //cvtColor(image, image, cv::COLOR_HSV2BGR); // BGR is standard
  //imshow(windowName, image);

  if (customCallback != nullptr)
  {
    std::thread t1(customCallback);
    t1.detach();
  }
}
