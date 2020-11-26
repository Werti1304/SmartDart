#include "HSVTrackbar.h"
#include <thread>

HSVTrackbar::HSVTrackbar(std::string name, int values[3], int width, int height) : ScalarTrackbar( name, &values[0], &values[1], &values[2], "H", "S", "V", 255, width, height  )
{
  // Run onChange once to get the ol' engine up n' running
  HSVTrackbar::onChangeInherit(0);
}

void HSVTrackbar::setCallback( void(* customCallback)() )
{
  this->customCallback = customCallback;
}

void HSVTrackbar::onChangeInherit( int )
{
  // https://stackoverflow.com/questions/31329437/trackbar-to-choose-color-using-c-opencv
  image.setTo(cv::Scalar(*values[0], *values[1], *values[2]));
  cvtColor(image, image, cv::COLOR_HSV2BGR); // BGR is standard
  imshow(windowName, image);

  if(customCallback != nullptr)
  {
    std::thread t1(customCallback);
    t1.detach();
  }
}
