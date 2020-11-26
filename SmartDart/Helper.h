#pragma once
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

class Helper
{
  static Mat readImageFromFile(std::string fileName, int flag);
};