#pragma once
#include "Types.h"
#include "WindowHelper.h"

#pragma region Color-Filter

namespace Resources
{
  const HSVColorFilter red_1_default(0, 110, 80,
    5, 255, 255);

  const HSVColorFilter red_2_default(170, 70, 80,
    180, 255, 255);

  const HSVColorFilter green_default(60, 100, 20,
    90, 255, 255);

  const HSVColorFilter red_1_histogram(0, 70, 80,
    5, 255, 255);

  const HSVColorFilter red_2_histogram(170, 70, 80,
    180, 255, 255);

  const HSVColorFilter green_histogram(50, 100, 20,
    90, 255, 130);

  const HSVColorFilter red_1_longexp(0, 70, 80,
    5, 255, 255);

  const HSVColorFilter red_2_longexp(170, 70, 80,
    180, 255, 255);

  const HSVColorFilter green_longexp(50, 100, 20,
    90, 255, 130);
};

#define RaspiWidth 1280
#define RaspiHeight 660
#define RaspiPath "/home/pi/Desktop/"

inline WindowHelper _win(RaspiPath, RaspiHeight, RaspiWidth);

const cv::Scalar _redColor{ 0, 0, 255 };
const cv::Scalar _greenColor{ 0, 255, 0 };
const cv::Scalar _blueColor{ 255, 0, 0 };
const cv::Scalar _whiteColor{ 255, 255, 255 };

inline std::map<int, int> DartAreaMapping
{
  {1, 1},
  {2, 18},
  {3, 4},
  {4, 13},
  {5, 6},
  {6, 10},
  {7, 15},
  {8, 2},
  {9, 17},
  {10, 3},
  {11, 19},
  {12, 7},
  {13, 16},
  {14, 8},
  {15, 11},
  {16, 14},
  {17, 9},
  {18, 12},
  {19, 5},
  {20, 20}
};

#pragma endregion 