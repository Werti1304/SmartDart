#pragma once
#include <opencv2/core/types.hpp>

struct HSVColorFilter
{
  cv::Scalar min_Color;
  cv::Scalar max_Color;

  HSVColorFilter(const double h_min, const double s_min, const double v_min, const double h_max, const double s_max, const double v_max) :
    min_Color(h_min, s_min, v_min), max_Color(h_max, s_max, v_max)
  { }
};
