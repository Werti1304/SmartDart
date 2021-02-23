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

enum DartAreas // S...simple, D...double, T...trible, OB...outer bullseye, IB...inner bullseye
{
  AREA_1,
  AREA_18,
  AREA_4,
  AREA_13,
  AREA_6,
  AREA_10,
  AREA_15,
  AREA_2,
  AREA_17,
  AREA_3,
  AREA_19,
  AREA_7,
  AREA_16,
  AREA_8,
  AREA_11,
  AREA_14,
  AREA_9,
  AREA_12,
  AREA_5,
  AREA_20
};