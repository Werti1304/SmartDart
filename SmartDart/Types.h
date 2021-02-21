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
  AREA_S1,
  AREA_S2,
  AREA_S3,
  AREA_S4,
  AREA_S5,
  AREA_S6,
  AREA_S7,
  AREA_S8,
  AREA_S9,
  AREA_S10,
  AREA_S11,
  AREA_S12,
  AREA_S13,
  AREA_S14,
  AREA_S15,
  AREA_S16,
  AREA_S17,
  AREA_S18,
  AREA_S19,
  AREA_S20,
  AREA_D1,
  AREA_D2,
  AREA_D3,
  AREA_D4,
  AREA_D5,
  AREA_D6,
  AREA_D7,
  AREA_D8,
  AREA_D9,
  AREA_D10,
  AREA_D11,
  AREA_D12,
  AREA_D13,
  AREA_D14,
  AREA_D15,
  AREA_D16,
  AREA_D17,
  AREA_D18,
  AREA_D19,
  AREA_D20,
  AREA_T1,
  AREA_T2,
  AREA_T3,
  AREA_T4,
  AREA_T5,
  AREA_T6,
  AREA_T7,
  AREA_T8,
  AREA_T9,
  AREA_T10,
  AREA_T11,
  AREA_T12,
  AREA_T13,
  AREA_T14,
  AREA_T15,
  AREA_T16,
  AREA_T17,
  AREA_T18,
  AREA_T19,
  AREA_T20,
  AREA_OB,
  AREA_IB,
  AREA_END
};