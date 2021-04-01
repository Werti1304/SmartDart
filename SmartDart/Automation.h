#pragma once
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include "Types.h"

using namespace cv;

namespace Automation
{
  void histogramEqualizationColored(const Mat& src, Mat& out);
  void erosion(const Mat& src, Mat& out);
  std::vector<std::vector<Point>> contours(const Mat& src, Mat& out, bool draw = true, int minPerimeter = 50);
  std::vector<std::vector<Point>> contoursSurround(const Mat& src, Mat& out, bool draw = true, int minPerimeter = 50);

  template<typename ...ColorFiltersT>
  void colorFilter(const Mat& src, Mat& outMask, Mat& outFiltered, ColorFiltersT... colorFiltersParam);
};

/**
 * \brief 
 * \tparam ColorFiltersT HSVColorFilter
 * \param src BGR source image
 * \param outMask bin/gray output image
 * \param outFiltered BGR output image
 * \param colorFiltersParam all color filters (are used incrementally)
 */
template <typename ... ColorFiltersT >
void Automation::colorFilter(const Mat& src, Mat& outMask, Mat& outFiltered, ColorFiltersT... colorFiltersParam)
{
  // Assert that all passed params are the right type
  //static_assert(std::is_same<param_pack<const HSVColorFilter>, param_pack<ColorFiltersT...>>::value, "Arguments must be HSVColorFilter.");
  HSVColorFilter colorFilters[]{ colorFiltersParam... };

  Mat srcHsv;
  cvtColor(src, srcHsv, COLOR_BGR2HSV); // Init inputImageHsv

  outMask = Mat::zeros(srcHsv.rows, srcHsv.cols, CV_8UC1);
  for (HSVColorFilter colorFilter : colorFilters)
  {
    Mat tmpMask;
    inRange(srcHsv, colorFilter.min_Color, colorFilter.max_Color, tmpMask);
    bitwise_or(outMask, tmpMask, outMask);
  }

  bitwise_and(src, src, outFiltered, outMask);
}