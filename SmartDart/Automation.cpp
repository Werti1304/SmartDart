#include "Automation.h"

#include <iostream>

/**
 * \brief Generates histogram based on Y channel
 * \param src BGR source image
 * \param out BGR output image
 */
void Automation::histogramEqualizationColored(const Mat& src, Mat& out)
{
  Mat ycrcb;

  cvtColor(src, ycrcb, COLOR_BGR2YCrCb);

  std::vector<Mat> channels;
  split(ycrcb, channels);

  equalizeHist(channels[0], channels[0]);

  merge(channels, ycrcb);

  cvtColor(ycrcb, out, COLOR_YCrCb2BGR);
}

/**
 * \brief Replace Canny-Edge Detection by eroding and subtracting the image
 * \param src gray/bin/BGR source image
 * \param out bin output image
 */
void Automation::erosion(const Mat& src, Mat& out)
{
  Mat src_gray;
  if (src.channels() > 1)
  {
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
  }
  else
  {
    src_gray = src;
  }

  Mat eroded_image;

  blur(src_gray, src_gray, Size(3, 3));

  Canny(src_gray, out, 50, 50 * 2);

  // TODO Maybe remove and replace in contours with approxPolyDP and replace blur with Gaussianblur here
  // Connects areas, is important for hit detection
  const Mat kernel = getStructuringElement(MORPH_ELLIPSE, {5, 5});
  dilate(out, out, kernel);

  //erode(src_gray, eroded_image, Mat());

  //out = src_gray - eroded_image;

  //threshold(out, out, 1, 255, THRESH_BINARY);
}

/**
 * \brief finds contours and filters them
 * \param src bgr/gray/bin source image
 * \param out colored output image
 * \param draw draw the contours
 * \param minPerimeter hard-coded minimum perimeter size to rule out trailing pixels
 * \return list of contours which are a list of points
 */
std::vector<std::vector<Point>> Automation::contours(const Mat& src, Mat& out, bool draw, int minPerimeter)
{
  std::vector<std::vector<Point>> contours;
  std::vector<Vec4i> hierarchy; // Not needed, because of retrieve-mode (RETR_EXTERNAL)

  Mat src_gray;
  if (src.channels() > 1)
  {
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
  }
  else
  {
    src_gray = src;
  }

  findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  RNG rng(time(nullptr)); // RNG with seed of current time

  out = Mat::zeros(src.size(), CV_8UC3);
  std::vector<std::vector<Point>> contoursFiltered;
  for (size_t i = 0; i < contours.size(); i++)
  {
    auto cContour = contours.at(i);
    if (arcLength(cContour, true) > minPerimeter)
    {
      contoursFiltered.push_back(cContour);

      Scalar color = Scalar(rng.uniform(50, 256), rng.uniform(0, 256), rng.uniform(0, 256));

      if (draw)
      {
        drawContours(out, contours, static_cast<int>(i), color, 2, LINE_8, hierarchy, 0);
      }
    }
  }
  return contoursFiltered;
}

