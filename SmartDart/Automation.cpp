#include "Automation.h"

#include <iostream>

#include "Resources.h"

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

  //blur(src_gray, src_gray, Size(3, 3));
  GaussianBlur(src_gray, src_gray, Size(5, 5), 0);

  //Canny(src_gray, out, 40, 40 * 2);

  erode(src_gray, eroded_image, Mat());

  out = src_gray - eroded_image;

  threshold(out, out, 1, 255, THRESH_BINARY);

  //Canny(src_gray, out, 40, 40 * 2);

  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(src_gray, src_gray, cv::MORPH_CLOSE, structuringElement);

  // TODO Maybe remove and replace in contours with approxPolyDP and replace blur with Gaussianblur here
  // Connects areas, is important for hit detection
  //const Mat kernel = getStructuringElement(MORPH_ELLIPSE, {5, 5});
  //dilate(out, out, kernel);



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

std::vector<std::vector<Point>> Automation::contoursSurround(const Mat& src, Mat& out, bool draw, int minPerimeter)
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
  findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  int biggestArea = 0;
  int biggestAreaIdx = -1;
  int secondBiggestArea = 0;
  int secondBiggestAreaIdx = -1;
  long areaSum = 0;

  if(contours.size() == 0)
  {
    std::cout << "No <red> contours found!";
    return contours;
  }
  Mat tmpImg = Mat::zeros(src.size(), CV_8UC1);
  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
  cv::morphologyEx(tmpImg, tmpImg, cv::MORPH_CLOSE, structuringElement);

  for(auto i = 0; i < contours.size(); i++)
  {
    const int area = contourArea(contours[i], true);
    if(area > 10)
    {
      areaSum += area;
      if (area > biggestArea)
      {
        secondBiggestAreaIdx = biggestAreaIdx;
        secondBiggestArea = biggestArea;
        biggestArea = area;
        biggestAreaIdx = i;
      }
      else if(area > secondBiggestArea)
      {
        secondBiggestAreaIdx = i;
        secondBiggestAreaIdx = i;
        secondBiggestArea = area;
      }
    }
  }

  if(biggestArea + secondBiggestArea < (areaSum - biggestArea - secondBiggestArea))
  {
    return Automation::contours(src, out, draw, minPerimeter);
  }
  std::cout << "Surround found, filtering out..\n";

  drawContours(tmpImg, contours, secondBiggestAreaIdx, _whiteColor, -1);

  threshold(tmpImg, tmpImg, 1, 255, THRESH_BINARY_INV);
  // Morph and Gaussian to close the holes
  GaussianBlur(tmpImg, tmpImg, { 15, 15 }, 2, 2);
  structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
  cv::morphologyEx(tmpImg, tmpImg, cv::MORPH_DILATE, structuringElement);

  Mat modifiedSrc;
  bitwise_not(tmpImg, tmpImg);
  bitwise_and(src, src, modifiedSrc, tmpImg);
  _win.imgshowResized("Filtered erosion", modifiedSrc);

  return Automation::contours(modifiedSrc, out, draw, minPerimeter );
}

