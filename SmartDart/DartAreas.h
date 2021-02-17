#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <list>

class DartAreas
{
  cv::Mat src;
  std::vector<std::vector<cv::Point>> contours;

public:
  class DartArea
  {
  public:
    enum Index
    {
      TopMost,
      RightMost,
      BottomMost,
      LeftMost
    };

    cv::Point corners[4];

    DartArea(std::vector<cv::Point>);
    cv::Mat markArea(int radius, const cv::Scalar& color, int thickness);
  };

private:
  std::list<DartArea> dartAreaList;

  void calculateAreas();

public:
  DartAreas(cv::Mat& src, std::vector<std::vector<cv::Point>> contours);

  cv::Mat markAreas(int radius, const cv::Scalar& color, int thickness);
};
