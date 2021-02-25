#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <list>

class DartArea
{
public:
  cv::Point meanPoint;
  cv::Point significantPoints[2];
  std::vector<cv::Point> contour;
  bool red;

  enum CornerArrangement
  {
    Outer1,
    Outer2,
    Inner1,
    Inner2
  };
  std::array<cv::Point, 4> corners;
  std::array<cv::Point, 4> meanCorners; // Only set for red!

  DartArea(std::vector<cv::Point>);
  DartArea();
  void draw(cv::Mat& src, const cv::Scalar& color = cv::Scalar(0, 255, 0), bool drawPts = false, int radius = 5, int thickness = 5);
  bool isRed() const;

  DartArea* neighbours[2] = { nullptr };

  bool operator==(const DartArea& d1) const;
  bool operator!=(const DartArea& d1) const;

  static std::list<DartArea> calculateAreas(std::vector<std::vector<cv::Point>> contours);
};

typedef std::array<DartArea*, 20> DartAreaArray;