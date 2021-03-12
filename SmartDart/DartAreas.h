#pragma once
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <list>

#include "DartAreaName.h"

class DartArea
{
public:
  enum CornerArrangement
  {
    Outer1,
    Outer2,
    Inner1,
    Inner2
  };

  // Get initialized in constructor
  /**
   * \brief Whether the area is defined by contours or self-defined points (only important for drawing)
   */
  bool drawAsContours;
  std::vector<cv::Point> contour;
  cv::Point meanPoint;
  cv::Point significantPoints[2];

  std::array<cv::Point, 4> corners;

  bool red;
  std::array<cv::Point, 4> meanCorners; // Only set for red!

  DartAreaName name;

  DartArea(std::vector<cv::Point> inputPoints, bool drawAsContours = true);
  DartArea();
  void draw(const cv::Mat& src, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 3) const;
  void drawUsingNameColor(cv::Mat& src, int thickness = 5) const;
  bool isRed() const;

  DartArea* neighbours[2] = { nullptr };

  bool operator==(const DartArea& d1) const;
  bool operator!=(const DartArea& d1) const;

  static std::list<DartArea> calculateAreas(std::vector<std::vector<cv::Point>> contours);
  static DartArea* getHighestYArea(std::list<DartArea*> dartAreas);
};

typedef std::array<DartArea*, 20> DartAreaArray;