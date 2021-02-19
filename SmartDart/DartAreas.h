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

  DartArea(std::vector<cv::Point>);
  void markArea(cv::Mat& src, int radius, const cv::Scalar& color, int thickness);

  DartArea* connectAreas[2] = { nullptr };

  bool operator==(const DartArea& d1) const;

  static std::list<DartArea> calculateAreas(std::vector<std::vector<cv::Point>> contours);
  static std::list<DartArea> defineDartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours);
  static void markAreas(cv::Mat& src, std::list<DartArea> dartAreas, int radius, const cv::Scalar& color, int thickness);
  static std::vector<std::vector<cv::Point>> convertToContours(std::list<DartArea> dartAreas);

private:
  static bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
};
