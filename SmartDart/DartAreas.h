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
  DartArea();
  void draw(cv::Mat& src, int radius, const cv::Scalar& color, int thickness);

  DartArea* connectAreas[2] = { nullptr };

  bool operator==(const DartArea& d1) const;
  bool operator!=(const DartArea& d1) const;

  static std::list<DartArea> calculateAreas(std::vector<std::vector<cv::Point>> contours);
  static void markAreas(::cv::Mat& src, std::array<DartArea, 20>, int radius, const cv::Scalar& color, int thickness);
  static std::vector<std::vector<cv::Point>> convertToContours(std::array<DartArea, 20> dartAreas);

private:
};

class DartAreaEx : public DartArea
{
public:
  enum CornerArrangement
  {
    TopLeft,
    TopRight,
    BottomRight,
    BottomLeft
  };

  DartAreaEx(DartArea, std::array<cv::Point, 4> corners);
  DartAreaEx();

  std::array<cv::Point, 4> corners;
};

class DartBoard
{
public:
  DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage);
  void drawBoardContours(cv::Mat& img, cv::Size sizeReference);
  bool isReady() const;

  std::array<DartArea, 20> singles; // TODO In se wörks
  std::array<DartArea, 20> doubles;
  std::array<DartArea, 20> tribles;
  DartArea innerBullseye; 
  DartArea outerBullseye; 
   
private:
  bool ready = false;
  void checkNeighbour(DartArea& area1, DartArea& area2, int idxArea1, int idxArea2, int maxDistance);

  bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
  static void filterTheOddOneOut(std::list<DartArea*>& dartBoardRed);
};