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

  DartArea(std::vector<cv::Point>);
  DartArea();
  void draw(cv::Mat& src, const cv::Scalar& color = cv::Scalar(0, 255, 0), bool drawPts = false, int radius = 5, int thickness = 5);
  bool isRed() const;

  DartArea* neighbour[2] = { nullptr };

  bool operator==(const DartArea& d1) const;
  bool operator!=(const DartArea& d1) const;

  static std::list<DartArea> calculateAreas(std::vector<std::vector<cv::Point>> contours);
  static void markAreas(::cv::Mat& src, std::array<DartArea*, 20>, int radius, const cv::Scalar& color, int thickness);
};

class DartBoard
{
public:
  DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage);
  void drawBoard(cv::Mat& img, cv::Size sizeReference);
  bool drawDartBoard(cv::Mat& img, int radius, int thickness);
  bool isReady() const;

  std::array<DartArea*, 20> singles; // TODO In se wörks
  std::array<DartArea*, 20> doubles;
  std::array<DartArea*, 20> triples;
  DartArea innerBullseye; 
  DartArea outerBullseye;
  cv::Point innerBullseyeCenter;

  const cv::Scalar redColor{ 0, 0, 255 };
  const cv::Scalar greenColor{ 0, 255, 0 };
   
private:
  bool ready = false;
  void checkNeighbour(DartArea& area1, DartArea& area2, int idxArea1, int idxArea2, int maxDistance);

  bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
  void getCorners();
  void getBullseye();
  static void filterTheOddOneOut(std::list<DartArea*>& dartBoardRed);

  // Are only here so we save 'em somewhere
  std::list<DartArea> greenContours{};
  std::list<DartArea> redContours{};
};

//cv::Point getMaxDistancePoint(std::vector<cv::Point> inputArr, cv::Point input)
//{
//  
//}
//
//template <size_t TArrSize>
//void getMaxDistancePoints(std::vector<cv::Point> inputArr, std::array<cv::Point, TArrSize> input, std::array<cv::Point, TArrSize>& output)
//{
//  
//}