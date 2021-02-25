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

class DartBoard
{
public:
  DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage);

  static void printText(cv::Mat& img, DartAreaArray areas, const std::string prefix = "", int fontFace = 1,
    int fontScale = 2, const cv::Scalar color = cv::Scalar(255, 255, 255), int thickness = 2);
  static void markAreas(::cv::Mat& src, DartAreaArray, int radius, const cv::Scalar& color, int thickness);

  void drawBoard(cv::Mat& img, cv::Size sizeReference);
  bool isReady() const;

  DartAreaArray doubles;
  DartAreaArray triples;
  DartArea innerBullseye; 
  DartArea outerBullseye;
  cv::Point outerBullseyeCenter;
  cv::Point innerBullseyeCenter;
  float outerBullseyeMeanRadius;
  float innerBullseyeMeanRadius;

  // Are only here so we save 'em somewhere
  std::list<DartArea> greenContours{};
  std::list<DartArea> redContours{};

private:
  bool ready = false;
  void checkNeighbour(DartArea& area1, DartArea& area2, int idxArea1, int idxArea2, int maxDistance);

  bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
  void getCorners();
  void getBullseye();
  void getSortedMeanCorners(DartAreaArray areas);
  static void filterTheOddOneOut(std::list<DartArea*>& dartBoardRed);
};
