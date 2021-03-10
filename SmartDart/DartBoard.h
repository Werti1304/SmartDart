#pragma once
#include "DartAreas.h"

class DartBoard
{
public:

  DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage);

  // Drawing Stuff
  static void printText(cv::Mat& img, DartAreaArray areas, const std::string prefix = "", int fontFace = 1,
    int fontScale = 2, const cv::Scalar color = cv::Scalar(255, 255, 255), int thickness = 2);
  void drawBoard(cv::Mat& img, cv::Size sizeReference);
  DartArea* detectHit(const cv::Point point);

  bool isReady() const;

  DartAreaArray singles;
  DartAreaArray doubles;
  DartAreaArray triples;

  DartArea bullseye;
  DartArea singleBull;
  cv::Point singleBullCenter;
  cv::Point bullseyeCenter;
  float singleBullMeanRadius;
  float bullseyeMeanRadius;

  cv::Point titlePoint;

  // Are only here so we save 'em somewhere
  std::list<DartArea> greenContours{};
  std::list<DartArea> redContours{};

private:
  bool ready = false;

  // Basic filtering to get to 2x20 areas
  void checkNeighbour(DartArea& area1, DartArea& area2, int idxArea1, int idxArea2, int maxDistance);
  bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
  static void filterTheOddOneOut(std::list<DartArea*>& dartBoardRed);

  // Advanced filtering
  // Have to be called in chronological order
  static DartAreaArray sortAreas(DartArea* highestYArea);
  void setCorners();
  void setBullseye();
  void setSortedMeanCorners(DartAreaArray areas);
  void setSingles();
  void setNames();
  void setTitlePoint();
};
