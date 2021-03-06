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

  // from 0 to 3: Top-Right-Bottom-Left
  cv::Rect rect;

  // Are only here so we save 'em somewhere
  std::list<DartArea> greenContours{};
  std::list<DartArea> redContours{};

private:

  bool ready = false;

  // Basic filtering to get to 2x20 areas
  void checkNeighbour(DartArea& area1, DartArea& area2, int idxArea1, int idxArea2, int maxDistance);
  bool getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList);
  static void filterTheOddOneOut(std::list<DartArea*>& dartAreas);

  // Advanced filtering
  // Have to be called in chronological order
  bool getRedAndGreens(std::list<DartArea*>& dartAreasOuterRed, std::list<DartArea*>& dartAreasOuterGreen, 
    std::list<DartArea*>& dartAreasInnerRed, std::list<DartArea*>& dartAreasInnerGreen);
  static DartAreaArray sortAreas(DartArea* highestYArea);
  void setCorners();
  void setBullseye();
  void setSortedMeanCorners(DartAreaArray areas);
  void setSingles();
  void setNames();
  void setRect();

  static bool getTwoNeighbours(DartArea* area, std::list<DartArea>& compAreas);
  bool getTheOneTrueRing(DartArea* startArea,
                         std::list<DartArea*>& dartAreaCandidatesRed, std::list<DartArea*>& dartAreaCandidatesGreen);
};
