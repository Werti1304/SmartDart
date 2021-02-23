#include "DartAreas.h"


#include <array>
#include <array>

#include "Resources.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

#include "Types.h"

void printPoint(cv::Point p)
{
  printf("(%d/%d)", p.x, p.y);
}

int getDistance(cv::Point pt1, cv::Point pt2)
{
  // Calculating distance 
  return sqrt(pow(pt2.x - pt1.x, 2) +
    pow(pt2.y - pt1.y, 2) * 1.0);
}

DartArea::DartArea(std::vector<cv::Point> inputPoints) : contour(inputPoints)
{
  const int ptCount = inputPoints.size();

  // Bestimmung des "Center of Mass" (Der Mittelpunkt)
  unsigned long pointXsum = 0;
  unsigned long pointYsum = 0;
  for(const auto pt : inputPoints)
  {
    pointXsum += pt.x;
    pointYsum += pt.y;
  }
  this->meanPoint.x = pointXsum / ptCount;
  this->meanPoint.y = pointYsum / ptCount;

  // Bestimmung des 1. signifikanten Punktes (Punkt der am weitesten von Mittelpunkt entfernt ist)
  int highestDistance = 0;
  for(const auto pt : inputPoints)
  {
    const int distance = getDistance(meanPoint, pt);

    if(distance > highestDistance)
    {
      highestDistance = distance;
      significantPoints[0] = pt;
    }
  }

  // Bestimmung des 2. signifikanten Punktes (Punkt der am weitesten von 1. signifikantem Punkt entfernt ist)
  highestDistance = 0;
  for (const auto pt : inputPoints)
  {
    const int distance = getDistance(pt, significantPoints[0]);

    if (distance > highestDistance)
    {
      highestDistance = distance;
      significantPoints[1] = pt;
    }
  }
}

DartArea::DartArea()
= default;

void DartArea::draw(cv::Mat& src, int radius = 5, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 5)
{
  circle(src, meanPoint, radius, color, thickness);
  for (const cv::Point pt : significantPoints)
  {
    circle(src, pt, radius, color, thickness);
  }
}

//TODO: Maybe replace with dictionary, unsure yet..

void DartArea::markAreas(cv::Mat& src, std::array<DartArea, 20> dartAreas, int radius = 5, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 5)
{
  int i = 0;
  for (DartArea area : dartAreas)
  {
    area.draw(src, radius, color, thickness);
  }
}

std::vector<std::vector<cv::Point>> DartArea::convertToContours(std::array<DartArea, 20> dartAreas)
{
  std::vector<std::vector<cv::Point>> result;

  for(DartArea dartArea : dartAreas)
  {
    result.push_back(dartArea.contour);
  }

  return result;
}

DartAreaEx::DartAreaEx(DartArea dartArea, std::array<cv::Point, 4> corners) : DartArea(dartArea), corners(corners)
{
}

DartAreaEx::DartAreaEx()
= default;

bool DartArea::operator==(const DartArea& d1) const
{
  return d1.contour == contour;
}

bool DartArea::operator!=(const DartArea& d1) const
{
  return !(*this == d1);
}

std::list<DartArea> DartArea::calculateAreas(std::vector<std::vector<cv::Point>> contours)
{
  std::list<DartArea> dartAreaList;

  for (const auto contour : contours)
  {
    dartAreaList.push_back(DartArea(contour));
  }
  return dartAreaList;
}

void DartBoard::checkNeighbour(DartArea& area1, DartArea& area2, const int idxArea1, const int idxArea2, const int maxDistance)
{
  if (area1.connectAreas[!idxArea1] == &area2)
  {
    return;
  }

  // Firstly looks for near point, secondly looks that second point of found area is not too near
  if (getDistance(area1.significantPoints[idxArea1], area2.significantPoints[idxArea2]) < maxDistance
    && getDistance(area1.significantPoints[idxArea1], area2.significantPoints[!idxArea2]) > maxDistance)
  {
    area1.connectAreas[idxArea1] = &area2;
    // area2 connecting removed, because green contours can become wrongly connected with this method
  }
}

void DartBoard::drawBoardContours(cv::Mat& img, cv::Size sizeReference)
{
  const cv::Scalar green = cv::Scalar(0, 255, 0);
  const cv::Scalar red = cv::Scalar(0, 0, 255);

  if(img.empty())
  {
    img = cv::Mat::zeros(sizeReference, CV_8UC3);
  }

  // Temporary contours safe
  std::vector<std::vector<cv::Point>> contoursBuff;
  for (DartArea dartArea : doubles)
  {
    contoursBuff.push_back(dartArea.contour);
  }

  for (DartArea dartArea : tribles)
  {
    contoursBuff.push_back(dartArea.contour);
  }

  contoursBuff.push_back(outerBullseye.contour);
  contoursBuff.push_back(innerBullseye.contour);

  for (size_t i = 0; i < contoursBuff.size(); i++)
  {
    drawContours(img, contoursBuff, static_cast<int>(i), i % 2 ? red : green, 2);
  }
}

bool DartBoard::getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList)
{
  const int maxDistance0 = getDistance(areaCmp.significantPoints[0], areaCmp.meanPoint);
  const int maxDistance1 = getDistance(areaCmp.significantPoints[1], areaCmp.meanPoint);

  for (DartArea& areaIter : areaList)
  {
    if (areaCmp.connectAreas[0] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 0, 0, maxDistance0);
      checkNeighbour(areaCmp, areaIter, 0, 1, maxDistance0);
    }
    if (areaCmp.connectAreas[1] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 1, 0, maxDistance1);
      checkNeighbour(areaCmp, areaIter, 1, 1, maxDistance1);
    }
    if (areaCmp.connectAreas[0] != nullptr && areaCmp.connectAreas[1] != nullptr)
    {
      // When both neighbours were found
      return true;
    }
  }
  return false;
}

void DartBoard::filterTheOddOneOut(std::list<DartArea*>& dartBoardRed)
{
  std::list<DartArea*> dartBoardTMp;

  for (DartArea* area : dartBoardRed)
  {
    bool add = true;
    for (DartArea* areaPtr : area->connectAreas)
    {
      if (areaPtr->connectAreas[0] == nullptr || areaPtr->connectAreas[1] == nullptr)
      {
        add = false;
      }
    }
    if (add)
    {
      dartBoardTMp.push_back(area);
    }
  }
  dartBoardRed = dartBoardTMp;
}

std::array<DartArea, 20> sortAreas(DartArea highestYArea)
{
  std::array<DartArea, 20> sorted;

  // Hardcoded, as long as dartboard is ca. upright this will suffice
  // If needed, implement algorithm that sorts from highest to lowest.
  sorted[19] = highestYArea;

  // This algorithm will be applicable to all other dartareas with enum % 20
  auto* tmp = sorted[19].connectAreas;
  for (int i = 0; i < 19; i++) // <19, because we already got the ((last)) one
  {
    if (i <= 3)
    {
      sorted[i] = tmp[0]->meanPoint.x > tmp[1]->meanPoint.x ? *tmp[0] : *tmp[1];
    }
    else if (i <= 8)
    {
      // y points are ordered from top(0) to bottom(max)
      sorted[i] = tmp[0]->meanPoint.y > tmp[1]->meanPoint.y ? *tmp[0] : *tmp[1];
    }
    else if (i <= 13)
    {
      sorted[i] = tmp[0]->meanPoint.x < tmp[1]->meanPoint.x ? *tmp[0] : *tmp[1];
    }
    else
    {
      sorted[i] = tmp[0]->meanPoint.y < tmp[1]->meanPoint.y ? *tmp[0] : *tmp[1];
    }
    //else
    //{
    //sorted[i] = sorted[19];
    //}
    tmp = sorted[i].connectAreas;
  }
  return sorted;
}

void drawAreas(cv::Mat& img, std::array<DartArea, 20> areas, const std::string prefix = "", int fontFace = 1,
               int fontScale = 2, const cv::Scalar color = cv::Scalar(255, 255, 255), int thickness = 2)
{
  for (int i = 0; i < 20; i++)
  {
    std::string text = prefix;
    text += std::to_string(DartAreaMapping.at(i + 1));
    //std::stringstream text;
    //text << i << " (" << areas[i].meanPoint.x << "/" << areas[i].meanPoint.y << ")";
    cv::putText(img, text, areas[i].meanPoint, fontFace, fontScale, color, thickness);
  }
}

std::array<DartAreaEx, 20> ascentAreas(std::array<DartArea, 20> sortedAreas)
{
  std::array<DartAreaEx, 20> sortedAreasEx;

  for(int i = 0; i < 20; i++)
  {
    // TODO Do... something here... not something stupid!.. but something..
  }
}

DartBoard::DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage)
{
  std::list<DartArea*> dartBoardRed;
  std::list<DartArea*> dartBoardGreen;

  for (DartArea& redArea : redContours)
  {
    const bool gotBothNeighbours = getNeighbours(redArea, greenContours);

    // Confirms that the red area has neighbours that are not the same
    if (gotBothNeighbours && redArea.connectAreas[0] != redArea.connectAreas[1])
    {
      dartBoardRed.push_back(&redArea);
    }
  }

  for(DartArea* redArea : dartBoardRed)
  {
    // Adds green contours to list if they're not already in it
    for (DartArea* greenNeighbour : redArea->connectAreas)
    {
      if (std::find(dartBoardGreen.begin(), dartBoardGreen.end(), greenNeighbour) != dartBoardGreen.end())
      {
        greenNeighbour->connectAreas[1] = redArea;
      }
      else
      {
        greenNeighbour->connectAreas[0] = redArea;
        dartBoardGreen.push_back(greenNeighbour);
      }
    }
  }

  for (DartArea* greenArea : dartBoardGreen)
  {
    if(greenArea->connectAreas[0] == nullptr || greenArea->connectAreas[1] == nullptr)
    {
      std::cout << "Something went wrong\n";
      return;
    }
  }

  if (dartBoardRed.size() > 20)
  {
    filterTheOddOneOut(dartBoardRed);

    if (dartBoardRed.size() != 20)
    {
      std::cout << "Couldn't properly generate red part of darboard, should have 20 but has " << dartBoardRed.size() << " areas!" << std::endl;
      return;
    }
  }
  if (dartBoardGreen.size() > 20)
  {
    filterTheOddOneOut(dartBoardGreen);

    if (dartBoardGreen.size() != 20)
    {
      std::cout << "Couldn't properly generate green part of darboard, should have 20 but has " << dartBoardGreen.size() << " areas!" << std::endl;
      return;
    }
  }
  if (dartBoardRed.size() < 20 || dartBoardGreen.size() < 20)
  {
    std::cout << "Couldn't properly generate darboard, should have 40 but has " << dartBoardRed.size() + dartBoardRed.size() << " areas!" << std::endl;
    return;
  }

  /*int i = 0;

  for(DartArea* area : dartBoardGreen)
  {
    tribles[i++] = *area;
  }

  i = 0;
  for (DartArea* area : dartBoardRed)
  {
    doubles[i++] = *area;
  }*/

  doubles = sortAreas(*dartBoardRed.back());

  std::list<DartArea*> innerCandidates;
  for(DartArea* dartArea : dartBoardRed)
  {
    if (!(std::find(doubles.begin(), doubles.end(), *dartArea) != doubles.end()))
    {
      innerCandidates.push_back(dartArea);
    }
  }

  tribles = sortAreas(*innerCandidates.back());

  cv::Mat drawing = cv::Mat::zeros(refImage.size(), CV_8UC3);
  drawAreas(drawing, doubles, "D");
  drawAreas(drawing, tribles, "T");

  //doubles[AREA_20].draw(drawing, 5, cv::Scalar(255, 0, 0));
  //doubles[AREA_6].draw(drawing, 5, cv::Scalar(0, 255, 0));
  //doubles[AREA_3].draw(drawing, 5, cv::Scalar(255, 0, 0));
  //doubles[AREA_11].draw(drawing, 5, cv::Scalar(0, 0, 255));

  const auto centerX = (doubles[AREA_11].meanPoint.x + doubles[AREA_6].meanPoint.x) / 2;
  const auto centerY = (doubles[AREA_20].meanPoint.y + doubles[AREA_3].meanPoint.y) / 2;
  cv::Point centerPoint = cv::Point(centerX, centerY);

  cv::circle(drawing, centerPoint, 5, cv::Scalar(255, 255, 255), 2);

  auto nearestGreenDistance = INT16_MAX;
  DartArea nearestGreen;
  for(DartArea area : greenContours)
  {
    auto distance = getDistance(centerPoint, area.meanPoint);
    if(nearestGreenDistance > distance)
    {
      nearestGreenDistance = distance;
      nearestGreen = area;
    }
  }
  outerBullseye = nearestGreen;

  auto nearestRedDistance = INT16_MAX;
  DartArea nearestRed;
  for (DartArea area : redContours)
  {
    auto distance = getDistance(centerPoint, area.meanPoint);
    if (nearestRedDistance > distance)
    {
      nearestRedDistance = distance;
      nearestRed = area;
    }
  }
  innerBullseye = nearestRed;

  _win.imgshowResized("Test", drawing);

  ready = true;
}

bool DartBoard::isReady() const
{
  return ready;
}
