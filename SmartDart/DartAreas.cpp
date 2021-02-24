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

void DartArea::draw(cv::Mat& src, const cv::Scalar& color, bool drawPts, int radius, int thickness)
{
  if(drawPts)
  {
    circle(src, meanPoint, radius, color, thickness);
    for (const auto pt : significantPoints)
    {
      circle(src, pt, radius, color, thickness);
    }
  }

  //std::vector<std::vector<cv::Point> > contourVec;
  //contourVec.push_back(contour);
  //cv::drawContours(src, contourVec, 0, color, 3); //Replace i with 0 for index. 
}

//TODO: Maybe replace with dictionary, unsure yet..
// TODO Refactor drawing stuff of dartareas (bit chaotic and unlogical)
void DartArea::markAreas(cv::Mat& src, std::array<DartArea*, 20> dartAreas, int radius = 5, const cv::Scalar& color = cv::Scalar(0, 255, 0), int thickness = 5)
{
  int i = 0;
  for (auto area : dartAreas)
  {
    area->draw(src, color, true, radius, thickness);
  }
}

std::vector<std::vector<cv::Point>> DartArea::convertToContours(std::array<DartArea*, 20> dartAreas)
{
  std::vector<std::vector<cv::Point>> result;

  for(DartArea* dartArea : dartAreas)
  {
    result.push_back(dartArea->contour);
  }

  return result;
}

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
  if (area1.neighbour[!idxArea1] == &area2)
  {
    return;
  }

  // Firstly looks for near point, secondly looks that second point of found area is not too near
  if (getDistance(area1.significantPoints[idxArea1], area2.significantPoints[idxArea2]) < maxDistance
    && getDistance(area1.significantPoints[idxArea1], area2.significantPoints[!idxArea2]) > maxDistance)
  {
    area1.neighbour[idxArea1] = &area2;
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
  for (auto dartArea : doubles)
  {
    contoursBuff.push_back(dartArea->contour);
  }

  for (auto dartArea : triples)
  {
    contoursBuff.push_back(dartArea->contour);
  }

  if(!outerBullseye.contour.empty() && !innerBullseye.contour.empty())
  {
    contoursBuff.push_back(outerBullseye.contour);
    contoursBuff.push_back(innerBullseye.contour);
  }

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
    if (areaCmp.neighbour[0] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 0, 0, maxDistance0);
      checkNeighbour(areaCmp, areaIter, 0, 1, maxDistance0);
    }
    if (areaCmp.neighbour[1] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 1, 0, maxDistance1);
      checkNeighbour(areaCmp, areaIter, 1, 1, maxDistance1);
    }
    if (areaCmp.neighbour[0] != nullptr && areaCmp.neighbour[1] != nullptr)
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
    for (DartArea* areaPtr : area->neighbour)
    {
      if (areaPtr->neighbour[0] == nullptr || areaPtr->neighbour[1] == nullptr)
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

std::array<DartArea*, 20> sortAreas(DartArea* highestYArea)
{
  std::array<DartArea*, 20> sorted;

  // Hardcoded, as long as dartboard is ca. upright this will suffice
  // If needed, implement algorithm that sorts from highest to lowest.
  sorted[19] = highestYArea;

  // This algorithm will be applicable to all other dartareas with enum % 20
  auto* tmp = sorted[19]->neighbour;
  for (int i = 0; i < 19; i++) // <19, because we already got the ((last)) one
  {
    if (i <= 3)
    {
      sorted[i] = tmp[0]->meanPoint.x > tmp[1]->meanPoint.x ? tmp[0] : tmp[1];
    }
    else if (i <= 8)
    {
      // y points are ordered from top(0) to bottom(max)
      sorted[i] = tmp[0]->meanPoint.y > tmp[1]->meanPoint.y ? tmp[0] : tmp[1];
    }
    else if (i <= 13)
    {
      sorted[i] = tmp[0]->meanPoint.x < tmp[1]->meanPoint.x ? tmp[0] : tmp[1];
    }
    else
    {
      sorted[i] = tmp[0]->meanPoint.y < tmp[1]->meanPoint.y ? tmp[0] : tmp[1];
    }
    tmp = sorted[i]->neighbour;
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

void DartBoard::getCorners()
{
  for(int i = 0; i < 20; i++)
  {
    DartArea* tripleArea = triples[i];
    DartArea* doubleArea = doubles[i];

    cv::Point ptTmp1;
    cv::Point ptTmp2;

    const cv::Point centerPoint = (tripleArea->meanPoint + doubleArea->meanPoint) / 2;

    auto maxDistanceSig0 = getDistance(tripleArea->significantPoints[0], tripleArea->meanPoint);
    auto maxDistanceSig1 = getDistance(tripleArea->significantPoints[1], tripleArea->meanPoint);
    auto maxDist1 = 0;
    auto maxDist2 = 0;

    // Gets points that are furthest away from centerPoint on both sides (both have to be in a radius opposite to each other)
    for (const auto pt : tripleArea->contour)
    {
      auto distTmp = getDistance(pt, centerPoint);
      if (getDistance(pt, tripleArea->significantPoints[0]) < maxDistanceSig0 && maxDist1 < distTmp)
      {
        maxDist1 = distTmp;
        ptTmp1 = pt;
      }
      distTmp = getDistance(pt, centerPoint);
      if (getDistance(pt, tripleArea->significantPoints[1]) < maxDistanceSig1 && maxDist2 < distTmp)
      {
        maxDist2 = distTmp;
        ptTmp2 = pt;
      }
    }
    tripleArea->corners[DartArea::Outer1] = ptTmp1;
    tripleArea->corners[DartArea::Outer2] = ptTmp2;

    // Gets points which are furthest away from Outer1 and Outer2 of the triples
    maxDist1 = 0;
    maxDist2 = 0;
    for (const auto pt : doubleArea->contour)
    {
      auto distTmp = getDistance(pt, tripleArea->corners[DartArea::Outer1]);
      if (distTmp > maxDist1)
      {
        maxDist1 = distTmp;
        ptTmp1 = pt;
      }
      distTmp = getDistance(pt, tripleArea->corners[DartArea::Outer2]);
      if (distTmp > maxDist2)
      {
        maxDist2 = distTmp;
        ptTmp2 = pt;
      }
    }
    doubleArea->corners[DartArea::Outer1] = ptTmp1;
    doubleArea->corners[DartArea::Outer2] = ptTmp2;

    // Gets points which are furthest away from Outer1 and Outer2
    maxDist1 = 0;
    maxDist2 = 0;
    for (const auto pt : tripleArea->contour)
    {
      auto distTmp = getDistance(pt, tripleArea->corners[DartArea::Outer1]);
      if (distTmp > maxDist1)
      {
        maxDist1 = distTmp;
        ptTmp1 = pt;
      }
      distTmp = getDistance(pt, tripleArea->corners[DartArea::Outer2]);
      if (distTmp > maxDist2)
      {
        maxDist2 = distTmp;
        ptTmp2 = pt;
      }
    }
    // Reversed order because Inner1 and Outer1 should be on the same side, but the algorithm alway gets the diagonally furthest away
    tripleArea->corners[DartArea::Inner2] = ptTmp1;
    tripleArea->corners[DartArea::Inner1] = ptTmp2;

    // Gets points which are furthest away from the outerPoint (mean of both other corners)
    // & are further away than minDistance from corners
    // & are within a radius of the corners (so that 2 points are achieved, not just 2 next to each other)
    const int minDistance = (getDistance(tripleArea->corners[DartArea::Inner1], tripleArea->corners[DartArea::Outer1])
      + getDistance(tripleArea->corners[DartArea::Inner2], tripleArea->corners[DartArea::Outer2])) / 6;
    maxDistanceSig0 = getDistance(doubleArea->significantPoints[0], doubleArea->meanPoint);
    maxDistanceSig1 = getDistance(doubleArea->significantPoints[1], doubleArea->meanPoint);
    const cv::Point outerPoint = (doubleArea->corners[DartArea::Outer1] + doubleArea->corners[DartArea::Outer2]) / 2;
    maxDist1 = 0;
    maxDist2 = 0;
    for (const auto pt : doubleArea->contour)
    {
      const auto minDistTmp = getDistance(pt, doubleArea->corners[DartArea::Outer1]);
      const auto minDistTmp2 = getDistance(pt, doubleArea->corners[DartArea::Outer2]);
      if (minDistTmp > minDistance && minDistTmp2 > minDistance)
      {
        auto distTmp = getDistance(pt, outerPoint);
        if (getDistance(pt, doubleArea->significantPoints[0]) < maxDistanceSig0 && distTmp > maxDist1)
        {
          maxDist1 = distTmp;
          ptTmp1 = pt;
        }
        distTmp = getDistance(pt, outerPoint);
        if (getDistance(pt, doubleArea->significantPoints[1]) < maxDistanceSig1 && distTmp > maxDist2)
        {
          maxDist2 = distTmp;
          ptTmp2 = pt;
        }
      }
    }
    // Reversed order because Inner1 and Outer1 should be on the same side, but the algorithm alway gets the diagonally furthest away
    doubleArea->corners[DartArea::Inner2] = ptTmp1;
    doubleArea->corners[DartArea::Inner1] = ptTmp2;
  }
}

void DartBoard::getBullseye(std::list<DartArea> greenContours, std::list<DartArea> redContours)
{
  const auto centerX = (doubles[AREA_11]->meanPoint.x + doubles[AREA_6]->meanPoint.x) / 2;
  const auto centerY = (doubles[AREA_20]->meanPoint.y + doubles[AREA_3]->meanPoint.y) / 2;
  const cv::Point centerPoint(centerX, centerY);

  auto nearestGreenDistance = INT16_MAX;
  DartArea nearestGreen;
  for(const DartArea area : greenContours)
  {
    const auto distance = getDistance(centerPoint, area.meanPoint);
    if(distance < nearestGreenDistance)
    {
      nearestGreenDistance = distance;
      nearestGreen = area;
    }
  }
  outerBullseye = nearestGreen;

  auto nearestRedDistance = INT16_MAX;
  DartArea nearestRed;
  for (const DartArea area : redContours)
  {
    const auto distance = getDistance(centerPoint, area.meanPoint);
    if (nearestRedDistance > distance)
    {
      nearestRedDistance = distance;
      nearestRed = area;
    }
  }
  innerBullseye = nearestRed;

  // If triples are detected as bullseyes, a simple check could be build in here
}

DartBoard::DartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours, const cv::Mat& refImage)
: greenContours(greenContours), redContours(redContours)
{
  std::list<DartArea*> dartBoardRed;
  std::list<DartArea*> dartBoardGreen;

  for (DartArea& redArea : this->redContours)
  {
    const bool gotBothNeighbours = getNeighbours(redArea, this->greenContours);

    // Confirms that the red area has neighbours that are not the same
    if (gotBothNeighbours && redArea.neighbour[0] != redArea.neighbour[1])
    {
      dartBoardRed.push_back(&redArea);
    }
  }

  for(DartArea* redArea : dartBoardRed)
  {
    // Adds green contours to list if they're not already in it
    for (DartArea* greenNeighbour : redArea->neighbour)
    {
      if (std::find(dartBoardGreen.begin(), dartBoardGreen.end(), greenNeighbour) != dartBoardGreen.end())
      {
        greenNeighbour->neighbour[1] = redArea;
      }
      else
      {
        greenNeighbour->neighbour[0] = redArea;
        dartBoardGreen.push_back(greenNeighbour);
      }
    }
  }

  for (DartArea* greenArea : dartBoardGreen)
  {
    if(greenArea->neighbour[0] == nullptr || greenArea->neighbour[1] == nullptr)
    {
      std::cout << "Something went wrong\n";
      return;
    }
  }

  for (DartArea* redArea : dartBoardRed)
  {
    if (redArea->neighbour[0] == nullptr || redArea->neighbour[1] == nullptr)
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

  doubles = sortAreas(dartBoardRed.back());

  std::list<DartArea*> innerCandidates;
  for(DartArea* dartArea : dartBoardRed)
  {
    if (!(std::find(doubles.begin(), doubles.end(), dartArea) != doubles.end()))
    {
      innerCandidates.push_back(dartArea);
    }
  }

  triples = sortAreas(innerCandidates.back());

  // Ab hier
  cv::Mat drawing = refImage.clone(); /*cv::Mat::zeros(refImage.size(), CV_8UC3);*/
  //drawAreas(drawing, doubles, "D");
  //drawAreas(drawing, triples, "T");

  getBullseye(greenContours, redContours);

  getCorners();

  for(int i = 0; i < 20; i++)
  {
    for(cv::Point point : triples[i]->corners)
    {
      cv::circle(drawing, point, 2, cv::Scalar(255, 255, 255), 2);
    }
    for (cv::Point point : doubles[i]->corners)
    {
      cv::circle(drawing, point, 2, cv::Scalar(255, 255, 255), 2);
    }
  }

  // Version 1: Mean of points
  cv::Mat try1 = refImage.clone();
  std::list<cv::Point> betwPtsDoubleOuter;

  for(int i = 0; i < 10; i++)
  {
    DartArea* dartArea = doubles[i * 2];

    cv::Point corner1 = dartArea->corners[DartArea::Outer1];
    cv::Point corner2 = dartArea->corners[DartArea::Outer2];

    std::array<cv::Point, 4> neighbourPts = {
      dartArea->neighbour[0]->corners[DartArea::Outer1],
      dartArea->neighbour[0]->corners[DartArea::Outer2],
      dartArea->neighbour[1]->corners[DartArea::Outer1],
      dartArea->neighbour[1]->corners[DartArea::Outer2] };

    cv::Point neighbourPoint1;
    cv::Point neighbourPoint2;
    int minDistance1 = INT16_MAX;
    int minDistance2 = INT16_MAX;
    for(cv::Point pt : neighbourPts)
    {
      int dist1 = getDistance(corner1, pt);
      int dist2 = getDistance(corner2, pt);
      if(dist1 < minDistance1)
      {
        minDistance1 = dist1;
        neighbourPoint1 = pt;
      }
      if(dist2 < minDistance2)
      {
        minDistance2 = dist2;
        neighbourPoint2 = pt;
      }
    }

    betwPtsDoubleOuter.push_back(neighbourPoint1);
    betwPtsDoubleOuter.push_back(neighbourPoint2);

    cv::circle(try1, neighbourPoint1, 2, cv::Scalar(255, 255, 255), 2);
    cv::circle(try1, neighbourPoint2, 2, cv::Scalar(255, 255, 255), 2);
  }

  _win.imgshowResized("Test", drawing);
  _win.imgshowResized("Test2", try1);

  ready = true;
}

bool DartBoard::isReady() const
{
  return ready;
}
