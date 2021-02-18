#include "DartAreas.h"


#include <iostream>
#include <opencv2/imgproc.hpp>

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

void DartArea::markArea(cv::Mat& src, int radius, const cv::Scalar& color, int thickness)
{
  circle(src, meanPoint, radius, color, thickness);
  for (const cv::Point pt : significantPoints)
  {
    circle(src, pt, radius, color, thickness);
  }
}

//TODO: Maybe replace with dictionary, unsure yet..
void checkNeighbour(DartArea& area1, DartArea& area2, const int idxArea1, const int idxArea2, const int maxDistance)
{
  // Firstly looks for near point, secondly looks that second point of found area is not too near
  if (getDistance(area1.significantPoints[idxArea1], area2.significantPoints[idxArea2]) < maxDistance 
    && getDistance(area1.significantPoints[idxArea1], area2.significantPoints[!idxArea2]) > maxDistance)
  {
    area1.connectAreas[idxArea1] = &area2;
    area2.connectAreas[idxArea2] = &area1;
  }
}

bool DartArea::getNeighbours(DartArea& areaCmp, std::list<DartArea>& areaList)
{
  const int maxDistance0 = getDistance(areaCmp.significantPoints[0], areaCmp.meanPoint);
  const int maxDistance1 = getDistance(areaCmp.significantPoints[1], areaCmp.meanPoint);

  for (DartArea& areaIter : areaList)
  {
    if(areaCmp.connectAreas[0] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 0, 0, maxDistance0);
      checkNeighbour(areaCmp, areaIter, 0, 1, maxDistance0);
    }
    if(areaCmp.connectAreas[1] == nullptr)
    {
      checkNeighbour(areaCmp, areaIter, 1, 0, maxDistance1);
      checkNeighbour(areaCmp, areaIter, 1, 1, maxDistance1);
    }
    if(areaCmp.connectAreas[0] != nullptr && areaCmp.connectAreas[1] != nullptr)
    {
      // When both neighbours were found
      return true;
    }
  }
  return false;
}

std::list<DartArea> DartArea::defineDartBoard(std::list<DartArea> greenContours, std::list<DartArea> redContours)
{
  std::list<DartArea> dartBoard;

  for (DartArea& redArea : redContours)
  {
    const bool gotBothNeighbours = getNeighbours(redArea, greenContours);

    // Confirms that the red area has neighbours that are not the same
    if (gotBothNeighbours && redArea.connectAreas[0] != redArea.connectAreas[1])
    {
      dartBoard.push_back(redArea);
    }
  }

  for (DartArea greenArea : greenContours)
  {
    // Confirms that the green area has neighbours that are not the same
    if (greenArea.connectAreas[0] && greenArea.connectAreas[1] && greenArea.connectAreas[0] != greenArea.connectAreas[1])
    {
      dartBoard.push_back(greenArea);
    }
  }

  return dartBoard;
}

void DartArea::markAreas(cv::Mat& src, std::list<DartArea> dartAreas, int radius, const cv::Scalar& color, int thickness)
{
  int i = 0;
  for (DartArea area : dartAreas)
  {
    area.markArea(src, radius, color, thickness);
  }
}

std::vector<std::vector<cv::Point>> DartArea::convertToContours(std::list<DartArea> dartAreas)
{
  std::vector<std::vector<cv::Point>> result;

  for(DartArea dartArea : dartAreas)
  {
    result.push_back(dartArea.contour);
  }

  return result;
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
