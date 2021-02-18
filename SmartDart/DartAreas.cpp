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

DartArea::DartArea(std::vector<cv::Point> inputPoints)
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

std::list<DartArea> DartArea::defineDartBoard(cv::Mat& src, std::list<DartArea> greenContours, std::list<DartArea> redContours)
{
  // TODO: Implement
  return std::list<DartArea>();

  for(DartArea redArea : redContours)
  {
    for(cv::Point significantPointRed : redArea.significantPoints)
    {
      int maxDistance = getDistance(significantPointRed, redArea.meanPoint);

      for(DartArea greenContour : greenContours)
      {
        for (cv::Point significantPointGreen : redArea.significantPoints)
        {
          if(getDistance(significantPointGreen, significantPointRed) < maxDistance)
          {
            //redArea.connectedAreas.push_back(&greenContour);
            break;
          }
        }
      }
    }
  }  
}

void DartArea::markAreas(cv::Mat& src, std::list<DartArea> dartAreas, int radius, const cv::Scalar& color, int thickness)
{
  int i = 0;
  for (DartArea area : dartAreas)
  {
    area.markArea(src, radius, color, thickness);
  }
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
