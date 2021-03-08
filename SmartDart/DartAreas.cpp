#include "DartAreas.h"

#include <opencv2/imgproc.hpp>
#include "Resources.h"

#include "PointHelper.h"

using namespace PointHelper;

DartArea::DartArea(std::vector<cv::Point> inputPoints, bool drawAsContours) : contour(inputPoints), drawAsContours(drawAsContours)
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

void DartArea::draw(cv::Mat& src, const cv::Scalar& color, int thickness) const
{
  if(drawAsContours)
  {
    std::vector<std::vector<cv::Point> > contourVec;
    contourVec.push_back(contour);
    cv::drawContours(src, contourVec, 0, color, 3); //Replace i with 0 for index.
    return;
  }

  cv::polylines(src, contour, true, color, thickness);
}

void DartArea::drawUsingNameColor(cv::Mat& src, int thickness) const
{
  cv::Scalar color = name.getColor();

  draw(src, color, thickness);
}

bool DartArea::isRed() const
{
  return red;
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
