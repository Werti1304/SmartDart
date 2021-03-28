#include "PointHelper.h"

#include <opencv2/imgproc.hpp>

void PointHelper::printPoint(cv::Point p)
{
  printf("(%d/%d)", p.x, p.y);
}

// 15 times faster than the classical float sqrt. 
// Reasonably accurate up to root(32500)
// Source: http://supp.iar.com/FilesPublic/SUPPORT/000419/AN-G-002.pdf
unsigned int root(unsigned int x)
{
  const unsigned int b = x;
  unsigned int a = x = 0x3f;
  x = b / x;
  a = x = (x + a) >> 1;
  x = b / x;
  a = x = (x + a) >> 1;
  x = b / x;
  x = (x + a) >> 1;
  return(x);
}

int PointHelper::getDistance(cv::Point pt1, cv::Point pt2)
{
  // Calculating distance
  return root(pow(pt2.x - pt1.x, 2) +
    pow(pt2.y - pt1.y, 2));
}

cv::Point PointHelper::getMassCenter(std::vector<cv::Point> contour)
{
  cv::Moments m = cv::moments(contour);
  return { m.m10 / m.m00, m.m01 / m.m00 };
}

// Returns point that is clockwise further as first element in array
std::array<cv::Point, 2> PointHelper::sortClockwise(cv::Point& orientationPt, std::array<cv::Point, 2> comparePts)
{
  const auto directionPt = orientationPt - (comparePts[0] + comparePts[1]) / 2; // Take the mean of both of them

  if (directionPt.x < 0)  // NOLINT(bugprone-branch-clone)
  {
    if (directionPt.y < 0)  // NOLINT(bugprone-branch-clone)
    {
      // Both negative
      if (comparePts[1].y > comparePts[0].y)
      {
        return { comparePts[1], comparePts[0] };
      }
    }
    else
    {
      // X negative Y positive
      if (comparePts[1].x > comparePts[0].x)
      {
        return { comparePts[1], comparePts[0] };
      }
    }
  }
  else
  {
    if (directionPt.y < 0)  // NOLINT(bugprone-branch-clone)
    {
      // Y negative X positive
      if (comparePts[1].x < comparePts[0].x)
      {
        return { comparePts[1], comparePts[0] };
      }
    }
    else
    {
      // Both positive
      if (comparePts[1].y < comparePts[0].y)
      {
        return { comparePts[1], comparePts[0] };
      }
    }
  }

  return comparePts;
}
