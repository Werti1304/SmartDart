#include "DartAreas.h"


#include <iostream>
#include <opencv2/imgproc.hpp>

DartAreas::DartAreas(cv::Mat& src, std::vector<std::vector<cv::Point>> contours) : src(src), contours(contours)
{
  calculateAreas();
}

void printPoint(cv::Point p)
{
  printf("(%d/%d)", p.x, p.y);
}

void DartAreas::calculateAreas()
{
  for(const auto contour : contours)
  {
    //printf("Area %d: ", i++);
    //printPoint(test.corners[DartArea::TopMost]);
    //printPoint(test.corners[DartArea::RightMost]);
    //printPoint(test.corners[DartArea::BottomMost]);
    //printPoint(test.corners[DartArea::LeftMost]);
    //putchar('\n');
    dartAreaList.push_back(DartArea(contour));
  }

  //int i = 0;
  //for(auto contour : contours)
  //{
  //  printf("Contour %d: ", i++);
  //  for(const auto point : contour)
  //  {
  //    printPoint(point);
  //  }
  //  putchar('\n');
  //}
}

DartAreas::DartArea::DartArea(std::vector<cv::Point> inputPoints)
{
  // Initialize with minimum values for loop
  corners[TopMost].y = 0;
  corners[RightMost].x = 0;
  corners[LeftMost].x = INT16_MAX;
  corners[BottomMost].y = INT16_MAX;

  for(const auto pt : inputPoints)
  {
    if(corners[TopMost].y < pt.y)
    {
      corners[TopMost] = pt;
    }
    else if(corners[RightMost].x < pt.x)
    {
      corners[RightMost] = pt;
    }
    else if(corners[LeftMost].x > pt.x)
    {
      corners[LeftMost] = pt;
    }
    else if(corners[BottomMost].y > pt.y)
    {
      corners[BottomMost] = pt;
    }
  }
}

cv::Mat DartAreas::markAreas(int radius, const cv::Scalar& color, int thickness)
{
  int i = 0;
  for(DartArea area : dartAreaList)
  {
    std::cout << "Area " << ++i << "\n";
    for(const cv::Point pt : area.corners)
    {
      cv::circle(src, pt, radius, color, thickness);
    }
  }
  return src;
}


