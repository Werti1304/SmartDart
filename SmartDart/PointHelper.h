#pragma once
#include <opencv2/core/types.hpp>

namespace PointHelper
{
  void printPoint(cv::Point p);
  int getDistance(cv::Point pt1, cv::Point pt2);

  cv::Point getMassCenter(std::vector<cv::Point> contour);

  template<class T, size_t size>
  std::array<cv::Point, size> findPoints(T pointContainer, std::array<cv::Point, size> comparePts, bool nearest = true);

  std::array<cv::Point, 2> sortClockwise(cv::Point& orientationPt, std::array<cv::Point, 2> comparePts);
};

/// <summary>
/// Finds (size) points that are (nearest/furthest) away in (pointContainer)
/// </summary>
/// <typeparam name="T">Iterable container for points</typeparam>
/// <param name="pointContainer"></param>
/// <param name="nearest"></param>
/// <param name="comparePts"></param>
/// <returns></returns>
template<class T, size_t size>
std::array<cv::Point, size> PointHelper::findPoints(T pointContainer, std::array<cv::Point, size> comparePts, bool nearest)
{
  std::array<cv::Point, size> outPts;

  if (nearest)
  {
    unsigned distances[size];
    std::fill(distances, distances + size, INT16_MAX);

    for (cv::Point pt : pointContainer)
    {
      for (auto i = 0; i < size; i++)
      {
        int distTmp = getDistance(pt, comparePts[i]);
        if (distTmp < distances[i])
        {
          distances[i] = distTmp;
          outPts[i] = pt;
        }
      }
    }
  }
  else
  {
    unsigned distances[size] = { 0 };
    for (cv::Point pt : pointContainer)
    {
      for (auto i = 0; i < size; i++)
      {
        int distTmp = getDistance(pt, comparePts[i]);
        if (distTmp > distances[i])
        {
          distances[i] = distTmp;
          outPts[i] = pt;
        }
      }
    }
  }
  return outPts;
}