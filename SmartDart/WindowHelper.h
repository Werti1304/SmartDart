#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class WindowHelper
{
public:
  /// <summary>
  /// WindowHelper meant for automatic creation of resized windows that are adjusted to the Raspis resolution
  /// </summary>
  /// <param name="windowHeight">Preffered window height</param>
  /// <param name="windowWidth">Preffered window width</param>
  WindowHelper(std::string defaultPath, int windowHeight, int windowWidth);

  void namedWindowResized(std::string windowName, int height, int width);
  void namedWindowResized(std::string windowName);

  void imgshowResized(std::string name, cv::Mat img, int height = -1, int width = -1);

  /// <summary>
  /// Reads image relative to defaultPath
  /// </summary>
  cv::Mat imreadRel(std::string relativePath);

private:
  std::string defaultPath;
  int windowHeight;
  int windowWidth;
};