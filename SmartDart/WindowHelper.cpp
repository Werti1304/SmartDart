#include "WindowHelper.h"

WindowHelper::WindowHelper(std::string defaultPath, int windowHeight, int windowWidth) : defaultPath(defaultPath), windowHeight(windowHeight), windowWidth(windowWidth)
{}

void WindowHelper::namedWindowResized(std::string windowName, int height, int width)
{
  cv::namedWindow(windowName, cv::WINDOW_NORMAL); // WINDOW_NORMAL needed for resizeWindow() func
  cv::resizeWindow(windowName, width, height);
}

void WindowHelper::namedWindowResized(std::string windowName)
{
  namedWindowResized(windowName, windowHeight, windowWidth);
}

//TODO: Check if window already exists, possible performanceloss here because of constant resizing of window
/// <summary>
/// Shows img in properly resized window. Multiple calls don't pose a problem.
/// </summary>
/// <param name="name">Name of the window.</param>
/// <param name="img">image to be displayed. Any dimensions are acceptable.</param>
/// <param name="height">Height of the window. Defaults to saved height.</param>
/// <param name="width">Width of the window. Defaults to saved width.</param>
void WindowHelper::imgshowResized(std::string name, cv::Mat img, int height, int width)
{
  // Only checking for the default parameters here
  if (height == -1 || width == -1)
  {
    namedWindowResized(name);
  }
  else
  {
    namedWindowResized(name, height, width);
  }
  
  cv::imshow(name, img);
}

cv::Mat WindowHelper::imreadRel(std::string relativePath)
{
  return cv::imread(defaultPath + relativePath);
}