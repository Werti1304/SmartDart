#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class ImageStacker
{
public:
  ImageStacker(std::string title, int width, int height);

  void addImage(cv::Mat image, bool display = false);

  void showImages();
private:
  cv::Mat output;
  std::vector<cv::Mat> imgs;
  std::string title;
  int width; 
  int height;
};