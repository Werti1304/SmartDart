#pragma once
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

  void namedWindowResized(std::string windowName, int height, int width) const;
  void namedWindowResized(std::string windowName) const;

  void imgshowResized(std::string name, cv::Mat img, int height = -1, int width = -1) const;

  /// <summary>
  /// Reads image relative to defaultPath
  /// </summary>
  cv::Mat imreadRel(std::string relativePath);

  template<size_t size>
  void switchableImgs(std::string name, std::array<cv::Mat, size> imgArr);

  template <class ... Imgs>
  void switchableImgsPack(std::string name, Imgs ... imgs);

private:
  std::string defaultPath;
  int windowHeight;
  int windowWidth;
};

template<size_t size>
void WindowHelper::switchableImgs(std::string name, std::array<cv::Mat, size> imgArr)
{
  int i = size - 1;
  int c;
  do
  {
    imgshowResized(name, imgArr[i]);

    c = cv::waitKey(0);

    switch (c)
    {
    case 'w':
      ++i;
      break;
    case 's':
      --i;
      break;
    default:
      break;
    }

    if (i < 0)
    {
      i = imgArr.size() - 1;
    }
    else if (i >= imgArr.size())
    {
      i = 0;
    }
  } while (c == 'w' || c == 's');
}

template<typename ...Imgs>
void WindowHelper::switchableImgsPack(std::string name, Imgs... imgs)
{
  cv::Mat imgArr[] = { imgs... };

  switchableImgsPack(imgArr);
}