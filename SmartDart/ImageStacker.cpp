#include "ImageStacker.h"
#include "WindowHelper.h"

ImageStacker::ImageStacker(std::string title, int width, int height) : title(title), width(width), height(height)
{
  cv::namedWindow(title, cv::WINDOW_NORMAL); // WINDOW_NORMAL needed for resizeWindow() func
  cv::resizeWindow(title, width / 2, height / 2);
}

void ImageStacker::addImage(cv::Mat image, bool display)
{
  imgs.push_back(image);
  if (display)
  {
    showImages();
  }
}

// --------------------------------------------------------------
// Function to draw several images to one image.
// Small images draws into cells of size cellSize.
// If image larger than size of cell ot will be trimmed.
// If image smaller than cellSize there will be gap between cells.
// 
void ImageStacker::showImages()
{
  int nImgs = imgs.size();

  if (nImgs == 1)
  {
    cv::imshow(title, imgs[0]);
    return;
  }

  float cellSide = sqrt(nImgs);
  int cellSideX = ceil(cellSide);
  int cellSideY = (float)(cellSideX - cellSide) > 0.5 ? cellSideX - 1 : cellSideX;
  
  cv::Mat imgArr[cellSideX];
  cv::Mat out[cellSideX];

  int idx = 0;


  WindowHelper windowHelper("", height / 2, width / 2);

  for (int i = 0; i < cellSideY; i++)
  {
    for (int j = 0; j < cellSideX; j++)
    {
      if (idx >= nImgs) // Add an empty image to compensate
      {
        imgArr[j] = cv::Mat::zeros(imgs[0].rows, imgs[0].cols, imgs[0].type());
      }
      else
      {
        imgArr[j] = imgs[idx++];
      }
    }
    cv::hconcat(imgArr, cellSideX, out[i]);
    windowHelper.imgshowResized(std::to_string(i), out[i]);
  }
  
  if (nImgs == 2)
  {
    cv::imshow(title, out[0]);
    return;
  }

  cv::vconcat(out, cellSideY, output);

  cv::imshow(title, output);
}
