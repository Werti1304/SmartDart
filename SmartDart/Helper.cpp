#include "Helper.h"

Mat Helper::readImageFromFile(std::string fileName, int flag = IMREAD_COLOR)
{
  return imread(fileName, flag);
}