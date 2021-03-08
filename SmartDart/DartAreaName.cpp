#include "DartAreaName.h"

#include "Resources.h"

int DartAreaName::getValue() const
{
    return DartAreaMapping.at(clockwiseIndex_);
}

int DartAreaName::getIndex() const
{
  return clockwiseIndex_;
}

std::string DartAreaName::to_String()
{
  std::stringstream str;

  switch(multiplicator_)
  {
  case Single: 
  case Double: 
  case Triple:
    str << getPrefix();
    str << " ";
    str << getValue();
    break;
  case SingleBull:
    str << "Single Bull";
  case Bullseye:
    str << "Bullseye";
    break;
  default: return "";
  }

  return str.str();
}

cv::Scalar DartAreaName::getColor() const
{
  switch (multiplicator_)
  {
  case SingleBull:
    return _greenColor;
  case Bullseye:
    return _redColor;
  case Single:
    return _whiteColor;
  default:;
  }

  return clockwiseIndex_ % 2 == 0 ? _redColor : _greenColor;
}

int DartAreaName::getScore() const
{
  switch (multiplicator_)
  {
  case SingleBull:
    return 25;
  case Bullseye:
    return 50;
  default:;
  }

  return getValue() * (multiplicator_ + 1);
}

std::string DartAreaName::getPrefix()
{
  switch (multiplicator_)
  {
  case Single:
    return "Single";
  case Double:
    return "Double";
  case Triple:
    return "Triple";
  case SingleBull:
    return "Outer Bullseye";
  case Bullseye:
    return "Inner Bullseye";
  default:;
  }
  return "";
}

char DartAreaName::getShortPrefix() const
{
  switch(multiplicator_)
  {
  case Single: 
    return 'S';
  case Double: 
    return 'D';
  case Triple: 
    return 'T';
  case SingleBull: 
    return 'O';
  case Bullseye: 
    return 'I';
  default: ;
  }
  return -1;
}
