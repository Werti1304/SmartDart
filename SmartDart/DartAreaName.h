#pragma once
#include <bits/stringfwd.h>
#include <opencv2/core/types.hpp>

class DartAreaName
{
public:
  enum Multiplicator
  {
    Single,
    Double,
    Triple,
    SingleBull,
    Bullseye,
    Undefined
  };

  explicit DartAreaName(int clockwiseIndex, Multiplicator multiplicator)
    : clockwiseIndex_(clockwiseIndex), multiplicator_(multiplicator)
  {}
  explicit DartAreaName()
    : clockwiseIndex_(0), multiplicator_(Undefined)
  {}

  int getValue() const;
  int getIndex() const;

  std::string to_String();

  cv::Scalar getColor() const;

  /**
   * \return Points for scoring
   */
  int getScore() const;

  /// <summary>
  /// Returns short Multiplicator Prefix
  /// </summary>
  /// <returns>Single Double or Triple, empty string as error/undefined value</returns>
  std::string getPrefix();

  /// <summary>
  /// Returns short Multiplicator Prefix
  /// </summary>
  /// <returns>S D or T, -1 as error/undefined value</returns>
  char getShortPrefix() const;

private:
  int clockwiseIndex_;
  Multiplicator multiplicator_;
};
