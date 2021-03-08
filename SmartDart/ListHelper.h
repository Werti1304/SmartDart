#pragma once
#include <bits/stl_list.h>

namespace ListHelper
{
  template<class T>
  void ListHelper::doListStats(std::list<T> list);
}

template<class T>
void ListHelper::doListStats(std::list<T> list)
{
  T min = numeric_limits<T>::max();
  T max = 0;
  double sum;

  for (auto i : list)
  {
    if (i < min)
    {
      min = i;
    }
    if (i > max)
    {
      max = i;
    }
    sum += i;
  }

  T valCount = list.size();
  T mean = sum / valCount;

  std::cout << "Values: " << valCount << "\nSum: " << sum << "\nMean: " << mean << "\nMin: " << min << "\nMax: " << max << endl;
}