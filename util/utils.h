#pragma once

#include <cassert>
#include <cmath>

template <class T>
float euclidian_dis(const T& n1, const T& n2) {
  assert(n1.size() == n2.size());
  float dis = 0;

  for (int i = n1.size() - 1; i >= 0; --i) {
    dis += (n1[i] - n2[i]) * (n1[i] - n2[i]);  // Euclidean distance
  }
  return std::sqrt(dis);
}