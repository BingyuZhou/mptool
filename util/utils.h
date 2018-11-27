#pragma once

#include <cassert>
#include <cmath>
#include <json.hpp>
#include "obstacle_sym.h"

template <class T>
float euclidian_dis(const T &n1, const T &n2) {
  assert(n1.size() == n2.size());
  float dis = 0.0f;

  for (int i = n1.size() - 1; i >= 0; --i) {
    dis += (n1[i] - n2[i]) * (n1[i] - n2[i]);  // Euclidean distance
  }
  return std::sqrt(dis);
}

template <class T>
bool inside_obstacle_area(const obstacle *obs, T point) {
  if (point[0] > obs->top_left.first && point[0] < obs->bottom_right.first &&
      point[1] > obs->bottom_right.second && point[1] < obs->top_left.second)
    return true;
  else
    return false;
};
