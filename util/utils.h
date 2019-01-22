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
}

float Lebesgue_measure(const std::vector<int> &state_space,
                       const std::vector<obstacle *> &obses) {
  // For E(2) spaces, return the free area size
  float all_area =
      (state_space[1] - state_space[0]) * (state_space[3] - state_space[2]);
  float obs_area = 0.0f;
  for (auto obs : obses) {
    obs_area += (obs->bottom_right.first - obs->top_left.first) *
                (obs->top_left.second - obs->bottom_right.second);
  }
  return all_area - obs_area;
}