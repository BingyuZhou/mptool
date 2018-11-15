#pragma once

#include <utility>

struct obstacle {
  const std::pair<float, float> top_left;
  const std::pair<float, float> bottom_right;

  obstacle(const std::pair<float, float>& tl, const std::pair<float, float>& br)
      : top_left(tl), bottom_right(br){};
};