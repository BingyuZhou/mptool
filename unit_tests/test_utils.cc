#include "utils.h"

#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <algorithm>

using namespace std;
typedef array<int, 2> point_2d;

GTEST("test_Lebesgue_measure") {
  std::vector<int> state_space{0, 10, -10, 10};
  obstacle *obs1 =
      new obstacle(std::make_pair(1.0f, 9.0f), std::make_pair(3.0f, 7.0f));
  obstacle *obs2 =
      new obstacle(std::make_pair(9.0f, 3.0f), std::make_pair(10.0f, -10.0f));
  obstacle *obs3 =
      new obstacle(std::make_pair(2.0f, 0.0f), std::make_pair(4.0f, -3.0f));
  obstacle *obs4 =
      new obstacle(std::make_pair(8.0f, 9.0f), std::make_pair(9.0f, 7.0f));

  std::vector<obstacle *> obstacles{obs1, obs2, obs3, obs4};

  float free_area = Lebesgue_measure(state_space, obstacles);
  EXPECT_EQ(free_area, 175.0f);
}