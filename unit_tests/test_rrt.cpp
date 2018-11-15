#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include "../rrt/rrt.h"

const float EPS = 0.1f;
typedef std::array<float, 2> point_2d;

GTEST("test_rrt_sample") {
  std::vector<int> state_space{0, 10, -10, 10};
  obstacle* obs1 =
      new obstacle(std::make_pair(1.0f, 9.0f), std::make_pair(3.0f, 7.0f));
  obstacle* obs2 =
      new obstacle(std::make_pair(9.0f, 3.0f), std::make_pair(10.0f, -10.0f));
  obstacle* obs3 =
      new obstacle(std::make_pair(2.0f, 0.0f), std::make_pair(4.0f, -3.0f));
  obstacle* obs4 =
      new obstacle(std::make_pair(8.0f, 9.0f), std::make_pair(9.0f, 7.0f));

  std::vector<obstacle*> obstacles{obs1, obs2, obs3, obs4};

  point_2d initial_point{0.0f, -10.0f};
  point_2d goal{10.0f, 10.0f};
  int radius = 1;
  int dim = 2;
  rrt<point_2d> rrt_solver(state_space, obstacles, dim, initial_point, goal,
                           radius);

  SHOULD("sample_point_in_state_space") {
    point_2d sample = rrt_solver.random_sample_2d();
    // std::cout << sample[0] << " " << sample[1] << std::endl;
    EXPECT_GE(sample[0], 0);
    EXPECT_LE(sample[0], 10);
    EXPECT_LE(sample[1], 10);
    EXPECT_GE(sample[1], -10);

    auto inside_obstacle_area = [](point_2d point, const obstacle* obs) {
      if (point[0] > obs->top_left.first &&
          point[0] < obs->bottom_right.first &&
          point[1] > obs->bottom_right.second &&
          point[1] < obs->top_left.second)
        return true;
      else
        return false;
    };

    EXPECT_FALSE(inside_obstacle_area(sample, obs1));
    EXPECT_FALSE(inside_obstacle_area(sample, obs2));
    EXPECT_FALSE(inside_obstacle_area(sample, obs3));
    EXPECT_FALSE(inside_obstacle_area(sample, obs4));
  }

  SHOULD("test_steer_functionality") {
    point_2d sample = rrt_solver.random_sample_2d();
    point_2d nearest{5, 5};
    point_2d new_node = rrt_solver.steer(nearest, sample);
    std::cout << new_node[0] << " " << new_node[1] << std::endl;

    EXPECT_LE(euclidian_dis(new_node, nearest), radius + EPS);
  }
}