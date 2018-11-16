#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <algorithm>
#include <fstream>
#include "../rrt/rrt.h"

const float EPS = 0.1f;
typedef std::array<float, 2> point_2d;

GTEST("test_rrt") {
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

  point_2d initial_point{5.0f, -9.0f};
  point_2d goal{0.0f, 10.0f};
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

    EXPECT_FALSE(inside_obstacle_area<point_2d>(obs1, sample));
    EXPECT_FALSE(inside_obstacle_area<point_2d>(obs2, sample));
    EXPECT_FALSE(inside_obstacle_area<point_2d>(obs3, sample));
    EXPECT_FALSE(inside_obstacle_area<point_2d>(obs4, sample));
  }

  SHOULD("test_steer_functionality") {
    point_2d sample = rrt_solver.random_sample_2d();
    point_2d nearest{5, 5};
    point_2d new_node = rrt_solver.steer(nearest, sample);
    // std::cout << new_node[0] << " " << new_node[1] << std::endl;

    EXPECT_LE(euclidian_dis(new_node, nearest), radius + EPS);
  }

  SHOULD("COLLISION_CHECK") {
    point_2d start{0, 1};
    point_2d end{10, 1};
    bool res = rrt_solver.obstacle_free(start, end);
    EXPECT_FALSE(res);

    start = {9.5, -2};
    end = {8, 6};
    res = rrt_solver.obstacle_free(start, end);
    EXPECT_FALSE(res);

    start = {7.2, -2};
    end = {3.5, 9};
    res = rrt_solver.obstacle_free(start, end);
    EXPECT_TRUE(res);

    start = {2, 0};
    end = {3, 0};
    res = rrt_solver.obstacle_free(start, end);
    EXPECT_TRUE(res);
  }
  SHOULD("TEST_EXTEND_GRAPH") {
    point_2d sample = rrt_solver.random_sample_2d();
    // std::cout << sample[0] << " " << sample[1] << std::endl;
    bool goal_reached = rrt_solver.extend(sample);

    Node<point_2d>* root = rrt_solver.my_kdtree.get_root();

    std::queue<Node<point_2d>*> q;
    q.push(root);

    while (!q.empty()) {
      Node<point_2d>* node = q.front();
      q.pop();
      std::cout << node->m_value[0] << " " << node->m_value[1] << std::endl;
      if (node->m_left) q.push(node->m_left);
      if (node->m_right) q.push(node->m_right);
    }
  }

  SHOULD("FIND_THE_PATH") {
    bool reached = rrt_solver.run(100000);
    std::vector<point_2d> p;
    EXPECT_TRUE(reached);
    if (reached) {
      rrt_solver.get_path(p);
      std::ofstream file;
      file.open("path.csv");
      std::for_each(p.begin(), p.end(), [&](point_2d point) {
        file << point[0] << " " << point[1] << "\n";
      });
      file.close();
    }
  }
}