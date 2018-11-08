/***
 * RRT(Rapidly explored Random Tree) algorithm
 **/

#include "rrt.h"
#include "../util/kdtree.h"

#include <algorithm>
#include <chrono>
#include <random>

template <class PointType>
rrt_node<PointType>::rrt_node() : m_parent(NULL){};

template <class PointType>
rrt_node<PointType>::rrt_node(rrt_node *parent,
                              std::vector<rrt_node *> children)
    : m_parent(parent), m_children(children){};

template <class PointType>
rrt_node<PointType>::rrt_node(rrt_node *parent, rrt_node *child)
    : m_parent(parent) {
  m_children.push_back(child);
};

//-----------------------rrt class----------------------

template <class PointType>
rrt<PointType>::rrt(const std::vector<int> &state_space,
                    const std::vector<obstacle *> &obstacles, const int &dim)
    : m_state_space_boundary(state_space),
      m_obstacles(obstacles),
      m_dimension(dim){};

template <class PointType>
PointType rrt<PointType>::random_sample_2d() {
  auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution_x(m_state_space_boundaty[0],
                                                    m_state_space_boundaty[1]);
  std::uniform_int_distribution<int> distribution_y(m_state_space_boundaty[2],
                                                    m_state_space_boundary[3]);

  int x = distribution_x(generator);
  int y = distribution_y(generator);
  PointType sample_point;

  auto inside_obstacle_area = [&](const obstacle *obs) {
    if (x > obs->top_left.first() && x < obs->bottom_right.first() &&
        y > obs->bottom_right.second() && y < obs->top_left.second())
      return true;
    else
      return false;
  };
  int count = 0;
  while (true) {
    for (auto obs : m_obstacles) {
      if (inside_obstacle_area(obs)) {
        x = distribution_x(generator);
        y = distribution_y(generator);
        count = 0;
        break;
      }
      ++count;
    }
    if (count == m_obstacles.size()) break;
  }
  sample_point[0] = x;
  sample_point[1] = y;
  return sample_point;
}

template <class PointType>
void rrt<PointType>::extend(rrt_node<PointType> *sampled_node) {}