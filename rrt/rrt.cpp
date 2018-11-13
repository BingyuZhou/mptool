/***
 * RRT(Rapidly explored Random Tree) algorithm
 **/

#include "rrt.h"
#include "../util/kdtree.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <random>

template <class PointType>
rrt_node<PointType>::rrt_node(PointType &v) : m_parent(NULL), m_value(v){};

template <class PointType>
rrt_node<PointType>::rrt_node(rrt_node *parent,
                              std::vector<rrt_node *> children)
    : m_parent(parent), m_children(children){};

template <class PointType>
rrt_node<PointType>::rrt_node(rrt_node *parent, rrt_node *child)
    : m_parent(parent) {
  m_children.push_back(child);
};

template <class PointType>
bool rrt_node<PointType>::operator==(const rrt_node &obj) {
  if (obj.m_value == this->m_value)
    return true;
  else
    return false;
};

//-----------------------rrt class----------------------

template <class PointType>
rrt<PointType>::rrt(const std::vector<int> &state_space,
                    const std::vector<obstacle *> &obstacles, const int &dim,
                    const PointType &initial_point, const PointType &goal,
                    const int &radius)
    : m_state_space_boundary(state_space),
      m_obstacles(obstacles),
      m_dimension(dim),
      m_initial(initial_point),
      m_goal(goal),
      m_radius(radius) {
  m_node_set.insert(rrt_node<PointType>(initial_point));
};

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

PointType random_sample_3d();
bool obstacle_free();

template <class PointType>
PointType rrt<PointType>::steer(const rrt_node<PointType> &nearest,
                                const PointType &sample) {
  PointType new_node;
  float dis = euclidian_dis(nearest.m_value, sample);
  if (dis <= m_radius)
    new_node = sample;
  else {
    for (int i = sample.size(); i > 0; --i) {
      new_node[i] = (sample[i] - nearest.m_value[i]) / dis * m_radius;
    }
  }
  return new_node;
};

template <class PointType>
void rrt<PointType>::extend(const PointType &sampled_node) {
  PointType nearest_val = my_kdtree.nearest_neighbor(sampled_node);
  rrt_node<PointType> tmp = rrt_node<PointType>(nearest_val);
  std::unordered_set<rrt_node<PointType>, rrt_node_hash>::const_iterator
      nearest_it = m_node_set.find(tmp);
  assert(nearest_it != m_node_set.end());

  PointType new_node = steer(*it, sampled_node);
}