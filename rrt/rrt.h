#pragma once

#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <vector>
#include "kdtree.h"
#include "obstacle_sym.h"

//------------------------DECLARITION------------------------
template <class PointType>
class rrt_node {
 public:
  rrt_node *m_parent;
  std::vector<rrt_node *> m_children;
  PointType m_value;
  rrt_node(const PointType &v);
  rrt_node(rrt_node *parent, std::vector<rrt_node *> children);
  rrt_node(rrt_node *parent, rrt_node *child);
  bool operator==(const rrt_node &obj) const;
};

template <class PointType>
struct rrt_node_hash {
  size_t operator()(const rrt_node<PointType> &obj) const {
    return boost::hash<PointType>()(obj.m_value);
  }
};

template <class PointType>
class rrt {
  const std::vector<int> m_state_space_boundary;
  const std::vector<obstacle *> m_obstacles;
  const int m_dimension;
  const PointType m_init;
  const PointType m_goal;
  const int m_radius;
  rrt_node<PointType> *m_root;
  std::unordered_map<PointType, rrt_node<PointType>, rrt_node_hash<PointType>>
      m_node_map;

 public:
  rrt(const std::vector<int> &state_space,
      const std::vector<obstacle *> &obstacles, const int &dim,
      const PointType &initial_point, const PointType &goal, const int &radius);

  KDtree<PointType> my_kdtree;
  PointType random_sample_2d();
  PointType random_sample_3d();
  bool obstacle_free(const PointType start, const PointType &end);
  PointType steer(const PointType &nearest, const PointType &sample);
  virtual void extend(const PointType &sampled_node);
};

//------------------------DEFINATION--------------------------
/***
 * RRT(Rapidly explored Random Tree) algorithm
 **/

#include "rrt.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <random>

template <class PointType>
rrt_node<PointType>::rrt_node(const PointType &v)
    : m_parent(NULL), m_value(v){};

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
bool rrt_node<PointType>::operator==(const rrt_node &obj) const {
  if (obj.m_value == m_value)
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
      m_init(initial_point),
      m_goal(goal),
      m_radius(radius) {
  m_node_map.insert({initial_point, rrt_node<PointType>(initial_point)});
};

template <class PointType>
PointType rrt<PointType>::random_sample_2d() {
  auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  std::uniform_real_distribution<float> distribution_x(
      m_state_space_boundary[0], m_state_space_boundary[1]);
  std::uniform_real_distribution<float> distribution_y(
      m_state_space_boundary[2], m_state_space_boundary[3]);

  float x = distribution_x(generator);
  float y = distribution_y(generator);
  PointType sample_point;

  auto inside_obstacle_area = [&](const obstacle *obs) {
    if (x > obs->top_left.first && x < obs->bottom_right.first &&
        y > obs->bottom_right.second && y < obs->top_left.second)
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
PointType rrt<PointType>::random_sample_3d(){};

template <class PointType>
bool rrt<PointType>::obstacle_free(const PointType start,
                                   const PointType &end) {
  // TODO:
  return true;
};

template <class PointType>
PointType rrt<PointType>::steer(const PointType &nearest,
                                const PointType &sample) {
  PointType new_node;
  float dis = euclidian_dis(nearest, sample);
  if (dis <= m_radius)
    new_node = sample;
  else {
    for (int i = sample.size() - 1; i >= 0; --i) {
      new_node[i] = (sample[i] - nearest[i]) / dis * m_radius + nearest[i];
    }
  }
  return new_node;
};

template <class PointType>
void rrt<PointType>::extend(const PointType &sampled_node) {
  PointType nearest_val = my_kdtree.nearest_neighbor(sampled_node);

  typename std::unordered_map<PointType, rrt_node<PointType>,
                              rrt_node_hash<PointType>>::iterator nearest_it =
      m_node_map.find(nearest_val);
  assert(nearest_it != m_node_map.end());

  PointType new_node = steer(nearest_it->first, sampled_node);

  if (obstacle_free(nearest_it->first, new_node)) {
    rrt_node<PointType> node(new_node);
    nearest_it->second.m_children.push_back(&node);
    node.m_parent = &(nearest_it->second);
  }
}