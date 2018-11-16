#pragma once

/***
 * RRT(Rapidly explored Random Tree) algorithm
 **/

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
  const float EPSILON = 0.05;  // distance tolerant at goal location
  const float m_bias = 0.1;    // bias of goal distribution

  std::unordered_map<PointType, rrt_node<PointType> *, rrt_node_hash<PointType>>
      m_node_map;  // have to use map, since we need to modify the node
  rrt_node<PointType> *m_reached;  // reached point near goal

 public:
  rrt(const std::vector<int> &state_space,
      const std::vector<obstacle *> &obstacles, const int &dim,
      const PointType &initial_point, const PointType &goal, const int &radius);
  void set_bias(float bias);
  KDtree<PointType> my_kdtree;
  PointType random_sample_2d();
  PointType random_sample_3d();
  bool obstacle_free(const PointType start, const PointType &end);
  PointType steer(const PointType &nearest, const PointType &sample);
  virtual bool extend(const PointType &sampled_node);

  bool run(int iteration);
  void get_path(std::vector<PointType> &path);
};

//------------------------DEFINATION--------------------------

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
  rrt_node<PointType> *start = new rrt_node<PointType>(initial_point);
  m_node_map.insert({initial_point, start});
  std::vector<PointType> point_list{initial_point};
  my_kdtree.construct_tree(point_list);
};

template <class PointType>
void rrt<PointType>::set_bias(float bias) {
  m_bias = bias;
}

template <class PointType>
PointType rrt<PointType>::random_sample_2d() {
  if (rand() % 10 / 10.0f < m_bias)
    return m_goal;
  else {
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::uniform_real_distribution<float> distribution_x(
        m_state_space_boundary[0], m_state_space_boundary[1]);
    std::uniform_real_distribution<float> distribution_y(
        m_state_space_boundary[2], m_state_space_boundary[3]);

    PointType sample_point;
    sample_point[0] = distribution_x(generator);
    sample_point[1] = distribution_y(generator);

    int count = 0;
    while (true) {
      for (auto obs : m_obstacles) {
        if (inside_obstacle_area<PointType>(obs, sample_point)) {
          sample_point[0] = distribution_x(generator);
          sample_point[1] = distribution_y(generator);
          count = 0;
          break;
        }
        ++count;
      }
      if (count == m_obstacles.size()) break;
    }

    return sample_point;
  }
}

template <class PointType>
PointType rrt<PointType>::random_sample_3d(){};

template <class PointType>
bool rrt<PointType>::obstacle_free(const PointType start,
                                   const PointType &end) {
  // Note: the distance form start to end less or equal to radius
  float dis = euclidian_dis(start, end);
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(0, dis);

  // Sampling check for 10 samples, O(10*N) where N is the number of obstacles
  for (int i = 0; i < 10; ++i) {
    PointType point;
    float sample_dis = distribution(generator);
    point[0] = start[0] + sample_dis / dis * (end[0] - start[0]);
    point[1] = start[1] + sample_dis / dis * (end[1] - start[1]);

    for (auto obs : m_obstacles)
      if (inside_obstacle_area<PointType>(obs, point)) return false;
  }
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
bool rrt<PointType>::extend(const PointType &sampled_node) {
  PointType nearest_val = my_kdtree.nearest_neighbor(
      sampled_node);  // find the nearest neighbor in kdtree

  typename std::unordered_map<PointType, rrt_node<PointType> *,
                              rrt_node_hash<PointType>>::iterator nearest_it =
      m_node_map.find(nearest_val);  // find the equivalent node in rrt
  assert(nearest_it != m_node_map.end());

  PointType new_node = steer(nearest_it->first, sampled_node);

  if (obstacle_free(nearest_it->first, new_node)) {
    rrt_node<PointType> *node = new rrt_node<PointType>(new_node);
    nearest_it->second->m_children.push_back(node);
    node->m_parent = nearest_it->second;
    my_kdtree.add_node(new_node);
    m_node_map.insert({new_node, node});

    if (euclidian_dis(new_node, m_goal) <= EPSILON) {
      m_reached = node;
      return true;
    }
    return false;
  }
}

/**
 * RRT
 * rrt initializes with start state
 * for i=1:N:
 *    x_rand = sample(i)
 *    x_nearest = kd_tree.NN(x_rand)
 *    x_new = steer(x_nearest, x_rand)
 *    if(obstacle_free(x_new, x_nearest))
 *        kd_tree.add(x_new)
 *        rrt.add(x_new)
 *        check if x_new at goal location
 **/
template <class PointType>
bool rrt<PointType>::run(int iteration) {
  for (int i = 0; i < iteration; ++i) {
    PointType x_rand;
    if (m_dimension == 2) x_rand = random_sample_2d();
    if (m_dimension == 3) x_rand = random_sample_3d();

    if (extend(x_rand)) return true;  // reach the goal location
  }
  return false;  // run out of iteration with failing to reach goal
}

template <class PointType>
void rrt<PointType>::get_path(std::vector<PointType> &path) {
  rrt_node<PointType> *tmp = m_reached;
  while (tmp) {
    path.push_back(tmp->m_value);
    tmp = tmp->m_parent;
  }
}
