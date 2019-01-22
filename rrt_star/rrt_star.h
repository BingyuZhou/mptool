#include "rrt.h"

#include <cmath>
/*******************************************************
 * *****************RRT* *******************************
 * asymptotically optimal path
 * *****************************************************/
using namespace std;
/* ------------------------------------------
 * -------------RRT star node----------------
 * -----------------------------------------*/
template <class PointType>
class rrt_star_node : public rrt_node<PointType> {
  float m_cost = 0.0f;

 public:
  using rrt_node<PointType>::rrt;
  void set_cost(const float &);
  float get_cost();
};

template <class PointType>
void rrt_star_node<PointType>::set_cost(const float &cost) {
  m_cost = cost;
}

template <class PointType>
float rrt_star_node<PointType>::get_cost() {
  return m_cost;
}

/* ------------------------------------------
 * ----------------rrt_star------------------
 * ------------------------------------------*/
template <class PointType>
class rrt_star : public rrt<PointType> {
  typedef rrt<PointType> rrt_p;

 public:
  using rrt_p::extend;
  using rrt_p::rrt;

  bool extend(const PointType &sampled_node);
};

template <class PointType>
bool rrt_star<PointType>::extend(const PointType &sample_node) {
  PointType nearest_val = rrt_p::my_kdtree.nearest_neighbor(sample_node);
  PointType new_node = rrt_p::steer(nearest_val, sample_node);

  if (rrt_p::obstacle_free(nearest_val, new_node)) {
    PointType min_node = nearest_val;
    rrt_node<PointType> *node = new rrt_node<PointType>(new_node);
    auto nearest_itr = rrt_p::m_node_map.find(nearest_val);
    node->set_cost(nearest_itr->second->get_cost() +
                   euclidian_dis(nearest_val, new_node));
    // radius of the near neighbors is:
    // min{(gamma/eta * logn / n)^(1/d), steer_radius}
    // gamma:= 2^d * (1+1/d) * mu(X_free)
    // mu: Lebesgue measure
    // d: dimension, eg. 2-D, 3-D
    // eta: volum of unit radius ball
    // n: size of current tree
    // TODO:
    float gamma = pow(2, rrt_p::m_dimension) * (1 + 1.0f / rrt_p::m_dimension) *
                  Lebesgue_measure();
    float eta = M_PI;
    float size_tree = rrt_p::my_kdtree.m_size;
    float radius = min(static_cast<float>(rrt_p::m_steer_radius),
                       pow(gamma / eta * log(size_tree) / size_tree,
                           1.0f / rrt_p::m_dimension));
    // cout << "radius for near neighbour:" << radius << endl;
    auto near_nodes = rrt_p::my_kdtree.near_radius(new_node, radius);

    for (auto near : near_nodes) {
      if (rrt_p::obstacle_free(near, new_node)) {
        auto near_node_itr = rrt_p::m_node_map.find(near);
        float cost =
            near_node_itr->second->get_cost() + euclidian_dis(near, new_node);
        if (cost < node->get_cost()) min_node = near;
      }
    }
    auto min_node_itr = rrt_p::m_node_map.find(min_node);
    node->m_parent = min_node_itr->second;
    min_node_itr->second->m_children.push_back(node);
    rrt_p::my_kdtree.add_node(new_node);
    rrt_p::m_node_map.insert({new_node, node});

    // Rewiring
    for (auto near : near_nodes) {
      if (near == min_node) continue;
      if (rrt_p::obstacle_free(near, new_node)) {
        auto near_itr = rrt_p::m_node_map.find(near);
        if (near_itr->second->get_cost() >
            node->get_cost() + euclidian_dis(new_node, near)) {
          auto parent = near_itr->second->m_parent;
          // remove link between parent and near, link new_node with near
          auto pos = find(parent->m_children.begin(), parent->m_children.end(),
                          near_itr->second);
          parent->m_children.erase(pos);
          near_itr->second->m_parent = node;
          node->m_children.push_back(near_itr->second);
        }
      }
    }

    // Check if reach the goal
    if (euclidian_dis(new_node, rrt_p::m_goal) <= rrt_p::EPSILON) {
      rrt_p::m_reached = node;
      return true;
    }
  }
  return false;
}