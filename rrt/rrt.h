#include <unordered_set>
#include <vector>
#include "../util/obstacle_sym.h"
template <class PointType>
class rrt_node {
 public:
  rrt_node *m_parent;
  std::vector<rrt_node *> m_children;
  PointType m_value;
  rrt_node(PointType &v);
  rrt_node(rrt_node *parent, std::vector<rrt_node *> children);
  rrt_node(rrt_node *parent, rrt_node *child);
  bool operator==(const rrt_node &obj){};
};

template <class PointType>
struct rrt_node_hash {
  size_t operator()(const rrt_node<PointType> &obj) {
    return std::hash<PointType>()(obj.m_value);
  }
};

template <class PointType>
class rrt : private KDtree {
  const std::vector<int> m_state_space_boundaty;
  const std::vector<obstacle *> m_obstacles;
  const int m_dimension;
  const PointType m_init;
  const PointType m_goal;
  const int m_radius;
  rrt_node<PointType> *m_root;
  std::unordered_set<rrt_node<PointType>, rrt_node_hash> m_node_set;

 public:
  rrt(const std::vector<int> &state_space,
      const std::vector<obstacle *> &obstacles, const int &dim,
      const PointType &initial_point, const PointType &goal, const int &radius);
  KDtree<PointType> my_kdtree;
  PointType random_sample_2d();
  PointType random_sample_3d();
  bool obstacle_free();
  PointType steer(const rrt_node<PointType> &nearest, const PointType &sample);
  virtual void extend(const PointType &sampled_node);
};