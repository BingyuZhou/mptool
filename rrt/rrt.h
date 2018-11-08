#include <vector>
#include "../util/obstacle_sym.h"
template <class PointType>
class rrt_node
{
public:
  rrt_node *m_parent;
  std::vector<rrt_node *> m_children;
  rrt_node();
  rrt_node(rrt_node *parent, std::vector<rrt_node *> children);
  rrt_node(rrt_node *parent, rrt_node *child);
};

template <class PointType>
class rrt : private KDtree
{
  const std::vector<int> m_state_space_boundaty;
  const std::vector<obstacle *> m_obstacles;
  const int m_dimension;

public:
  rrt(const std::vector<int> &state_space, const std::vector<obstacle *> &obstacles, const int &dim);
  PointType random_sample_2d();
  PointType random_sample_3d();
  bool obstacle_free();
  virtual void extend(rrt_node<PointType> *sampled_node);
};