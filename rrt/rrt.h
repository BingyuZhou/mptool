#include <vector>
#include "../util/kdtree.h"

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
public:
  PointType random_sample();
  bool obstacle_free();
  virtual void extend(rrt_node<PointType> *sampled_node);
};