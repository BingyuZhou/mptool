/**
 * Balanced K-D tree
 */
#include <vector>

template <class PointType>
class Node
{
  Node *m_left, *m_right;
  PointType m_value;

public:
  Node(const PointType &value, const Node *left = NULL, const Node *right = NULL);
  ~Node();
};

template <class PointType>
class KDtree
{
  Node *m_root;

public:
  KDtree();
  ~KDtree();
  KDtree(const std::vector<PointType> &point_list);
  void build_tree(typename std::vector<PointType>::iterator &point_start, const int length, const int &depth);
  PointType nearest_neighbor(const PointType &node_new);
};