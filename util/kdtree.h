/**
 * Balanced K-D tree
 */
#include <assert.h>
#include <algorithm>
#include <vector>

template <class PointType>
class Node {
 public:
  Node *m_left, *m_right;
  int distance;
  PointType m_value;
  Node(){};
  Node(const PointType &value, Node *left = NULL, Node *right = NULL)
      : m_value(value), m_left(left), m_right(right){};
  ~Node(){};
};

template <class PointType>
class KDtree {
  Node<PointType> *m_root;

 public:
  KDtree(){};
  ~KDtree(){};
  KDtree(const std::vector<PointType> &point_list) {
    build_tree(point_list.begin(), point_list.size(), 0);
  };

  void construct_tree(const typename std::vector<PointType> &point_list) {
    m_root = build_tree(point_list.begin(), point_list.size(), 0);
  };

  Node<PointType> *build_tree(
      const typename std::vector<PointType>::iterator &point_start,
      const int length, int &depth) {
    if (length == 0) return NULL;

    int dimention = (*point_start).size();

    int axis = depth % dimention;

    std::sort(point_start, point_start + length,
              [&](const PointType &n1, const PointType &n2) {
                return n1[axis] < n2[axis];
              });  // O(NlogN)

    int median = length / 2;

    Node<PointType> *current_node =
        new Node<PointType>(*(point_start + median));
    current_node->m_left = build_tree(point_start, median, ++depth);
    current_node->m_right =
        build_tree(point_start + median + 1, length - median - 1, ++depth);

    return current_node;
  };

  PointType nearest_neighbor(const PointType &node_new) {
    // Find the nearest neighbor of the node_new in the kd tree

    PointType nearest = m_root->m_value;
    auto calc_distance =
        [](const PointType &n1, const PointType &n2) {
          assert(n1.size() == n2.size());
          int dis = 0;

          for (int i = n1.size(); i >= 0; --i) {
            dis += pow(n1[i] - n2[i]);
          }
          return dis;
        }

    std::vector<Node *>
        min_heap{m_root};
    int shortest_distance = calc_distance(node_new, m_root->m_value);

    while (!min_heap.empty()) {
      std::pop_heap(min_heap.begin(), min_heap.end(), [](Node *n1, Node *n2) {
        return n1->distance < n2.distance;
      });
      Node *current_node = min_heap.back();
      min_heap.pop_back();

      int distance = calc_distance(node_new, current_node->m_value);
      if (distance < shortest_distance) {
        shortest_distance = distance;
        nearest = current_node->m_value;
      }
    }
  };
};