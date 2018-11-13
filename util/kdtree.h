/**
 * Balanced K-D tree
 */
#pragma once

#include <algorithm>
#include <vector>
#include "utils.h"

template <class PointType>
class Node {
 public:
  Node *m_left, *m_right, *m_parent;
  int m_distance;
  int m_split_axis;
  PointType m_value;
  Node(){};
  Node(const PointType &value, Node *left = NULL, Node *right = NULL,
       Node *parent = NULL)
      : m_value(value), m_left(left), m_right(right), m_parent(parent){};
  ~Node(){};
};

template <class PointType>
class KDtree {
  Node<PointType> *m_root;

 public:
  KDtree(){};
  ~KDtree(){};

  Node<PointType> *get_root() { return m_root; };

  void construct_tree(typename std::vector<PointType> &point_list) {
    m_root = build_tree(point_list.begin(), point_list.size(), 0);
  };

  Node<PointType> *build_tree(
      const typename std::vector<PointType>::iterator &point_start,
      const int length, int depth) {
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
    current_node->m_split_axis = axis;
    current_node->m_left = build_tree(point_start, median, ++depth);
    if (current_node->m_left) current_node->m_left->m_parent = current_node;
    current_node->m_right =
        build_tree(point_start + median + 1, length - median - 1, ++depth);
    if (current_node->m_right) current_node->m_right->m_parent = current_node;

    return current_node;
  };

  void add_node(PointType node_add) {
    int dimention = node_add.size();
    if (m_root == NULL)
      m_root = new Node<PointType>(node_add);
    else {
      Node<PointType> *node_tmp = m_root;
      while (true) {
        if (node_add[node_tmp->m_split_axis] <=
            node_tmp->m_value[node_tmp->m_split_axis]) {
          if (node_tmp->m_left == NULL) {
            node_tmp->m_left = new Node<PointType>(node_add);
            node_tmp->m_left->m_split_axis =
                (node_tmp->m_split_axis + 1) % dimention;
            break;
          } else
            node_tmp = node_tmp->m_left;
        } else {
          if (node_tmp->m_right == NULL) {
            node_tmp->m_right = new Node<PointType>(node_add);
            node_tmp->m_right->m_split_axis =
                (node_tmp->m_split_axis + 1) % dimention;
            break;
          } else
            node_tmp = node_tmp->m_right;
        }
      }
    }
  }

  PointType nearest_neighbor(const PointType &node_new) {
    // Find the nearest neighbor of the node_new in the kd tree

    PointType nearest = m_root->m_value;

    std::vector<Node<PointType> *> min_heap{m_root};
    int shortest_distance = euclidian_dis(node_new, m_root->m_value);
    m_root->m_distance = shortest_distance;

    while (!min_heap.empty()) {
      std::pop_heap(min_heap.begin(), min_heap.end(),
                    [](Node<PointType> *n1, Node<PointType> *n2) {
                      return n1->m_distance > n2->m_distance;
                    });
      Node<PointType> *current_node = min_heap.back();
      min_heap.pop_back();

      if (current_node->m_distance < shortest_distance) {
        shortest_distance = current_node->m_distance;
        nearest = current_node->m_value;
      }

      auto distance_to_parent_split_axis = [&]() {
        int dis = current_node->m_parent
                      ->m_value[current_node->m_parent->m_split_axis] -
                  node_new[current_node->m_parent->m_split_axis];
        return dis * dis;
      };
      auto is_parent_middle = [&]() {
        return (node_new[current_node->m_parent->m_split_axis] -
                current_node->m_parent
                    ->m_value[current_node->m_parent->m_split_axis]) *
                           (current_node->m_parent->m_value
                                [current_node->m_parent->m_split_axis] -
                            current_node->m_value[current_node->m_parent
                                                      ->m_split_axis]) >
                       0
                   ? true
                   : false;
      };

      // prune the useless branch
      if (current_node->m_parent &&
          shortest_distance < distance_to_parent_split_axis() &&
          is_parent_middle())
        continue;

      // hyperplane intersects with hypersphere or not
      int dis_to_hyperplane =
          current_node->m_value[current_node->m_split_axis] -
          node_new[current_node->m_split_axis];
      if (shortest_distance <=
          dis_to_hyperplane * dis_to_hyperplane)  // compare square distance
      {
        if (node_new[current_node->m_split_axis] <
            current_node->m_value[current_node->m_split_axis]) {
          if (current_node->m_left) {
            current_node->m_left->m_distance =
                euclidian_dis(node_new, current_node->m_left->m_value);
            min_heap.push_back(current_node->m_left);
            std::push_heap(min_heap.begin(), min_heap.end(),
                           [](Node<PointType> *n1, Node<PointType> *n2) {
                             return n1->m_distance < n2->m_distance;
                           });
          }
        } else {
          if (current_node->m_right) {
            current_node->m_right->m_distance =
                euclidian_dis(node_new, current_node->m_right->m_value);
            min_heap.push_back(current_node->m_right);
            std::push_heap(min_heap.begin(), min_heap.end(),
                           [](Node<PointType> *n1, Node<PointType> *n2) {
                             return n1->m_distance > n2->m_distance;
                           });
          }
        }
      } else {
        if (current_node->m_left) {
          current_node->m_left->m_distance =
              euclidian_dis(node_new, current_node->m_left->m_value);
          min_heap.push_back(current_node->m_left);
          std::push_heap(min_heap.begin(), min_heap.end(),
                         [](Node<PointType> *n1, Node<PointType> *n2) {
                           return n1->m_distance > n2->m_distance;
                         });
        }
        if (current_node->m_right) {
          current_node->m_right->m_distance =
              euclidian_dis(node_new, current_node->m_right->m_value);
          min_heap.push_back(current_node->m_right);
          std::push_heap(min_heap.begin(), min_heap.end(),
                         [](Node<PointType> *n1, Node<PointType> *n2) {
                           return n1->m_distance > n2->m_distance;
                         });
        }
      }
    }
    return nearest;
  };
};