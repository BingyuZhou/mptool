#include "kdtree.h"
#include <algorithm>
template <class PointType>
Node<PointType>::Node(const PointType &value, const Node *left = NULL, const Node *right = NULL) : m_value(value), m_left(left), m_right(right) {}

template <class PointType>
KDtree<PointType>::KDtree(){};

template <class PointType>
KDtree<PointType>::KDtree(const std::vector<PointType> &point_list)
{
    build_tree(point_list.begin(), point_list.size(), 0);
}

template <class PointType>
void KDtree<PointType>::build_tree(typename std::vector<PointType>::iterator &point_start, const int length, const int &depth)
{
    if (length == 0)
        return NULL;

    int dimention = std::tuple_size<PointType>::value;

    int axis = depth % dimention;

    std::sort(point_start, point_start + length, [](PointType n1, PointType n2) { return std::get<axis>(n1) < std::get<axis>(n2); }); // O(NlogN)

    int median = length / 2;

    if (depth == 0)
    {
        m_root = new Node(*(point_start + median));
        m_root->m_left = build_tree(point_start, median, ++depth);
        m_root->m_right = build_tree(point_start + median, length - median, ++depth);
    }
    else
    {
        Node *current_node = new Node(*(point_start + median));
        current_node->m_left = build_tree(point_start, median, ++depth);
        current_node->m_right = build_tree(point_start + median, length - median, ++depth);
    }
}

template <class PointType>
PointType KDtree<PointType>::nearest_neighbor(const PointType &node_new) {}