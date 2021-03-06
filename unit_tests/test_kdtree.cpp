#include "../util/kdtree.h"

#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <tuple>

using namespace std;
typedef array<int, 2> point_2d;

GTEST("test_kd_tree_2d") {
  KDtree<point_2d> my_kdtree;
  vector<point_2d> point_list{{2, 3}, {5, 4}, {9, 6}, {4, 7}, {8, 1}, {7, 2}};

  SHOULD("return_correct_tree_structure") {
    int depth = 0;
    my_kdtree.construct_tree(point_list);
    Node<point_2d> *root = my_kdtree.get_root();
    EXPECT_EQ(root->m_value[0], 7);
    EXPECT_EQ(root->m_value[1], 2);
    EXPECT_EQ(root->m_left->m_value[0], 5);
    EXPECT_EQ(root->m_left->m_value[1], 4);
    EXPECT_EQ(root->m_right->m_value[0], 9);
    EXPECT_EQ(root->m_right->m_value[1], 6);
    EXPECT_EQ(root->m_right->m_left->m_value[0], 8);
    EXPECT_EQ(root->m_left->m_left->m_value[0], 2);
  }

  SHOULD("return_nearest_neighbor") {
    my_kdtree.construct_tree(point_list);
    EXPECT_EQ(my_kdtree.get_root()->m_value[0], 7);
    point_2d node{6, 2};
    point_2d nearest;
    nearest = my_kdtree.nearest_neighbor(node);
    EXPECT_EQ(nearest[0], 7);
    EXPECT_EQ(nearest[1], 2);
    node = {10, 3};
    nearest = my_kdtree.nearest_neighbor(node);
    EXPECT_EQ(nearest[0], 8);
    EXPECT_EQ(nearest[1], 1);

    node = {8, 3};
    nearest = my_kdtree.nearest_neighbor(node);
    EXPECT_EQ(nearest[0], 7);
    EXPECT_EQ(nearest[1], 2);

    node = {1, 1};
    nearest = my_kdtree.nearest_neighbor(node);
    EXPECT_EQ(nearest[0], 2);
    EXPECT_EQ(nearest[1], 3);
  }

  SHOULD("add_new_node_to_kdtree") {
    my_kdtree.construct_tree(point_list);
    point_2d node{5, 1};
    my_kdtree.add_node(node);
    EXPECT_EQ(my_kdtree.get_root()->m_left->m_left->m_right->m_value[0],
              node[0]);
    node = {7, 1};
    my_kdtree.add_node(node);
    EXPECT_EQ(my_kdtree.get_root()->m_left->m_left->m_right->m_left->m_value[0],
              node[0]);
  }
  SHOULD("FIND_NEARREAST_AFTER_ADD_NODE") {
    point_list = {{2, 3}};
    my_kdtree.construct_tree(point_list);
    point_2d node{9, 7};
    my_kdtree.add_node(node);
    my_kdtree.add_node({4, 7});
    my_kdtree.add_node({5, 4});
    my_kdtree.add_node({7, 2});
    my_kdtree.add_node({8, 1});
    my_kdtree.add_node({9, 6});

    node = {10, 6};
    auto nearest = my_kdtree.nearest_neighbor(node);
    EXPECT_EQ(nearest[0], 9);
    EXPECT_EQ(nearest[1], 6);
  }
  SHOULD("FIND_ALL_NEARS") {
    my_kdtree.construct_tree(point_list);
    EXPECT_EQ(my_kdtree.get_root()->m_value[0], 7);
    point_2d node{7, 4};

    float radius = 4.0f;
    vector<point_2d> near = my_kdtree.near_radius(node, radius);

    // EXPECT_THAT(near[0], ElementsAre(7, 2));

    for_each(near.begin(), near.end(),
             [](point_2d s) { cout << s[0] << ' ' << s[1] << endl; });
  }
}