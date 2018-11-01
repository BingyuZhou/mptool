#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <tuple>
#include "../util/kdtree.h"

typedef std::array<int, 2> point_2d;

GTEST("test_kd_tree") {
  KDtree<point_2d> my_kdtree;

  SHOULD("return_correct_tree_structure") {
    std::vector<point_2d> point_list{{2, 3}, {5, 4}, {9, 6},
                                     {4, 7}, {8, 1}, {7, 2}};
    int depth = 0;
    Node<point_2d>* root =
        my_kdtree.build_tree(point_list.begin(), point_list.size(), depth);
    EXPECT_EQ(root->m_value[0], 7);
    EXPECT_EQ(root->m_value[1], 2);
    EXPECT_EQ(root->m_left->m_value[0], 5);
    EXPECT_EQ(root->m_left->m_value[1], 4);
    EXPECT_EQ(root->m_right->m_value[0], 9);
    EXPECT_EQ(root->m_right->m_value[1], 6);
    EXPECT_EQ(root->m_right->m_left->m_value[0], 8);
    EXPECT_EQ(root->m_left->m_left->m_value[0], 2);
  }
}