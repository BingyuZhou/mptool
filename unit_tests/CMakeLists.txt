set (libs GUnit util rrt rrt_star)

ADD_GUNIT_TEST(test_kdtree.cpp  ${libs})
ADD_GUNIT_TEST(test_rrt.cpp ${libs})
ADD_GUNIT_TEST(test_rrt_star.cpp ${libs})
ADD_GUNIT_TEST(test_utils.cc ${libs})
ADD_GUNIT_TEST(test_rk4.cc ${libs})

enable_testing()