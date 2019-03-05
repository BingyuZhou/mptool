set (libs GUnit util rrt rrt_star car Eigen nlopt cmpc Clothoids)

ADD_GUNIT_TEST(test_kdtree.cpp  ${libs})
ADD_GUNIT_TEST(test_rrt.cpp ${libs})
ADD_GUNIT_TEST(test_rrt_star.cpp ${libs})
ADD_GUNIT_TEST(test_utils.cc ${libs})
ADD_GUNIT_TEST(test_rk4.cc ${libs})
ADD_GUNIT_TEST(test_base_car.cc ${libs})
ADD_GUNIT_TEST(example_mpc.cc ${libs})
ADD_GUNIT_TEST(test_cmpc.cc ${libs})

enable_testing()

