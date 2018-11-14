set (libs GUnit util rrt)

ADD_GUNIT_TEST(test_kdtree.cpp  ${libs})
ADD_GUNIT_TEST(test_rrt.cpp ${libs})

enable_testing()