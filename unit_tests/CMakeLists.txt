set (libs GUnit util rrt rrt_star car)

ADD_GUNIT_TEST(test_kdtree.cpp  ${libs})
ADD_GUNIT_TEST(test_rrt.cpp ${libs})
ADD_GUNIT_TEST(test_rrt_star.cpp ${libs})
ADD_GUNIT_TEST(test_utils.cc ${libs})
ADD_GUNIT_TEST(test_rk4.cc ${libs})
ADD_GUNIT_TEST(test_base_car.cc ${libs})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/rrt_star ${CMAKE_SOURCE_DIR}/model )
enable_testing()