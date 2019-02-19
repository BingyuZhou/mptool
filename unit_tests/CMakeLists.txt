set (libs GUnit util rrt rrt_star car Eigen nlopt)

ADD_GUNIT_TEST(test_kdtree.cpp  ${libs})
ADD_GUNIT_TEST(test_rrt.cpp ${libs})
ADD_GUNIT_TEST(test_rrt_star.cpp ${libs})
ADD_GUNIT_TEST(test_utils.cc ${libs})
ADD_GUNIT_TEST(test_rk4.cc ${libs})
ADD_GUNIT_TEST(test_base_car.cc ${libs})

enable_testing()
ADD_EXECUTABLE(test_mpc example_mpc.cc)
target_link_libraries(test_mpc ${libs} )
ADD_TEST(mpc test_mpc)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/rrt_star ${CMAKE_SOURCE_DIR}/model )
