ADD_LIBRARY(rrt_star SHARED
    rrt_star.h)

target_link_libraries(rrt_star PUBLIC util rrt)
target_include_directories(rrt_star  SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/util> $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/rrt>)

set_target_properties(rrt_star PROPERTIES LINKER_LANGUAGE CXX)