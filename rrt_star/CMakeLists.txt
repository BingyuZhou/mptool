ADD_LIBRARY(rrt_star SHARED
    rrt_star.h)

target_include_directories(rrt_star
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/rrt_star)

target_link_libraries(rrt_star PUBLIC util rrt)

set_target_properties(rrt_star PROPERTIES LINKER_LANGUAGE CXX)