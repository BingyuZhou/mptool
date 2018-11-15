ADD_LIBRARY(rrt SHARED
    rrt.h)

target_link_libraries(rrt PUBLIC util)
target_include_directories(rrt  SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/util>)

set_target_properties(rrt PROPERTIES LINKER_LANGUAGE CXX)