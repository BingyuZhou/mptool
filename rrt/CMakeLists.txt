ADD_LIBRARY(rrt SHARED
    rrt.h
    rrt.cpp)

target_link_libraries(rrt PUBLIC util)
target_include_directories(rrt PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/util>)

set_target_properties(util PROPERTIES LINKER_LANGUAGE CXX)