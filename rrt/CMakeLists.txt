ADD_LIBRARY(rrt SHARED
    rrt.h)

target_include_directories(rrt
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/rrt)
target_link_libraries(rrt util)

set_target_properties(rrt PROPERTIES LINKER_LANGUAGE CXX)