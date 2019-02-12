ADD_LIBRARY(util STATIC
    utils.h
    kdtree.h
    obstacle_sym.h
    rk4.h)

target_include_directories(util SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ext/eigen-git-mirror>)
set_target_properties(util PROPERTIES LINKER_LANGUAGE CXX)