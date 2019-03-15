ADD_LIBRARY(util STATIC
    utils.h
    kdtree.h
    obstacle_sym.h
    rk4.h
    logger)

target_include_directories(util
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/util)

target_link_libraries(util Eigen GUnit)
set_target_properties(util PROPERTIES LINKER_LANGUAGE CXX)