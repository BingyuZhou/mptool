ADD_LIBRARY(util STATIC
    utils.h
    kdtree.h
    obstacle_sym.h)

set_target_properties(util PROPERTIES LINKER_LANGUAGE CXX)