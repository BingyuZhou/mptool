ADD_LIBRARY(cmpc SHARED
solver.h
solver.cc
objective.h
objective.cc
constraints.h
constraints.cc
obs.h
obs.cc
)

target_include_directories(cmpc
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/chance_mpc/
)

TARGET_LINK_LIBRARIES(cmpc car Eigen nlopt)