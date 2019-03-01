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

TARGET_LINK_LIBRARIES(cmpc car Eigen nlopt)