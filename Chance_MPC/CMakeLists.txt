ADD_LIBRARY(cmpc SHARED
solver.h
solver.cc
objective.h
objective.cc
constraints.h
constraints.cc)

TARGET_LINK_LIBRARIES(CMPC car)