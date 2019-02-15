##########################################################
# GUnit
##########################################################

ADD_SUBDIRECTORY(GUnit/libs/googletest)

ADD_LIBRARY(GUnit INTERFACE)

TARGET_INCLUDE_DIRECTORIES(GUnit
    SYSTEM INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GUnit/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GUnit/libs/googletest/googletest/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GUnit/libs/googletest/googlemock/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GUnit/libs/json/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/GUnit/libs/gherkin-cpp/include>
    )
TARGET_LINK_LIBRARIES(GUnit INTERFACE gtest_main gmock_main)

##########################################################
# Eigen3
##########################################################

INCLUDE_DIRECTORIES($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ext/eigen-git-mirror/Eigen/>)

##########################################################
# NLOPT
##########################################################

ADD_SUBDIRECTORY(nlopt)
