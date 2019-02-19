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
set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/ext/eigen-git-mirror/Eigen CACHE PATH "eigen include dir")
install(DIRECTORY ${EIGEN_INCLUDE_DIR} DESTINATION ${CMAKE_SOURCE_DIR})

add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIR}> 
)
install(TARGETS Eigen EXPORT my_export_set DESTINATION lib)
##########################################################
# NLOPT
##########################################################

add_subdirectory(nlopt)
