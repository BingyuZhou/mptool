add_subdirectory(car)

add_library(road_rep SHARED
    road_rep.h
    road_rep.cc)

target_include_directories(road_rep
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/model/)

target_link_libraries(road_rep Clothoids)