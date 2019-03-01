ADD_LIBRARY(car SHARED
    base_car.h
    base_car.cc)

target_include_directories(car
    SYSTEM INTERFACE
    ${CMAKE_SOURCE_DIR}/model/car)   
target_link_libraries(car util Eigen)

set_target_properties(car PROPERTIES LINKER_LANGUAGE CXX)