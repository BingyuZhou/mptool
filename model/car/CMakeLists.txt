ADD_LIBRARY(car SHARED
    base_car.h)

target_link_libraries(car PUBLIC util)
target_include_directories(car  SYSTEM PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/util>)

set_target_properties(car PROPERTIES LINKER_LANGUAGE CXX)