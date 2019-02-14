#include "car/base_car.h"

#include <GUnit.h>
#include <iostream>

using namespace std;
const float EPS = 1E-3;

GTEST("TEST_BASE_CAR") {
  car* obj = new car(3.0f, 1.8f);

  SHOULD("TEST_STATE_UPDATE") {
    pose p0{0.0f, 0.0f, 0.0f};
    obj->set_initial_state(p0, 0.0f, 0.0f, 0.0f);

    obj->step(0.1f, 1.0f, 1.0f);

    EXPECT_FLOAT_EQ(obj->get_v(), 1);
    EXPECT_LE(abs(obj->get_pose().heading - 0.0111), EPS);
    EXPECT_FLOAT_EQ(obj->get_acc(), 1);
  }

}
