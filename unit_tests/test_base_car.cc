#include "base_car.h"

#include <GUnit.h>
#include <iostream>

using namespace std;
const float EPS = 1E-3;

GTEST("TEST_BASE_CAR") {
  car* obj = new car(3.0f, 1.8f, 1);

  SHOULD("TEST_STATE_UPDATE") {
    pose p0{0.0f, 0.0f, 0.0f};
    obj->set_initial_state(p0, 0.0f, 0.0f, 0.0f);

    obj->step(0.1f, 1.0f);

    EXPECT_FLOAT_EQ(obj->get_v(), 1);
    EXPECT_LE(abs(obj->get_pose().heading - 0.0111), EPS);
    EXPECT_FLOAT_EQ(obj->get_acc(), 1);
  }

  SHOULD("TEST_JACOBIN") {
    pose p0{2.0f, 1.0f, 0.0f};
    obj->set_initial_state(p0, 5.0f, 2.0f, 0.0f);
    obj->set_sample(0.01);

    auto jacobin_num = obj->jacob_numeric(obj->get_state(), obj->get_action());
    auto jacobin = obj->jacob(obj->get_state());

    EXPECT_EQ(jacobin.size(), jacobin_num.size());
    for (int i = 0; i < jacobin.rows(); ++i) {
      for (int j = 0; j < jacobin.cols(); ++j) {
        EXPECT_LE(abs(jacobin(i, j) - jacobin_num(i, j)), EPS);
      }
    }
  }
  SHOULD("CAR_SIM") {
    std::ofstream out;
    out.open("car.txt");
    obj->set_initial_state({2.0f, 1.0f, 0.0f}, 5.0f, 2.0f, 0.0f);
    obj->set_sample(0.1);
    out << obj->get_state().transpose() << std::endl;
    for (int i = 0; i < 200; ++i) {
      obj->step(0.01, 0);
      out << obj->get_state().transpose() << std::endl;
    }
    out.close();
  }
  delete obj;
}
