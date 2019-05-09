#include "base_car.h"

#include <GUnit.h>
#include <Eigen/Core>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <iostream>

using namespace std;
using namespace Eigen;
const float EPS = 1E-3;
void dynamics(const VectorXd& state, VectorXd& x_dot, const double t) {
  VectorXd x_dot(6);
  x_dot(0) = state(4) * cos(state(2));        // d_x = v cos(theta)
  x_dot(1) = state(4) * sin(state(2));        // d_y = v sin(theta)
  x_dot(2) = state(4) * tan(state(3)) / 4.0;  // d_theta = v tan(delta) / L
  x_dot(3) = 0.01;                            // d_delta = u_delta
  x_dot(4) = 0.5;                             // d_v = u_v
  x_dot(5) = state(4);                        // d_s = v
};

GTEST("TEST_BASE_CAR") {
  car* obj = new car(3.0f, 1.8f, 1);

  SHOULD("TEST_STATE_UPDATE") {
    pose p0{0.0f, 0.0f, 0.0f};
    obj->set_initial_state(p0, 0.0f, 0.0f, 0.0f);

    obj->step(0.1f, 1.0f);

    EXPECT_FLOAT_EQ(obj->get_v(), 1);
    EXPECT_LE(abs(obj->get_pose().heading - 0.0111), EPS);
    EXPECT_LE(abs(obj->get_steer_angle() - 0.1), EPS);
    EXPECT_FLOAT_EQ(obj->get_acc(), 1);
  }

  SHOULD("TEST_JACOBIN") {
    pose p0{2.0f, 1.0f, 1.0f};
    obj->set_initial_state(p0, 1.0f, 2.0f, 5.0f);
    obj->set_sample(0.1);

    auto jacobin_num = obj->jacob_numeric(obj->get_state(), obj->get_action());
    auto jacobin = obj->jacob(obj->get_state());

    EXPECT_EQ(jacobin.size(), jacobin_num.size());
    for (int i = 0; i < jacobin.rows(); ++i) {
      for (int j = 0; j < jacobin.cols(); ++j) {
        cout << i << j << " " << jacobin(i, j) << " " << jacobin_num(i, j)
             << endl;
        // EXPECT_LE(abs(jacobin(i, j) - jacobin_num(i, j)), EPS);
      }
    }
  }
  SHOULD("CAR_SIM") {
    std::ofstream out;
    out.open("car.txt");
    obj->set_initial_state({2.0f, 1.0f, 0.0f}, 5.0f, 2.0f, 0.0f);
    obj->set_sample(0.1);
    out << obj->get_state().transpose() << std::endl;
    for (int i = 0; i < 100; ++i) {
      obj->step(0.01, 0.5);
      out << obj->get_state().transpose() << std::endl;
    }
    out.close();

    boost::numeric::odeint::runge_kutta4<VectorXd> stepper;
    VectorXd s;
    s << 2.0, 1.0, 0.0, 5.0, 2.0, 0.0;
    for (int i = 0; i < 100; ++i) {
      stepper.do_step(dynamics, s, 0.1, 0.1);
    }
  }
  delete obj;
}
