#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include "rk4.h"

const float EPS = 1e-4;
using namespace boost::numeric::odeint;

Eigen::VectorXd ode(const Eigen::VectorXd& x, const Eigen::VectorXd& actions) {
  return 3 * x;
};

Eigen::VectorXd lorenz(const Eigen::VectorXd& x,
                       const Eigen::VectorXd& actions) {
  Eigen::VectorXd dxdt(3);
  double sigma = 10.0, R = 28.0, b = 8 / 3.0;
  dxdt(0) = sigma * (x(1) - x(0));
  dxdt(1) = R * x(0) - x(1) - x(0) * x(2);
  dxdt(2) = -b * x(2) + x(0) * x(1);
  return dxdt;
}

void lorenz2(const Eigen::VectorXd& x, Eigen::VectorXd& dxdt,
             const double /*t*/) {
  double sigma = 10.0, R = 28.0, b = 8 / 3.0;
  dxdt.resize(3);
  dxdt(0) = sigma * (x(1) - x(0));
  dxdt(1) = R * x(0) - x(1) - x(0) * x(2);
  dxdt(2) = -b * x(2) + x(0) * x(1);
}

GTEST("test_rk4") {
  SHOULD("Integral for simple function") {
    Eigen::VectorXd x0(2);
    x0(0) = 1.0;
    x0(1) = 2.0;

    Eigen::VectorXd real = x0 * exp(3 * 0.1);

    Eigen::VectorXd actions(2);

    rk4(ode, 0.1, 0.1, x0, actions);

    EXPECT_LE(abs(x0(0) - real(0)), EPS);
    EXPECT_LE(abs(x0(1) - real(1)), EPS);
  }
  SHOULD("Lorenz system") {
    std::ofstream out;
    out.open("lorenz.txt");

    Eigen::VectorXd x0(3);
    x0 << 10.0, 1.0, 1.0;
    Eigen::VectorXd actions(2);
    for (int i = 0; i < 10000; ++i) {
      rk4(lorenz, 0.01, 0.01, x0, actions);
      out << x0.transpose() << std::endl;
    }
    out.close();

    runge_kutta4<Eigen::VectorXd, double, Eigen::VectorXd, double,
                 vector_space_algebra>
        rk;
    x0 << 10.0, 1.0, 1.0;
    out.open("lorenz2.txt");
    for (int i = 0; i < 10000; ++i) {
      rk.do_step(lorenz2, x0, 0.01 * i, 0.01);
      out << x0.transpose() << std::endl;
    }
    out.close();
  }
}