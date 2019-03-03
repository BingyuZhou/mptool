#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "rk4.h"

const float EPS = 1e-4;

Eigen::VectorXd ode(const Eigen::VectorXd& x,
                    const std::array<double, 2> actions) {
  return 3 * x;
};

GTEST("test_rk4") {
  SHOULD("Integral for simple function") {
    Eigen::VectorXd x0(2);
    x0(0) = 1.0;
    x0(1) = 2.0;

    Eigen::VectorXd real = x0 * exp(3 * 0.1);

    std::array<double, 2> actions{0.0, 0.0};

    rk4(ode, 0.1, 0.1, x0, actions);

    EXPECT_LE(abs(x0(0) - real(0)), EPS);
    EXPECT_LE(abs(x0(1) - real(1)), EPS);
  }
}