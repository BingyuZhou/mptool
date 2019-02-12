#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "../util/rk4.h"

const float EPS = 1e-4;

Eigen::VectorXf ode(const Eigen::VectorXf& x) { return 3 * x; };

GTEST("test_rk4") {
  SHOULD("Integral for simple function") {
    Eigen::VectorXf x0(2);
    x0(0) = 1.0f;
    x0(1) = 2.0f;

    auto result = rk4(ode, 0.1f, 0.1f, x0);

    Eigen::VectorXf real = x0 * exp(3 * 0.1f);
    EXPECT_LE(abs(result(0) - real(0)), EPS);
    EXPECT_LE(abs(result(1) - real(1)), EPS);
  }
}