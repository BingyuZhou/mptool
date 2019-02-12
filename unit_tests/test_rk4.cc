#include <GUnit/GTest-Lite.h>
#include <GUnit/GTest.h>
#include <gtest/gtest.h>
#include <iostream>
#include "../util/rk4.h"

Eigen::VectorXf ode(const Eigen::VectorXf& x) { return 3 * x; };

GTEST("test_rk4") {
  SHOULD("Integral for simple function") {
    Eigen::VectorXf x0(2);
    x0(0) = 1.0f;
    x0(1) = 2.0f;

    auto result = rk4(ode, 0.1f, 0.1f, x0);
    std::cout << result << std::endl;

    std::cout << 0.1f * ode(x0) << std::endl;
  }
}