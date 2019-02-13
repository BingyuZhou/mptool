#include "Eigen/Dense"

#include <array>
#include <iostream>

/**
 * Runge-Kutta 4th order
 *
 * Solve ode: x_dot = f(x, u)
 */
void rk4(const std::function<Eigen::VectorXf(Eigen::VectorXf,
                                             std::array<float, 2>)>& ode_fun,
         const float& delta_t, const float& t, Eigen::VectorXf& state,
         const std::array<float, 2>& actions) {
  for (int16_t i = t / delta_t; i > 0; --i) {
    Eigen::VectorXf k1 = delta_t * ode_fun(state, actions);
    Eigen::VectorXf k2 = delta_t * ode_fun(state + 0.5 * k1, actions);
    Eigen::VectorXf k3 = delta_t * ode_fun(state + 0.5 * k2, actions);
    Eigen::VectorXf k4 = delta_t * ode_fun(state + k3, actions);
    state += (k1 + 2 * k2 + 2 * k3 + k4) / 6.0f;
  }
}