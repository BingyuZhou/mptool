#include <iostream>

#include "Eigen/Dense"

/**
 * Runge-Kutta 4th order
 */
Eigen::VectorXf rk4(
    const std::function<Eigen::VectorXf(Eigen::VectorXf)>& ode_fun,
    const float& delta_t, const float& t, const Eigen::VectorXf& s_init) {
  Eigen::VectorXf state = s_init;
  for (int16_t i = t / delta_t; i > 0; --i) {
    Eigen::VectorXf k1 = delta_t * ode_fun(state);
    Eigen::VectorXf k2 = delta_t * ode_fun(state + 0.5 * k1);
    Eigen::VectorXf k3 = delta_t * ode_fun(state + 0.5 * k2);
    Eigen::VectorXf k4 = delta_t * ode_fun(state + k3);
    state += (k1 + 2 * k2 + 2 * k3 + k4) / 6.0f;
  }

  return state;
}