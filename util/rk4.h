#include "Eigen/Dense"

#include <array>
#include <iostream>

/**
 * Runge-Kutta 4th order
 *
 * Solve ode: x_dot = f(x, u)
 */
void rk4(const std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd)>&
             ode_fun,
         const double& delta_t, const double& t, Eigen::VectorXd& state,
         const Eigen::VectorXd& actions) {
  for (int16_t i = t / delta_t; i > 0; --i) {
    Eigen::VectorXd k1 = delta_t * ode_fun(state, actions);
    Eigen::VectorXd k2 = delta_t * ode_fun(state + 0.5 * k1, actions);
    Eigen::VectorXd k3 = delta_t * ode_fun(state + 0.5 * k2, actions);
    Eigen::VectorXd k4 = delta_t * ode_fun(state + k3, actions);
    state += (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
  }
}