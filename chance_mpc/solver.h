#pragma once

/** Chance constraint MPC
 * var = [steer_vel, throttle, x, y, theta, steer_angle, v, distance]
 */

#include <boost/math/interpolators/barycentric_rational.hpp>
#include "Eigen/Core"
#include "obs.h"

#include <stdint.h>
#include <iostream>
#include <vector>

namespace cmpc {

typedef boost::math::barycentric_rational<double> Spline;

/// Settings for mpc
struct opt_set {
  double sample;        ///< sampling time
  uint16_t horizon;     ///< MPC Horizon
  uint16_t state_dim;   ///< State dimention
  uint16_t action_dim;  ///< Action dimention
  uint16_t state_action_dim;

  float length;                 ///< Car length
  float width;                  ///< Car width
  Eigen::VectorXd weights;      ///< Weights of objective
  Spline* ref_path_x;           ///< Reference path (dis->x)
  Spline* ref_path_y;           ///< Reference path (dis->y)
  std::vector<obs*> obstacles;  ///< Obstacles with prediction
  float road_lb;                ///< Road lower boundary
  float road_ub;                ///< Road upper boundary

  pose init_pose;
  double init_v;
  double init_steer;
  double init_dis;

  double yaw_max;  ///< Max yaw rate
  double* lb;      ///< steer_v, throttle, x, y, theta, delta, v, dis
  double* ub;

  void update(const Eigen::VectorXd& state) {
    init_pose.x = state(0);
    init_pose.y = state(1);
    init_pose.heading = state(2);
    init_steer = state(3);
    init_v = state(4);
    init_dis = state(5);
  }
};

/// Chance-constraint MPC
/// Objective
double set_objective(unsigned n, const double* x, double* grad, void* data);

/// Inequality constraints
void set_inequality_const(unsigned m, double* result, unsigned n,
                          const double* x, double* grad, void* data);

/// Equality constraints
void set_equality_const(unsigned m, double* result, unsigned n, const double* x,
                        double* grad, void* data);

/// Solve
void solve(double* x, const opt_set* opt_data);

};  // namespace cmpc