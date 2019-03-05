#pragma once

#include <boost/math/interpolators/barycentric_rational.hpp>
#include "Eigen/Core"
#include "obs.h"

#include <stdint.h>
#include <vector>

namespace cmpc {

/// Settings for mpc
struct opt_set {
  double sample;        ///< sampling time
  uint16_t horizon;     ///< MPC Horizon
  uint16_t state_dim;   ///< State dimention
  uint16_t action_dim;  ///< Action dimention
  uint16_t state_action_dim;

  float length;             ///< Car length
  float width;              ///< Car width
  Eigen::VectorXd weights;  ///< Weights of objective
  boost::math::barycentric_rational<double>*
      ref_path_x;  ///< Reference path (dis->x)
  boost::math::barycentric_rational<double>*
      ref_path_y;               ///< Reference path (dis->y)
  std::vector<obs*> obstacles;  ///< Obstacles with prediction
  float road_lb;                ///< Road lower boundary
  float road_ub;                ///< Road upper boundary

  pose init_pose;
  double init_v;
  double init_steer;
  double init_dis;

  double yaw_max;  ///< Max yaw rate
  double* lb;
  double* ub;
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
double* solve(const opt_set* opt_data);

};  // namespace cmpc