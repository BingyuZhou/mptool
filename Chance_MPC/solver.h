#pragma once

#include <stdint.h>

namespace cmpc {

/// Settings for mpc
struct opt_set {
  float length;                                    ///< Car length
  float width;                                     ///< Car width
  Eigen::VectorXd weights;                         ///< Weights of objective
  boost::math::cubic_b_spline<float>* ref_path_x;  ///< Reference path (dis->x)
  boost::math::cubic_b_spline<float>* ref_path_y;  ///< Reference path (dis->y)
  std::vector<obs*> obstacles;  ///< Obstacles with prediction
  float road_lb;                ///< Road lower boundary
  float road_ub;                ///< Road upper boundary

  pose init_pose;
  double init_v;
  double init_steer;
  double init_dis;

  double yaw_max;  ///< Max yaw rate

  opt_set(){};
};

/// Chance-constraint MPC
class cmpc {
  double m_sample;        ///< sampling time
  uint16_t m_horizon;     ///< MPC Horizon
  uint16_t m_state_dim;   ///< State dimention
  uint16_t m_action_dim;  ///< Action dimention

 public:
  cmpc();
  /// Constructor with settings
  cmpc(const double& sample_t, const uint16_t& horizon,
       const uint16_t& state_dim, const uint16_t& action_dim);

  /// Objective
  double set_objective(unsigned n, const double* x, double* grad, void* data);

  /// Inequality constraints
  void set_inequality_const(unsigned m, double* result, unsigned n,
                            const double* x, double* grad, void* data);

  /// Equality constraints
  void set_equality_const(unsigned m, double* result, unsigned n,
                          const double* x, double* grad, void* data);

  /// Solve
  void solve(const double* lb, const double* ub, const opt_set* opt_data);
};
};  // namespace cmpc