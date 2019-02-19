#pragma once

namespace cmpc {

/// Settings for mpc
struct opt_set {
  uint16_t horizon;     ///< Prediction horizon
  uint16_t state_dim;   ///< State dimention
  uint16_t action_dim;  ///< Action dimention
  car* ego_veh;         ///< Object of ego-veh
};

/// Constraints
class constraint {
  float m_sample;

 public:
  constraint(const float& sample_t);

  /// Equality constraints i.e. state dynamics
  Eigen::VectorXf single_equality_const(car* ego_veh, const float& steer_v,
                                        const float& throttle);

  /// Inequality constraints excluding boundaries
  double* single_inequality_const();

  /// Whole bunch of equality constraints
  double* equality_const(unsigned n, const double* x, double* grad, void* data);

  /// Whole bunch of inequality cinstraints
  double* inequality_const(unsigned n, const double* x, double* grad,
                           void* data);
};
};  // namespace cmpc