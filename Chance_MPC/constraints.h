#pragma once

#include <base_car.h>

#include <vector>

namespace cmpc {

typedef std::vector<std::vector<pose>> trajs;

/// Settings for mpc
struct opt_set {
  uint16_t horizon;     ///< Prediction horizon
  uint16_t state_dim;   ///< State dimention
  uint16_t action_dim;  ///< Action dimention
  car* ego_veh;         ///< Object of ego-veh
};

/// Constraints
class constraint {
  float m_sample;       ///< sampling time
  double m_ego_radius;  ///< radius of enveloping circles

 public:
  constraint(const float& sample_t);

  /// Equality constraints i.e. state dynamics
  Eigen::VectorXf single_equality_const(car* ego_veh, const float& steer_v,
                                        const float& throttle);
  /// Brief Representation of ego-veh. Compute four circles enveloping car
  void represent_ego(const pose& ego_veh, const float& radius,
                     std::vector<double>& repr_ego);
  /// Brief Collision avoidance. Ego-veh vs one obstacle
  double* collision_avoidance(const Eigen::MatrixXd& repr_ego,
                              const uint16_t& num_policies,
                              const std::vector<pose>& obs_pred,
                              const float& obs_length, const float& obs_width,
                              const double* uncertainty);

  /// Whole bunch of equality constraints
  double* equality_const(unsigned n, const double* x, double* grad, void* data);

  /// Whole bunch of inequality cinstraints
  double* inequality_const(unsigned n, const double* x, double* grad,
                           void* data);
};
};  // namespace cmpc