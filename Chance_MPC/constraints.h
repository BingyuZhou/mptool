#pragma once

#include "base_car.h"
#include "obs.h"

#include <vector>

namespace cmpc {

/// Settings for mpc
struct opt_set {
  uint16_t horizon;     ///< Prediction horizon
  uint16_t state_dim;   ///< State dimention
  uint16_t action_dim;  ///< Action dimention

  pose init_pose;         ///< Initial pose
  float init_v;           ///< Initial velocity
  float init_steer;       ///< Initial steer angel
  float init_dis;         ///< Initial distance
  float length;           ///< Car length
  float width;            ///< Car width
  double envelop_radius;  ///< Enveloping radius of ego-veh

  uint16_t num_obs;             ///< Number of obstacles
  std::vector<obs*> obstacles;  ///< Obstacles
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
  void represent_ego(const pose& ego_veh, const float& length,
                     Eigen::MatrixXd& repr_ego);
  /// Brief Collision avoidance. Ego-veh vs one obstacle
  double* collision_avoidance(const Eigen::MatrixXd& repr_ego,
                              const uint16_t& num_policies,
                              const std::vector<pose>& obs_pred,
                              const float& obs_length, const float& obs_width,
                              const double* uncertainty);
  /// Road boundary
  double* road_boundary(const float& e_contour, const double& road_ub,
                        const double& road_lb);

  /// Whole bunch of equality constraints
  void equality_const(unsigned m, double* result, unsigned n, const double* x,
                      double* grad, void* data);

  /// Whole bunch of inequality cinstraints
  void inequality_const(unsigned m, double* result, unsigned n, const double* x,
                        double* grad, void* data);
};
};  // namespace cmpc