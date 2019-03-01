#pragma once

#include "base_car.h"
#include "obs.h"

#include <array>
#include <vector>

namespace cmpc {

/// Constraints
class constraint {
  /// Equality constraints i.e. state dynamics
  static Eigen::VectorXf single_equality_const(car* ego_veh,
                                               const double& steer_v,
                                               const double& throttle,
                                               const float& sample_t);

  /// Brief Collision avoidance. Ego-veh vs one obstacle
  static Eigen::VectorXd collision_avoidance(
      const Eigen::MatrixXd& repr_ego, const double& ego_radius,
      const uint16_t& num_policies, const std::vector<pose>& obs_pred,
      const float& obs_length, const float& obs_width,
      const array<double, 2>& uncertainty);

 public:
  constraint();

  /// Brief Representation of ego-veh. Compute four circles enveloping car
  static void represent_ego(const pose& ego_veh, const float& length,
                            const float& width, Eigen::MatrixXd& repr_ego,
                            double& ego_radius);

  /// Road boundary
  static double* road_boundary(const float& e_contour, const double& road_ub,
                               const double& road_lb);

  /// Yaw regulation
  static double yaw_regulate(const double& v, const double& length,
                             const double& steer_angle, const double& yaw_max);

  /// Whole bunch of equality constraints (one step)
  static void equality_const_step(
      const uint16_t& action_dim, const uint16_t& state_dim,
      const float& length, const float& width, const pose& init_pose,
      const double& init_v, const double& init_steer, const double& init_dis,
      car* car_sim, const float& sample, const double* x, double* result);

  /// Whole bunch of inequality cinstraints (one step)
  static void collision_const_step(const uint16_t& t, const pose& ego_pose,
                                   const float& length, const float& width,
                                   const uint16_t& action_dim,
                                   const uint16_t& state_dim,
                                   const std::vector<obs*>& obstacles,
                                   Eigen::VectorXd& result);
};
};  // namespace cmpc