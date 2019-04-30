#include "constraints.h"
#include "Eigen/Core"
#include "utils.h"

using namespace std;

const uint16_t num_ego_circles = 4;

namespace cmpc {
constraint::constraint(){};

Eigen::VectorXd constraint::single_equality_const(car* ego_veh,
                                                  const double& steer_v,
                                                  const double& throttle) {
  ego_veh->step(steer_v, throttle);
  return ego_veh->get_state();
}

void constraint::represent_ego(const pose& ego_veh, const float& length,
                               const float& width, Eigen::MatrixXd& repr_ego,
                               double& ego_radius) {
  Eigen::Matrix2d rot = rotation2D(ego_veh.heading);

  Eigen::MatrixXd segment(2, 4);
  segment << -3.0 / 8.0 * length, -1.0 / 8.0 * length, 1.0 / 8.0 * length,
      3.0 / 8.0 * length, 0, 0, 0, 0;
  Eigen::Vector2d ego_pos(ego_veh.x, ego_veh.y);

  repr_ego = rot * segment;
  repr_ego.colwise() += ego_pos;

  ego_radius = sqrt(pow(length / 8.0, 2) + pow(width / 2.0, 2));
}

Eigen::VectorXd constraint::collision_avoidance(
    const Eigen::MatrixXd& repr_ego, const double& ego_radius,
    const uint16_t& num_policies, const vector<pose>& obs_pred,
    const float& obs_length, const float& obs_width,
    const array<double, 2>& uncertainty) {
  assert(obs_pred.size() == num_policies);

  Eigen::VectorXd result =
      Eigen::VectorXd::Zero(num_ego_circles * num_policies);

  // for each policy
  for (int i = 0; i < num_policies; ++i) {
    // Uncertainty ellipsoid
    // (s-center)*ellip_pos*(s-center)^T<=1
    double a_semi = obs_length / sqrt(2) + ego_radius + uncertainty[0];
    double b_semi = obs_width / sqrt(2) + ego_radius + uncertainty[1];

    Eigen::Vector2d center;
    center << obs_pred[i].x, obs_pred[i].y;

    Eigen::Matrix2d ellip_pos;
    auto rot = rotation2D(obs_pred[i].heading);
    Eigen::DiagonalMatrix<double, 2> ab(1.0 / (a_semi * a_semi),
                                        1.0 / (b_semi * b_semi));
    ellip_pos = rot * ab * rot.transpose();
    Eigen::MatrixXd term = repr_ego.colwise() - center;
    Eigen::MatrixXd collision =
        1.0 - ((term * ellip_pos).array() * term.array()).rowwise().sum();

    result.segment(i * num_ego_circles, num_ego_circles) = collision;
  }

  return result;
};

void constraint::road_boundary(const double& e_contour, const double& road_ub,
                               const double& road_lb, double* result) {
  result[0] = e_contour - road_ub;
  result[1] = road_lb - e_contour;
}

double constraint::yaw_regulate(const double& v, const double& length,
                                const double& steer_angle,
                                const double& yaw_max, vector<double>& grad) {
  grad.resize(8);
  grad[0] = 0.0;
  grad[1] = 0.0;
  grad[2] = 0.0;
  grad[3] = 0.0;
  grad[4] = 0.0;
  grad[5] = v / length / pow(cos(steer_angle), 2);
  grad[6] = tan(steer_angle) / length;
  grad[7] = 0.0;

  double result = abs(v / length * tan(steer_angle)) - yaw_max;
  if (result < 0) {
    grad[5] *= -1;
    grad[6] *= -1;
  }

  return result;
}

void constraint::equality_const_step(const uint16_t& action_dim,
                                     const uint16_t& state_dim, car* car_sim,
                                     const double* x, vector<double>& result) {
  uint16_t state_action_dim = action_dim + state_dim;

  auto new_state = single_equality_const(car_sim, x[0], x[1]);
  result.resize(state_dim);
  for (int j = 0; j < state_dim; ++j) result[j] = x[j + 2] - new_state(j);

};  // namespace cmpc

void constraint::collision_const_step(const uint16_t& t, const pose& ego_pose,
                                      const float& length, const float& width,
                                      const uint16_t& action_dim,
                                      const uint16_t& state_dim,
                                      const std::vector<obs*>& obstacles,
                                      Eigen::VectorXd& result) {
  uint16_t state_action_dim = action_dim + state_dim;

  int tail = 0;

  // Collision
  Eigen::MatrixXd repr_ego;
  double ego_radius;
  represent_ego(ego_pose, length, width, repr_ego, ego_radius);

  for (int i = 0; i < obstacles.size(); ++i) {
    obs* obstacle_tmp = obstacles[i];

    std::vector<pose> pred_slice(obstacle_tmp->get_num_policies());

    for (int j = 0; j < obstacle_tmp->get_num_policies(); ++j)
      pred_slice[j] = obstacle_tmp->pred_trajs[j][t];

    result.segment(tail, num_ego_circles * obstacle_tmp->get_num_policies()) =
        collision_avoidance(repr_ego, ego_radius,
                            obstacle_tmp->get_num_policies(), pred_slice,
                            obstacle_tmp->get_l(), obstacle_tmp->get_w(),
                            obstacle_tmp->uncertainty);

    tail += num_ego_circles * obstacle_tmp->get_num_policies();
  }
}
};  // namespace cmpc
