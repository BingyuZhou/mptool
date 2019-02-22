#include "constraints.h"
#include "Eigen/Core"
#include "utils.h"

namespace cmpc {
constraint::constraint(const float& sample_t) : m_sample(sample_t){};

Eigen::VectorXf constraint::single_equality_const(car* ego_veh,
                                                  const float& steer_v,
                                                  const float& throttle) {
  ego_veh->step(steer_v, throttle, m_sample);
  return ego_veh->get_state();
}

void represent_ego(const pose& ego_veh, const float& length,
                   Eigen::MatrixXd& repr_ego) {
  Eigen::Matrix2d rot = rotation2D(ego_veh.heading);

  Eigen::MatrixXd segment(2, 4);
  segment << -3.0 / 8.0 * length, -1.0 / 8.0 * length, 1.0 / 8.0 * length,
      3.0 / 8.0 * length, 0, 0, 0, 0;
  Eigen::Vector2d ego_pos(ego_veh.x, ego_veh.y);

  repr_ego = rot * segment;
  repr_ego.colwise() += ego_pos;
}

double* constraint::collision_avoidance(const Eigen::MatrixXd& repr_ego,
                                        const uint16_t& num_policies,
                                        const std::vector<pose>& obs_pred,
                                        const float& obs_length,
                                        const float& obs_width,
                                        const double* uncertainty) {
  assert(obs_pred.size() == num_policies);

  Eigen::VectorXd result;

  // for each policy
  for (int i = 0; i < num_policies; ++i) {
    // Uncertainty ellipsoid
    // (s-center)*ellip_pos*(s-center)^T<=1
    double a_semi = obs_length / sqrt(2) + m_ego_radius + uncertainty[0];
    double b_semi = obs_width / sqrt(2) + m_ego_radius + uncertainty[1];

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

    result << result, collision;
  }

  return result.data();
};

double* road_boundary(const float& e_contour, const double& road_ub,
                      const double& road_lb) {
  double result[2];
  result[0] = e_contour - road_ub;
  result[1] = road_lb - e_contour;

  return result;
}

double yaw_regulate(const car* ego_veh, const double& yaw_max) {
  return abs(ego_veh->get_v() / ego_veh->get_l() *
             tan(ego_veh->get_steer_angle())) -
         yaw_max;
}

void constraint::equality_const(unsigned m, double* result, unsigned n,
                                const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  uint16_t state_action_dim = opt_data->action_dim + opt_data->state_dim;

  car* car_sim = new car(opt_data->length, opt_data->width);

  car_sim->set_initial_state(opt_data->init_pose, opt_data->init_v,
                             opt_data->init_steer, opt_data->init_dis);

  for (int i = 0; i < opt_data->horizon; ++i) {
    auto new_state = single_equality_const(car_sim, x[i * state_action_dim],
                                           x[i * state_action_dim + 1]);
    for (int j = 0; j < opt_data->state_dim; ++j)
      result[j] =
          new_state(j) - x[i * state_action_dim + opt_data->action_dim + j];
  }
};

void constraint::inequality_const(unsigned m, double* result, unsigned n,
                                  const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  uint16_t state_action_dim = opt_data->action_dim + opt_data->state_dim;

  // Collision
  for (int t = 0; t < opt_data->horizon; ++t) {
    pose tmp_pose(x[t * state_action_dim + opt_data->action_dim],
                  x[t * state_action_dim + opt_data->action_dim + 1],
                  x[t * state_action_dim + opt_data->action_dim + 2]);
    Eigen::MatrixXd repr_ego;
    represent_ego(tmp_pose, opt_data->length, repr_ego);

    for (int i = 0; i < opt_data->num_obs; ++i) {
      obs* obstacle_tmp = opt_data->obstacles[i];

      std::vector<pose> pred_slice(obstacle_tmp->get_num_policies());

      for (int j = 0; j < obstacle_tmp->get_num_policies(); ++j)
        pred_slice[j] = obstacle_tmp->pred_trajs[j][t];

      collision_avoidance(repr_ego, obstacle_tmp->get_num_policies(),
                          pred_slice, obstacle_tmp->get_l(),
                          obstacle_tmp->get_w(), );
    }
  }
};

}  // namespace cmpc