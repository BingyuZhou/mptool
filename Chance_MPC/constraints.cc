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
    Eigen::DiagonalMatrix<double, 2> ab(1.0 / a_semi * a_semi,
                                        1.0 / b_semi * b_semi);
    ellip_pos = rot * ab * rot.transpose();
    Eigen::MatrixXd term = repr_ego.colwise() - center;
    auto collision =
        ((term * ellip_pos).array() * term.array()).rowwise().sum();
  }
};

double* constraint::equality_const(unsigned n, const double* x, double* grad,
                                   void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  double result[opt_data->state_dim * opt_data->horizon] = {0.0};
  uint16_t state_action_dim = opt_data->action_dim + opt_data->state_dim;
  for (int i = 0; i < opt_data->horizon; ++i) {
    auto new_state =
        single_equality_const(opt_data->ego_veh, x[i * state_action_dim],
                              x[i * state_action_dim + 1]);
    for (int j = 0; j < opt_data->state_dim; ++j)
      result[j] =
          new_state(j) - x[i * state_action_dim + opt_data->action_dim + j];
  }
  return result;
};

double* constraint::inequality_const(unsigned n, const double* x, double* grad,
                                     void* data){};

}  // namespace cmpc