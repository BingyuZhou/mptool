#include "constraints.h"
#include "Eigen/Core"
#include "base_car.h"

namespace cmpc {
constraint::constraint(const float& sample_t) : m_sample(sample_t){};
Eigen::VectorXf constraint::single_equality_const(car* ego_veh,
                                                  const float& steer_v,
                                                  const float& throttle) {
  ego_veh->step(steer_v, throttle, m_sample);
  return ego_veh->get_state();
}

double* constraint::single_inequality_const(){};

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