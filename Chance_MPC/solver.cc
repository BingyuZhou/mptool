#include "solver.h"
#include "constraints.h"
#include "nlopt.h"
#include "objective.h"

using namespace std;
using namespace Eigen;

namespace cmpc {
cmpc::cmpc(){};

cmpc::cmpc(const double& sample_t, const uint16_t& horizon,
           const uint16_t& state_dim, const uint16_t& action_dim)
    : m_sample(sample_t),
      m_horizon(horizon),
      m_state_dim(state_dim),
      m_action_dim(action_dim) {}

double cmpc::set_objective(unsigned n, const double* x, double* grad,
                           void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  double sum_cost = 0;
  for (int i = 0; i < m_horizon; ++i) {
    sum_cost += obj::step_cost(
        opt_data->weights, m_sample, opt_data->ref_path_x, opt_data->ref_path_y,
        opt_data->length, x + i * (m_state_dim + m_action_dim));
  }
  return sum_cost;
}

/// Inequality constraints
void cmpc::set_inequality_const(unsigned m, double* result, unsigned n,
                                const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  int tail = 0;
  // Collision
  VectorXd col_result;
  for (int i = 0; i < m_horizon; ++i) {
    const double* current_x = x + i * (m_state_dim + m_action_dim);
    // representation of ego-veh
    pose ego_pose(current_x[2], current_x[3], current_x[4]);

    // Number of collision avoidance inequalities
    int sum_num_policies = 0;
    for (auto o : opt_data->obstacles)
      sum_num_policies += o->get_num_policies();

    col_result.resize(sum_num_policies);

    constraint::collision_const_step(i, ego_pose, opt_data->length,
                                     opt_data->width, m_action_dim, m_state_dim,
                                     opt_data->obstacles, col_result);
    memcpy(result + tail, col_result.data(), sizeof col_result);
    tail += sum_num_policies;
  }

  // Road boundary
  for (int i = 0; i < m_horizon; ++i) {
    const double* current_x = x + i * (m_state_dim + m_action_dim);
    float e_contour, e_lag;
    obj::error(current_x, opt_data->ref_path_x, opt_data->ref_path_y, e_contour,
               e_lag);

    double* road_result = constraint::road_boundary(
        e_contour, opt_data->road_ub, opt_data->road_lb);
    memcpy(result + tail, road_result, sizeof road_result);
    tail += 2;
  }

  // Yaw regulation
  for (int i = 0; i < m_horizon; ++i) {
    const double* current_x = x + i * (m_state_dim + m_action_dim);
    result[tail] = constraint::yaw_regulate(current_x[6], opt_data->length,
                                            current_x[5], opt_data->yaw_max);
    tail += 1;
  }

  // Check size of inequality consts
  assert(m == tail);
}

/// Equality constraints
void cmpc::set_equality_const(unsigned m, double* result, unsigned n,
                              const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  car* car_sim = new car(opt_data->length, opt_data->width);

  double* eq_result = new double[m_state_dim];
  for (int i = 0; i < m_horizon; ++i) {
    constraint::equality_const_step(
        m_action_dim, m_state_dim, opt_data->length, opt_data->width,
        opt_data->init_pose, opt_data->init_v, opt_data->init_steer,
        opt_data->init_dis, car_sim, m_sample,
        x + i * (m_state_dim + m_action_dim), eq_result);
    memcpy(result + i * m_state_dim, eq_result, sizeof eq_result);
  }
  delete[] eq_result;
}

/// Solve
void cmpc::solve() {}
}  // namespace cmpc