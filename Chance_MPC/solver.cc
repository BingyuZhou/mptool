#include "solver.h"
#include "constraints.h"
#include "nlopt.h"
#include "objective.h"

#include <iostream>

using namespace std;
using namespace Eigen;

const uint16_t num_ego_circles = 4;
const uint16_t num_road_bound = 2;
const uint16_t num_yaw_reg = 1;

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
  // Number of collision avoidance inequalities
  int sum_num_policies = 0;
  for (auto o : opt_data->obstacles) sum_num_policies += o->get_num_policies();
  VectorXd col_result(sum_num_policies);

  for (int i = 0; i < m_horizon; ++i) {
    const double* current_x = x + i * (m_state_dim + m_action_dim);
    // representation of ego-veh
    pose ego_pose(current_x[2], current_x[3], current_x[4]);

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
void cmpc::solve(const double* lb, const double* ub, const opt_set* opt_data) {
  // Number of optimization variables
  const int num_var = m_horizon * (m_state_dim + m_action_dim);

  // Optimizer
  nlopt_opt opt = nlopt_create(nlopt_algorithm::NLOPT_LN_COBYLA, num_var);

  // Vars boundary
  nlopt_set_lower_bounds(opt, lb);
  nlopt_set_upper_bounds(opt, ub);

  // Objective
  nlopt_set_min_objective(opt, this->set_objective, (void*)opt_data);

  // Equality constrs
  double tol_eq[m_state_dim * m_horizon] = {1e-8};
  nlopt_result r = nlopt_add_equality_mconstraint(opt, m_state_dim * m_horizon,
                                                  this->set_equality_const,
                                                  (void*)opt_data, tol_eq);
  if (r < 0) cout << r << endl;

  // Inequality constrs
  // Number of collision avoidance inequalities
  int sum_num_policies = 0;
  for (auto o : opt_data->obstacles) sum_num_policies += o->get_num_policies();
  int num_ineq = m_horizon * (sum_num_policies * num_ego_circles +
                              num_road_bound + num_yaw_reg);

  double tol_ineq[num_ineq] = {1e-3};
  r = nlopt_add_inequality_mconstraint(
      opt, num_ineq, this->set_inequality_const, (void*)opt_data, tol_ineq);
  if (r < 0) cout << r << endl;

  // Optimization
  nlopt_set_force_stop(opt, 300);

  double x[num_var] = {0.0};

  double minf;
  if (nlopt_optimize(opt, x, &minf) < 0) {
    printf("nlopt failed! %d\n", nlopt_optimize(opt, x, &minf));
  } else {
    printf("found minimum cost %g\n", minf);
  }
  nlopt_destroy(opt);
}
}  // namespace cmpc