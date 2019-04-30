#include "solver.h"
#include "constraints.h"
#include "logger.h"
#include "nlopt.h"
#include "objective.h"

#include <iostream>

using namespace std;
using namespace Eigen;

const uint16_t num_ego_circles = 4;
const uint16_t num_road_bound = 2;
const uint16_t num_yaw_reg = 1;

logger logging("/tmp/");

namespace cmpc {

double set_objective(unsigned n, const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  double sum_cost = 0;
  vector<double> grad_step;
  for (int i = 0; i < opt_data->horizon; ++i) {
    sum_cost += obj::step_cost(opt_data->weights, opt_data->sample,
                               opt_data->ref_path_x, opt_data->ref_path_y,
                               opt_data->length,
                               x + i * opt_data->state_action_dim, grad_step);
    if (grad)
      memcpy(grad + i * opt_data->state_action_dim, grad_step.data(),
             sizeof(double) * grad_step.size());
  }

  // logging
  logging.add_cost(sum_cost);
  logging.add_x(x);
  logging.dump();
  logging.print();
  return sum_cost;
}

/// Inequality constraints
void set_inequality_const(unsigned m, double* result, unsigned n,
                          const double* x, double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  int tail = 0;
  // Collision
  // Number of collision avoidance inequalities
  int sum_num_policies = 0;
  for (auto o : opt_data->obstacles) sum_num_policies += o->get_num_policies();
  VectorXd col_result(sum_num_policies);

  if (sum_num_policies > 0) {
    for (int i = 0; i < opt_data->horizon; ++i) {
      const double* current_x = x + i * opt_data->state_action_dim;
      // representation of ego-veh
      pose ego_pose = {current_x[2], current_x[3], current_x[4]};

      constraint::collision_const_step(
          i, ego_pose, opt_data->length, opt_data->width, opt_data->action_dim,
          opt_data->state_dim, opt_data->obstacles, col_result);
      memcpy(result + tail, col_result.data(), sizeof col_result);
      tail += sum_num_policies;
    }
  }

  // Road boundary
  for (int i = 0; i < opt_data->horizon; ++i) {
    const double* current_x = x + i * opt_data->state_action_dim;
    double e_contour, e_lag;
    vector<double> grad_contour, grad_lag;
    obj::error(current_x, opt_data->ref_path_x, opt_data->ref_path_y, e_contour,
               e_lag, grad_contour, grad_lag);

    // grad
    if (grad) {
      memcpy(grad + tail * n + i * opt_data->state_action_dim,
             grad_contour.data(), sizeof(double) * grad_contour.size());
    }

    double road_result[2];
    constraint::road_boundary(e_contour, opt_data->road_ub, opt_data->road_lb,
                              road_result);
    memcpy(result + tail, road_result, sizeof road_result);
    tail += 2;
  }

  // Yaw regulation
  for (int i = 0; i < opt_data->horizon; ++i) {
    const double* current_x = x + i * opt_data->state_action_dim;
    vector<double> grad_yaw;
    result[tail] =
        constraint::yaw_regulate(current_x[6], opt_data->length, current_x[5],
                                 opt_data->yaw_max, grad_yaw);
    if (grad) {
      memcpy(grad + tail * n + i * opt_data->state_action_dim, grad_yaw.data(),
             sizeof(double) * grad_yaw.size());
    }
    tail += 1;
  }

  // log cost
  double cost = 0;
  for (int i = 0; i < tail; ++i) {
    if (result[i] > 0) cost += result[i];
  }
  logging.add_ineq(cost);

  // Check size of inequality consts
  assert(m == tail);
}

/// Equality constraints
void set_equality_const(unsigned m, double* result, unsigned n, const double* x,
                        double* grad, void* data) {
  opt_set* opt_data = reinterpret_cast<opt_set*>(data);
  car* car_sim = new car(opt_data->length, opt_data->width, opt_data->sample);
  car_sim->set_initial_state(opt_data->init_pose, opt_data->init_steer,
                             opt_data->init_v, opt_data->init_dis);

  vector<double> eq_result;
  for (int i = 0; i < opt_data->horizon; ++i) {
    constraint::equality_const_step(opt_data->action_dim, opt_data->state_dim,
                                    car_sim, x + i * opt_data->state_action_dim,
                                    eq_result);
    memcpy(result + i * opt_data->state_dim, eq_result.data(),
           sizeof(double) * eq_result.size());
    // update gradient
    // gradient range: state_dim * x.size()
    // gradient order: \part c_i / \part x_j := grad[i*n+j]
    if (grad) {
      if (i > 0) {
        const VectorXd x0 = Map<const VectorXd>(
            x + (i - 1) * opt_data->state_action_dim + opt_data->action_dim,
            opt_data->state_dim);
        MatrixXd jacob = car_sim->jacob(x0);
        for (int j = 0; j < jacob.rows(); ++j) {
          double* slice = grad + i * n * opt_data->state_dim + j * n +
                          (i - 1) * opt_data->state_action_dim;
          memcpy(slice, jacob.row(j).data(), sizeof(double) * jacob.cols());
        }
      }

      for (int j = 0; j < opt_data->state_dim; ++j)
        grad[i * n + opt_data->action_dim + j * (opt_data->state_dim + 1)] =
            -1.0;
    }
  }
  // log cost
  double cost = 0;
  for (int i = 0; i < opt_data->horizon * opt_data->state_dim; ++i) {
    cost += abs(result[i]);
  }
  logging.add_eq(cost);

  delete car_sim;
}

/// Solve mpc
double* solve(const opt_set* opt_data) {
  logging.set_size(opt_data->horizon, opt_data->state_action_dim);

  // Number of optimization variables
  const int num_var = opt_data->horizon * opt_data->state_action_dim;

  // Optimizer
  nlopt_opt opt = nlopt_create(nlopt_algorithm::NLOPT_LD_SLSQP, num_var);
  // nlopt_opt local_opt = nlopt_create(nlopt_algorithm::NLOPT_LN_COBYLA,
  // num_var);

  // Vars boundary
  auto lr = nlopt_set_lower_bounds(opt, opt_data->lb);
  auto ur = nlopt_set_upper_bounds(opt, opt_data->ub);

  // Objective
  nlopt_set_min_objective(opt, set_objective, (void*)opt_data);

  // Equality constrs
  vector<double> tol_eq(opt_data->state_dim * opt_data->horizon, 1e-3);
  nlopt_result r = nlopt_add_equality_mconstraint(
      opt, opt_data->state_dim * opt_data->horizon, set_equality_const,
      (void*)opt_data, tol_eq.data());
  if (r < 0) cout << "add equality" << r << endl;

  // Inequality constrs
  // Number of collision avoidance inequalities
  int sum_num_policies = 0;
  for (auto o : opt_data->obstacles) sum_num_policies += o->get_num_policies();
  int num_ineq = opt_data->horizon * (sum_num_policies * num_ego_circles +
                                      num_road_bound + num_yaw_reg);

  vector<double> tol_ineq(num_ineq, 1e-3);
  r = nlopt_add_inequality_mconstraint(opt, num_ineq, set_inequality_const,
                                       (void*)opt_data, tol_ineq.data());
  if (r < 0) cout << "add inequality" << r << endl;

  // Optimization
  // nlopt_set_maxeval(opt, 600);
  nlopt_set_ftol_rel(opt, 1e-2);
  nlopt_set_xtol_rel(opt, 1e-1);

  // nlopt_set_ftol_rel(local_opt, 1e-1);
  // nlopt_set_xtol_rel(local_opt, 1);

  // nlopt_set_local_optimizer(opt, local_opt);

  // Initial guess
  double* x = new double[num_var];

  for (int i = 0; i < opt_data->horizon; ++i) {
    x[i * opt_data->state_action_dim + 2] = opt_data->init_pose.x;
    x[i * opt_data->state_action_dim + 3] = opt_data->init_pose.y;
    x[i * opt_data->state_action_dim + 4] = opt_data->init_pose.heading;
  }

  double minf;
  if (nlopt_optimize(opt, x, &minf) < 0) {
    printf("nlopt failed! %d\n", nlopt_optimize(opt, x, &minf));
  } else {
    printf("found minimum cost %g\n", minf);
  }
  nlopt_destroy(opt);
  logging.~logger();

  return x;
}
}  // namespace cmpc