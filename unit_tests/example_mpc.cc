#include "Eigen/Core"
#include "nlopt.h"

#include <GUnit.h>
#include <iostream>
using namespace std;

const double EPS = 1E-2;
/**
 * Objective
 * @param n number of optimized variables
 * @param x optimized variables
 * @param grad gradient of obj function
 * @param f_data other data, eg. coeff
 */
double obj(unsigned n, const double* x, double* grad, void* f_data) {
  double cost = 0;
  for (int i = 0; i < n; i = i + 3) {
    cost += x[i] * x[i] + x[i + 1] * x[i + 1] + 0.5 * x[i + 2] * x[i + 2];
  }
  return cost;
};

/**
 * Dynamic (Equality constraint)
 *
 * @param m number of constraint
 * @param result H(x)
 * @param n number of variables
 * @param x variables
 * @param grad gradient
 * @param f_data initial state
 */
void dynamics(unsigned m, double* result, unsigned n, const double* x,
              double* grad, void* f_data) {
  Eigen::Vector2d state;
  double* x0 = reinterpret_cast<double*>(f_data);
  state(0) = x0[0];
  state(1) = x0[1];
  Eigen::Matrix2d A;
  A << 2, -1, 1, 0.2;
  Eigen::Vector2d B;
  B << 1, 0;

  int j = 0;
  for (int i = 0; i < n; i = i + 3) {
    state = A * state + B * x[i + 2];
    result[j] = state(0) - x[i];
    result[j + 1] = state(1) - x[i + 1];
    j = j + 2;
  }
};

GTEST("nlopt_example") {
  const unsigned int horizon = 7;
  const unsigned int num_state = 2;
  const unsigned int num_action = 1;
  const unsigned int num_param = horizon * (num_action + num_state);

  nlopt_opt opt = nlopt_create(nlopt_algorithm::NLOPT_LN_COBYLA, num_param);

  double lb[num_param];
  for (int i = 0; i < num_param; i = i + num_state + num_action) {
    lb[i] = -5.0;
    lb[i + 1] = -5.0;
    lb[i + 2] = -1.0;
  }
  nlopt_set_lower_bounds(opt, lb);

  double ub[num_param];
  for (int i = 0; i < num_param; i = i + 3) {
    ub[i] = 5.0;
    ub[i + 1] = 5.0;
    ub[i + 2] = 1.0;
  }
  nlopt_set_upper_bounds(opt, ub);

  nlopt_set_min_objective(opt, obj, nullptr);

  double init[2] = {3.0, 1.0};
  double tol[num_action * horizon] = {1e-8};
  nlopt_result r = nlopt_add_equality_mconstraint(opt, 6, dynamics, init, tol);
  if (r < 0) cout << r << endl;

  nlopt_set_xtol_rel(opt, 1e-4);

  double x[num_param] = {0.0};

  double minf;
  if (nlopt_optimize(opt, x, &minf) < 0) {
    printf("nlopt failed! %d\n", nlopt_optimize(opt, x, &minf));
  } else {
    printf("found minimum cost %g actions %g %g %g %g %g %g %g\n", minf, x[2],
           x[5], x[8], x[11], x[14], x[17], x[20]);
    for (int i = 0; i < 21; i = i + 3) {
      cout << x[i] << " " << x[i + 1] << endl;
    }
  }
  nlopt_destroy(opt);

  EXPECT_LE(abs(x[18]), EPS);
  EXPECT_LE(abs(x[19]), EPS);
}