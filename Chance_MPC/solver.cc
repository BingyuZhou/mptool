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
        opt_data->length, x + m_state_dim + m_action_dim);
  }
  return sum_cost;
}

/// Inequality constraints
void cmpc::set_inequality_const(unsigned m, double* result, unsigned n,
                                const double* x, double* grad, void* data) {}

/// Equality constraints
void cmpc::set_equality_const(unsigned m, double* result, unsigned n,
                              const double* x, double* grad, void* data) {}

/// Solve
void cmpc::solve() {}
}  // namespace cmpc