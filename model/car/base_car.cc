#include "base_car.h"
#include "rk4.h"

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <cmath>

using namespace boost::numeric::odeint;
using namespace Eigen;

const double EPS = 1E-4;

car::car(const float& l, const float& w, const double& sample)
    : m_length(l),
      m_width(w),
      m_state(VectorXd::Zero(6)),
      m_actions(VectorXd::Zero(2)),
      m_sample(sample) {}

void car::step(const double& steer_v, const double& throttle) {
  m_actions(0) = steer_v;
  m_actions(1) = throttle;

  std::function<VectorXd(const VectorXd&, const VectorXd&)> dynamics =
      [&](const VectorXd& state, const VectorXd& actions) {
        VectorXd x_dot(6);
        x_dot(0) = state(4) * cos(state(2));
        x_dot(1) = state(4) * sin(state(2));
        x_dot(2) = state(4) * tan(state(3)) / m_length;
        x_dot(3) = actions[0];
        x_dot(4) = actions[1];
        x_dot(5) = state(4);

        return x_dot;
      };
  rk4(dynamics, m_sample, m_sample, m_state, m_actions);
}

VectorXd car::step(const VectorXd& state, const VectorXd& actions) {
  std::function<VectorXd(const VectorXd&, const VectorXd&)> dynamics =
      [&](const VectorXd& state, const VectorXd& actions) {
        VectorXd x_dot(6);
        x_dot(0) = state(4) * cos(state(2));
        x_dot(1) = state(4) * sin(state(2));
        x_dot(2) = state(4) * tan(state(3)) / m_length;
        x_dot(3) = actions(0);
        x_dot(4) = actions(1);
        x_dot(5) = state(4);

        return x_dot;
      };
  VectorXd state_update = state;
  rk4(dynamics, m_sample, m_sample, state_update, actions);
  return state_update;
}

/// Jacobin of dynamics over action and state pairs.
/// Jacobin:=[/part f\ /part u,/part f\ /part x]
MatrixXd car::jacob(const VectorXd& state) {
  MatrixXd jacobin(state.size(), state.size() + 2);
  jacobin(0, 4) = -state(4) * sin(state(2));
  jacobin(1, 4) = state(4) * cos(state(2));
  jacobin(2, 5) = state(4) / m_length / pow(cos(state(3)), 2);
  jacobin(0, 6) = cos(state(2));
  jacobin(1, 6) = sin(state(2));
  jacobin(2, 6) = tan(state(3)) / m_length;
  jacobin(5, 6) = 1;
  jacobin(3, 0) = 1;
  jacobin(4, 1) = 1;

  jacobin *= m_sample;
  for (int i = 0; i < jacobin.rows(); ++i) {
    jacobin(i, i + 2) = 1.0;
  }

  return jacobin;
}

/// Numerical approx of jacobin
/// Jacobin:=[/part f\ /part u,/part f\ /part x]
MatrixXd car::jacob_numeric(const VectorXd& state, const VectorXd& actions) {
  MatrixXd jacobin(state.size(), state.size() + 2);
  VectorXd delta_s(VectorXd::Zero(state.size())), delta_a(VectorXd::Zero(2));
  auto central_diff = [&]() {
    VectorXd diff = (step(state + delta_s, actions + delta_a) -
                     step(state - delta_s, actions - delta_a)) /
                    (2 * EPS);
    return diff;
  };
  // /part f\ /part  u
  delta_a(0) = EPS;
  jacobin.col(0) = central_diff();
  delta_a(0) = 0;
  delta_a(1) = EPS;
  jacobin.col(1) = central_diff();
  delta_a(1) = 0;

  for (int i = 0; i < state.size(); ++i) {
    delta_s(i) = EPS;
    jacobin.col(i + 2) = central_diff();
    delta_s(i) = 0;
  }

  return jacobin;
}

float car::get_l() const { return m_length; }

float car::get_w() const { return m_width; }

double car::get_v() const { return m_state(4); }

double car::get_acc() const { return m_actions(1); }
double car::get_steer_v() const { return m_actions(0); }
VectorXd car::get_action() const { return m_actions; }

double car::get_steer_angle() const { return m_state(3); }

pose car::get_pose() const {
  pose p = {m_state(0), m_state(1), m_state(2)};
  return p;
}

VectorXd car::get_state() const { return m_state; }

void car::set_initial_state(const pose& init_s, const double& steer,
                            const double& v, const double& s) {
  m_state(0) = init_s.x;
  m_state(1) = init_s.y;
  m_state(2) = init_s.heading;
  m_state(3) = steer;
  m_state(4) = v;
  m_state(5) = s;
}

void car::set_sample(const double& sample_time) { m_sample = sample_time; }
