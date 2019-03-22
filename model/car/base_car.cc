#include "base_car.h"
#include "rk4.h"

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <cmath>

using namespace boost::numeric::odeint;

const double EPS = 1E-4;

car::car(const float& l, const float& w, const double& sample)
    : m_length(l),
      m_width(w),
      m_state(Eigen::VectorXd::Zero(6)),
      m_actions(Eigen::VectorXd::Zero(2)),
      m_sample(sample) {}

void car::step(const double& steer_v, const double& throttle) {
  m_actions(0) = steer_v;
  m_actions(1) = throttle;

  std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>
      dynamics =
          [&](const Eigen::VectorXd& state, const Eigen::VectorXd& actions) {
            Eigen::VectorXd x_dot(6);
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

Eigen::VectorXd car::step(const Eigen::VectorXd& state,
                          const Eigen::VectorXd& actions) {
  std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>
      dynamics =
          [&](const Eigen::VectorXd& state, const Eigen::VectorXd& actions) {
            Eigen::VectorXd x_dot(6);
            x_dot(0) = state(4) * cos(state(2));
            x_dot(1) = state(4) * sin(state(2));
            x_dot(2) = state(4) * tan(state(3)) / m_length;
            x_dot(3) = actions(0);
            x_dot(4) = actions(1);
            x_dot(5) = state(4);

            return x_dot;
          };
  Eigen::VectorXd state_update = state;
  rk4(dynamics, m_sample, m_sample, state_update, actions);
  return state_update;
}

/// Jacobin of dynamics over state and action pairs
Eigen::MatrixXd car::jacob(const Eigen::VectorXd& state) {
  Eigen::MatrixXd jacobin(state.size(), state.size() + 2);
  jacobin(0, 2) = -state(4) * sin(state(2));
  jacobin(1, 2) = state(4) * cos(state(2));
  jacobin(2, 3) = state(4) / m_length / pow(cos(state(3)), 2);
  jacobin(0, 4) = cos(state(2));
  jacobin(1, 4) = sin(state(2));
  jacobin(2, 4) = tan(state(3)) / m_length;
  jacobin(5, 4) = 1;
  jacobin(3, 6) = 1;
  jacobin(4, 7) = 1;

  jacobin *= m_sample;
  for (int i = 0; i < jacobin.rows(); ++i) {
    jacobin(i, i) = 1.0;
  }

  return jacobin;
}

Eigen::MatrixXd car::jacob_numeric(const Eigen::VectorXd& state,
                                   const Eigen::VectorXd& actions) {
  Eigen::MatrixXd jacobin(state.size(), state.size() + 2);
  Eigen::VectorXd delta_s(Eigen::VectorXd::Zero(state.size())),
      delta_a(Eigen::VectorXd::Zero(2));
  auto central_diff = [&]() {
    Eigen::VectorXd diff = (step(state + delta_s, actions + delta_a) -
                            step(state - delta_s, actions - delta_a)) /
                           (2 * EPS);
    return diff;
  };
  for (int i = 0; i < state.size(); ++i) {
    delta_s(i) = EPS;
    jacobin.col(i) = central_diff();
    delta_s(i) = 0;
  }

  delta_a(0) = EPS;
  jacobin.col(6) = central_diff();
  delta_a(0) = 0;
  delta_a(1) = EPS;
  jacobin.col(7) = central_diff();
  return jacobin;
}

float car::get_l() const { return m_length; }

float car::get_w() const { return m_width; }

double car::get_v() const { return m_state(4); }

double car::get_acc() const { return m_actions(1); }
double car::get_steer_v() const { return m_actions(0); }
Eigen::VectorXd car::get_action() const { return m_actions; }

double car::get_steer_angle() const { return m_state(3); }

pose car::get_pose() const {
  pose p = {m_state(0), m_state(1), m_state(2)};
  return p;
}

Eigen::VectorXd car::get_state() const { return m_state; }

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
