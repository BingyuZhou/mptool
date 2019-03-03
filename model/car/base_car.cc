#include "base_car.h"
#include "rk4.h"

#include <cmath>

car::car(const float& l, const float& w)
    : m_length(l),
      m_width(w),
      m_state(Eigen::VectorXd::Zero(6)),
      m_sample(0.1) {}

void car::step(const double& steer_v, const double& throttle,
               const double& time_last) {
  m_actions[0] = steer_v;
  m_actions[1] = throttle;

  std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                const std::array<double, 2>&)>
      dynamics = [&](const Eigen::VectorXd& state,
                     const std::array<double, 2>& actions) {
        Eigen::VectorXd x_dot(6);
        x_dot(0) = state(4) * cos(state(2));
        x_dot(1) = state(4) * sin(state(2));
        x_dot(2) = state(4) * tan(state(3)) / m_length;
        x_dot(3) = actions[0];
        x_dot(4) = actions[1];
        x_dot(5) = state(4);

        return x_dot;
      };
  rk4(dynamics, m_sample, time_last, m_state, m_actions);
}

float car::get_l() const { return m_length; }

float car::get_w() const { return m_width; }

double car::get_v() const { return m_state[4]; }

double car::get_acc() const { return m_actions[1]; }
double car::get_steer_v() const { return m_actions[0]; }

double car::get_steer_angle() const { return m_state[3]; }

pose car::get_pose() const {
  pose p = {m_state[0], m_state[1], m_state[2]};
  return p;
}

Eigen::VectorXd car::get_state() const { return m_state; }

void car::set_initial_state(const pose& init_s, const double& v,
                            const double& steer, const double& s) {
  m_state(0) = init_s.x;
  m_state(1) = init_s.y;
  m_state(2) = init_s.heading;
  m_state(3) = steer;
  m_state(4) = v;
  m_state(5) = s;
}

void car::set_sample(const double& sample_time) { m_sample = sample_time; }
