#include "base_car.h"
#include "rk4.h"

#include <cmath>

Eigen::VectorXf car::dynamics(const Eigen::VectorXf& state,
                              const std::array<float, 2>& actions) {
  Eigen::VectorXf x_dot;
  x_dot(0) = state(4) * cos(state(2));
  x_dot(1) = state(4) * sin(state(2));
  x_dot(2) = state(4) * tan(state(3));
  x_dot(3) = actions[0];
  x_dot(4) = actions[1];

  return x_dot;
}

void car::step(const float& steer_v, const float& throttle,
               const float& time_last) {
  m_actions[0] = steer_v;
  m_actions[1] = throttle;
  rk4(dynamics, m_sample, time_last, m_state, m_actions);
}

float car::get_l() { return m_length; }

float car::get_w() { return m_width; }

float car::get_v() { return m_state[4]; }

float car::get_acc() { return m_actions[1]; }
float car::get_steer_v() { return m_actions[0]; }

pose car::get_pose() {
  pose p = {m_state[0], m_state[1], m_state[2]};
  return p;
}

void car::set_initial_state(const pose& init_s, const float& v,
                            const float& steer, const float& s) {
  m_state(0) = init_s.x;
  m_state(1) = init_s.y;
  m_state(2) = init_s.heading;
  m_state(3) = steer;
  m_state(4) = v;
  m_state(5) = s;
}

void car::set_sample(const float& sample_time) { m_sample = sample_time; }