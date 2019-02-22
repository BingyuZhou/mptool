#include "base_car.h"
#include "rk4.h"

#include <cmath>

car::car(const float& l, const float& w)
    : m_length(l),
      m_width(w),
      m_state(Eigen::VectorXf::Zero(6)),
      m_sample(0.1f) {}

void car::step(const float& steer_v, const float& throttle,
               const float& time_last) {
  m_actions[0] = steer_v;
  m_actions[1] = throttle;

  std::function<Eigen::VectorXf(const Eigen::VectorXf&,
                                const std::array<float, 2>&)>
      dynamics = [&](const Eigen::VectorXf& state,
                     const std::array<float, 2>& actions) {
        Eigen::VectorXf x_dot(6);
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

float car::get_v() const { return m_state[4]; }

float car::get_acc() const { return m_actions[1]; }
float car::get_steer_v() const { return m_actions[0]; }

float car::get_steer_angle() const { return m_state[3]; }

pose car::get_pose() const {
  pose p = {m_state[0], m_state[1], m_state[2]};
  return p;
}

Eigen::VectorXf car::get_state() { return m_state; }

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
