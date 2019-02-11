#include "base_car.h"

float car::get_l() { return m_length; }

float car::get_w() { return m_width; }

void car::set_initial_state(const pose& init_s) { m_state = init_s; }

pose car::get_state() { return m_state; }

void car::step(const float& steer_v, const float& throttle, const float& t) {
  m_acc = throttle;
  m_steer_v = steer_v;
}