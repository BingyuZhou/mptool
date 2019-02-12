#include "base_car.h"
#include "rk4.h"
float car::get_l() { return m_length; }

float car::get_w() { return m_width; }

void car::set_initial_state(const pose& init_s) { m_state = init_s; }

pose car::get_state() { return m_state; }

Eigen::VectorXf car::dynamics(const Eigen::VectorXf& state) {}