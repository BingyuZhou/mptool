#pragma once
#include <array>

struct pose {
  float x;
  float y;
  float heading;
};

class car {
  /// size
  float m_length;
  float m_width;

  /// Observable states
  pose m_state;

  /// acceleration
  float m_acc;

  /// velocity
  float m_vel;
  float m_steer_v;

 public:
  car(const float& l, const float& w) : m_length(l), m_width(w){};

  /// State update via vehicle dynamics
  virtual void step(const float& steer_v, const float& throttle,
                    const float& t);
  float get_l();
  float get_w();
  float get_v();
  pose get_state();
  void set_initial_state(const pose&);
}
