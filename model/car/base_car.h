#pragma once
#include "Eigen/Dense"

#include <array>

struct pose {
  float x;        ///< m
  float y;        ///< m
  float heading;  ///< rad
};

class car {
  /// size
  float m_length;  ///< m
  float m_width;   ///< m

  /// Sampling time
  float m_sample;  ///< sec

  /// State [x, y, \theta, \delta, v, s]
  Eigen::VectorXf m_state;

  /// Actions [steer_v, acc]
  std::array<float, 2> m_actions;

  /// Vehicle dynamics, state:= [x, y, \theta, \delta, v, s] actions:=[steer_v,
  /// throttle]
  virtual Eigen::VectorXf dynamics(const Eigen::VectorXf& state,
                                   const std::array<float, 2>& actions);

 public:
  car(const float& l, const float& w);

  /// One step update
  void step(const float& steer_v, const float& throttle,
            const float& time_last);
  float get_l();
  float get_w();
  float get_v();
  float get_acc();
  float get_steer_v();
  pose get_pose();

  void set_initial_state(const pose& init_s, const float& v, const float& steer,
                         const float& s);
  void set_sample(const float&);
};
