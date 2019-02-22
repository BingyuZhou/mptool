#pragma once
#include "Eigen/Dense"

#include <array>

struct pose {
  float x;        ///< m
  float y;        ///< m
  float heading;  ///< rad

  pose(const float& x_i, const float& y_i, const float& heading_i)
      : x(x_i), y(y_i), heading(heading_i) {}
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

 public:
  car(const float& l, const float& w);

  /// One step update
  void step(const float& steer_v, const float& throttle,
            const float& time_last);
  float get_l() const;
  float get_w() const;
  float get_v() const;
  float get_acc() const;
  float get_steer_v() const;
  float get_steer_angle() const;
  pose get_pose() const;
  Eigen::VectorXf get_state();

  void set_initial_state(const pose& init_s, const float& v, const float& steer,
                         const float& s);
  void set_sample(const float&);
};
