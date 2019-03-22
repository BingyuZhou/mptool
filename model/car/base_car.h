#pragma once
#include "Eigen/Dense"

#include <array>

struct pose {
  double x;        ///< m
  double y;        ///< m
  double heading;  ///< rad
};

class car {
  /// size
  float m_length;  ///< m
  float m_width;   ///< m

  /// Sampling time
  double m_sample;  ///< sec

  /// State [x, y, \theta, \delta, v, s]
  Eigen::VectorXd m_state;

  /// Actions [steer_v, acc]
  Eigen::VectorXd m_actions;

 public:
  car(const float& l, const float& w, const double& sample);

  /// One step update
  void step(const double& steer_v, const double& throttle);
  Eigen::VectorXd step(const Eigen::VectorXd& state,
                       const Eigen::VectorXd& actions);

  // void jacob_theta(const Eigen::VectorXd& state, Eigen::VectorXd& jacobin,
  //                  const double /*t*/);
  // void jacob_delta(const Eigen::VectorXd& state, Eigen::VectorXd& jacobin,
  //                  const double /*t*/);
  // void jacob_v(const Eigen::VectorXd& state, Eigen::VectorXd& jacobin,
  //              const double /*t*/);
  // void jacob_steer_v(const Eigen::VectorXd& state, Eigen::VectorXd& jacobin,
  //                    const double /*t*/);
  // void jacob_throttle(const Eigen::VectorXd& state, Eigen::VectorXd& jacobin,
  //                     const double /*t*/);

  Eigen::MatrixXd jacob(const Eigen::VectorXd& state);
  Eigen::MatrixXd jacob_numeric(const Eigen::VectorXd& state,
                                const Eigen::VectorXd& actions);
  float get_l() const;
  float get_w() const;
  double get_v() const;
  double get_acc() const;
  double get_steer_v() const;
  Eigen::VectorXd get_action() const;
  double get_steer_angle() const;
  pose get_pose() const;
  Eigen::VectorXd get_state() const;

  void set_initial_state(const pose& init_s, const double& steer,
                         const double& v, const double& s);
  void set_sample(const double&);
};
