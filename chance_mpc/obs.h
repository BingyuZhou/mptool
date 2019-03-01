#pragma once

#include "base_car.h"

#include <stdint.h>
#include <array>
#include <vector>

namespace cmpc {
typedef std::vector<std::vector<pose>> trajs;

class obs {
  uint16_t m_num_policies;  ///< Number of different policies(intentions)
  double m_l;               ///< Obstacle length
  double m_w;               ///< Obstacles width

 public:
  trajs pred_trajs;  ///< Trajectory prediction under different intentions

  std::array<double, 2> uncertainty;  ///< Uncertainty of obstalces along
                                      ///< longitudinal and latittude axis

  obs(const std::array<double, 2>& uncertain);

  /// Set obstacle size
  void set_size(const double& l, const double& w);

  /// Behavior prediction
  void pred();

  /// Obstacle length
  double get_l() const;

  /// Obstacle width
  double get_w() const;

  /// Number of policies
  uint16_t get_num_policies() const;
};
}  // namespace cmpc