#pragma once

#include "base_car.h"

#include <stdint.h>
#include <vector>

namespace cmpc {
typedef std::vector<std::vector<pose>> trajs;

class obs {
  uint16_t m_num_policies;
  double m_l;
  double m_w;

 public:
  trajs& pred_trajs;

  obs();

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