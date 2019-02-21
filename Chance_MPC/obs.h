#pragma once

#include <stdint.h>

class obs {
  uint16_t m_num_policies;

 public:
  obs();

  /// Behavior prediction
  void pred();

  uint16_t get_num_policies();
}