#pragma once

#include "Clothoid.hh"
#include "boost/math/interpolators/barycentric_rational.hpp"

#include <vector>

class road_rep {
 public:
  road_rep();

  void build_representation(const std::vector<double>& x_way,
                            const std::vector<double>& y_way,
                            const std::vector<double>& theta_way,
                            int num_sample_seg, bool record_path,
                            boost::math::barycentric_rational<double>& ref_x,
                            boost::math::barycentric_rational<double>& ref_y);
}
