#pragma once

#include <boost/math/interpolators/barycentric_rational.hpp>

#include <vector>

class road_rep {
 public:
  road_rep();

  /// Bary line representation of road
  void build_representation(
      const std::vector<double>& x_way, const std::vector<double>& y_way,
      const std::vector<double>& theta_way, int num_sample_seg,
      std::vector<boost::math::barycentric_rational<double>*>& ref,
      double& s_max);

  /// Plot road representation
  void plot(const boost::math::barycentric_rational<double>* ref_x,
            const boost::math::barycentric_rational<double>* ref_y,
            const double& s_max);
};
