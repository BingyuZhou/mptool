#include "Eigen/Core"

#include <array>
#include <boost/math/interpolators/barycentric_rational.hpp>

namespace cmpc {
class obj {
 public:
  obj();

  /// step cost
  static double step_cost(
      const Eigen::VectorXd& coeff, const double& t_sample,
      const boost::math::barycentric_rational<double>* ref_path_x,
      const boost::math::barycentric_rational<double>* ref_path_y,
      const float& length, const double* x);
  /// Contour & lag error
  static void error(const double* x,
                    const boost::math::barycentric_rational<double>* ref_path_x,
                    const boost::math::barycentric_rational<double>* ref_path_y,
                    double& e_contour, double& e_lag);
};
}  // namespace cmpc