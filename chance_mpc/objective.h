#include "Eigen/Core"

#include <boost/math/interpolators/barycentric_rational.hpp>
#include <vector>

namespace cmpc {
class obj {
 public:
  obj();

  /// step cost
  static double step_cost(
      const Eigen::VectorXd& coeff, const double& t_sample,
      const boost::math::barycentric_rational<double>* ref_path_x,
      const boost::math::barycentric_rational<double>* ref_path_y,
      const float& length, const double* x, std::vector<double>& grad);
  /// Contour & lag error
  static void error(const double* x,
                    const boost::math::barycentric_rational<double>* ref_path_x,
                    const boost::math::barycentric_rational<double>* ref_path_y,
                    double& e_contour, double& e_lag,
                    std::vector<double>& grad_contour,
                    std::vector<double>& grad_lag);
};
}  // namespace cmpc