#include "Eigen/Core"

#include <boost/math/interpolators/barycentric_rational.hpp>
#include <vector>

namespace cmpc {

typedef boost::math::barycentric_rational<double> Spline;

class obj {
 public:
  obj();

  /// step cost
  static double step_cost(const Eigen::VectorXd& coeff, const double& t_sample,
                          const Spline* ref_path_x, const Spline* ref_path_y,
                          const float& length, const double* x,
                          std::vector<double>& grad);
  /// Contour & lag error
  static void error(const double* x, const Spline* ref_path_x,
                    const Spline* ref_path_y, double& e_contour, double& e_lag,
                    std::vector<double>& grad_contour,
                    std::vector<double>& grad_lag);
  /// Numerical gradient
  static double grad_error(double* x, const Spline* ref_path_x,
                           const Spline* ref_path_y);
  static double grad(const Spline* ref_path_x, const Spline* ref_path_y,
                     double* x);
};
}  // namespace cmpc