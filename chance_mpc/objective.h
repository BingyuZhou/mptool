#include "Eigen/Core"

#include <array>
#include <boost/math/interpolators/cubic_b_spline.hpp>

namespace cmpc {
class obj {
 public:
  obj();

  /// step cost
  static double step_cost(const Eigen::VectorXd& coeff, const double& t_sample,
                          const boost::math::cubic_b_spline<float>* ref_path_x,
                          const boost::math::cubic_b_spline<float>* ref_path_y,
                          const float& length, const double* x);
  /// Contour & lag error
  static void error(const double* x,
                    const boost::math::cubic_b_spline<float>* ref_path_x,
                    const boost::math::cubic_b_spline<float>* ref_path_y,
                    double& e_contour, double& e_lag);
};
}  // namespace cmpc