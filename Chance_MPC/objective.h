#include <boost/math/interpolators/cubic_b_spline.hpp>
#include "Eigen/Core"

namespace cmpc {
class obj {
  Eigen::VectorXf m_state;   ///< [x, y, \theta, \delta, v, s]
  Eigen::VectorXf m_action;  ///< [steer_v, acceleration]

 public:
  obj();

  /// step cost
  float step_cost(const Eigen::VectorXf& coeff, const float& t_sample);

  /// Contour & lag error
  void error(const boost::math::cubic_b_spline<float>* ref_path_x,
             const boost::math::cubic_b_spline<float>* ref_path_y,
             float& e_contour, float& e_lag);
};
}  // namespace cmpc