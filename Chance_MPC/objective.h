#include "Eigen/Core"

#include <array>
#include <boost/math/interpolators/cubic_b_spline.hpp>

namespace cmpc {
class obj {
  float m_state[6];   ///< [x, y, \theta, \delta, v, s]
  float m_action[2];  ///< [steer_v, acceleration]

 public:
  obj();

  /// step cost
  float step_cost(const Eigen::VectorXf& coeff, const float& t_sample,
                  const boost::math::cubic_b_spline<float>* ref_path_x,
                  const boost::math::cubic_b_spline<float>* ref_path_y,
                  const float& length);

  /// Contour & lag error
  void error(const boost::math::cubic_b_spline<float>* ref_path_x,
             const boost::math::cubic_b_spline<float>* ref_path_y,
             float& e_contour, float& e_lag);
};
}  // namespace cmpc