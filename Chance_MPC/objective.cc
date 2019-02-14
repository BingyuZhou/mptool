#include "objective.h"

using namespace std;
using namespace Eigen;

namespace cmpc {
obj::obj() {}

float obj::step_cost(const Eigen::VectorXf& coeff, const float& t_sample) {}

void obj::error(const boost::math::cubic_b_spline<float>* ref_path_x,
                const boost::math::cubic_b_spline<float>* ref_path_y,
                float& e_contour, float& e_lag) {
  float s = m_state(5);
  float x_ref = (*ref_path_x)(s);
  float y_ref = (*ref_path_y)(s);

  float dx_ref = ref_path_x->prime(s);
  float dy_ref = ref_path_y->prime(s);

  float tangent = std::sqrt(dx_ref * dx_ref + dy_ref * dy_ref);
  float cos_longit = dx_ref / tangent;
  float sin_longit = dy_ref / tangent;

  e_contour =
      sin_longit * (m_state(0) - x_ref) - cos_longit * (m_state(1) - y_ref);
  e_lag = cos_longit * (m_state(0) - x_ref) + sin_longit * (m_state(1) - x_ref);
}

}  // namespace cmpc