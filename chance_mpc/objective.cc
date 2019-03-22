#include "objective.h"
#include "nlopt.h"

#include <cmath>

using namespace std;
using namespace Eigen;

namespace cmpc {
obj::obj() {}

double obj::step_cost(
    const Eigen::VectorXd& coeff, const double& t_sample,
    const boost::math::barycentric_rational<double>* ref_path_x,
    const boost::math::barycentric_rational<double>* ref_path_y,
    const float& length, const double* x) {
  VectorXd terms(6);
  obj::error(x, ref_path_x, ref_path_y, terms(0), terms(1));

  terms(0) *= terms(0);
  terms(1) *= terms(1);

  terms(2) = x[6] * t_sample;  // progress
  terms(3) = x[0] * x[0];      // action
  terms(4) = x[1] * x[1];      // action

  double theta_d = x[6] / length * tan(x[5]);
  terms(5) = theta_d * theta_d;  // yaw rate

  return coeff.dot(terms);
}

void obj::error(const double* x,
                const boost::math::barycentric_rational<double>* ref_path_x,
                const boost::math::barycentric_rational<double>* ref_path_y,
                double& e_contour, double& e_lag) {
  float s = x[7];
  float x_ref = (*ref_path_x)(s);
  float y_ref = (*ref_path_y)(s);

  float dx_ref = ref_path_x->prime(s);
  float dy_ref = ref_path_y->prime(s);

  float tangent = std::sqrt(dx_ref * dx_ref + dy_ref * dy_ref);
  float cos_longit = dx_ref / tangent;
  float sin_longit = dy_ref / tangent;

  e_contour = sin_longit * (x[2] - x_ref) - cos_longit * (x[3] - y_ref);
  e_lag = cos_longit * (x[2] - x_ref) + sin_longit * (x[3] - y_ref);
}

}  // namespace cmpc