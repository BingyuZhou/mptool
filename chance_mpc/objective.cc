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
    const float& length, const double* x, vector<double>& grad) {
  VectorXd terms(6);
  vector<double> grad_contour, grad_lag;
  obj::error(x, ref_path_x, ref_path_y, terms(0), terms(1), grad_contour,
             grad_lag);

  terms(0) *= terms(0);
  terms(1) *= terms(1);

  terms(2) = x[6] * t_sample;  // progress
  terms(3) = x[0] * x[0];      // action
  terms(4) = x[1] * x[1];      // action

  double theta_d = x[6] / length * tan(x[5]);
  terms(5) = theta_d * theta_d;  // yaw rate

  double cost = coeff.dot(terms);
  // grad
  double d_theta_d_v = tan(x[5]) / length;
  double d_theta_d_delta = x[6] / length / pow(cos(x[5]), 2);

  grad.resize(8);
  grad[0] = coeff(3) * 2 * x[0];  // \part cost / \part steer_v
  grad[1] = coeff(4) * 2 * x[1];  // \part cost / \part throttle
  grad[2] = 2 * (coeff(0) * terms(0) * grad_lag[2] +
                 coeff(1) * terms(1) * grad_contour[2]);
  grad[3] = 2 * (coeff(0) * terms(0) * grad_lag[3] +
                 coeff(1) * terms(1) * grad_contour[3]);
  grad[4] = 2 * (coeff(0) * terms(0) * grad_lag[4] +
                 coeff(1) * terms(1) * grad_contour[4]);
  grad[5] = 2 * coeff(5) * theta_d * d_theta_d_delta;
  grad[6] = coeff(2) * t_sample + 2 * coeff(5) * theta_d * d_theta_d_v;
  grad[7] = 0.0;
  return coeff.dot(terms);
}

void obj::error(const double* x,
                const boost::math::barycentric_rational<double>* ref_path_x,
                const boost::math::barycentric_rational<double>* ref_path_y,
                double& e_contour, double& e_lag, vector<double>& grad_contour,
                vector<double>& grad_lag) {
  double s = x[7];
  double x_ref = (*ref_path_x)(s);
  double y_ref = (*ref_path_y)(s);

  double dx_ref = ref_path_x->prime(s);
  double dy_ref = ref_path_y->prime(s);

  double tangent = std::sqrt(dx_ref * dx_ref + dy_ref * dy_ref);
  double cos_longit = dx_ref / tangent;
  double sin_longit = dy_ref / tangent;

  e_contour = sin_longit * (x[2] - x_ref) - cos_longit * (x[3] - y_ref);
  e_lag = cos_longit * (x[2] - x_ref) + sin_longit * (x[3] - y_ref);

  // grad
  grad_contour.resize(8);
  grad_contour[0] = 0.0;
  grad_contour[1] = 0.0;
  grad_contour[2] = sin(x[4]);
  grad_contour[3] = -cos(x[4]);
  grad_contour[4] = cos(x[4]) * (x[2] - x_ref) + sin(x[4]) * (x[3] - y_ref);
  grad_contour[5] = 0.0;
  grad_contour[6] = 0.0;
  grad_contour[7] = 0.0;

  grad_lag.resize(8);
  grad_lag[0] = 0.0;
  grad_lag[1] = 0.0;
  grad_lag[2] = cos(x[4]);
  grad_lag[3] = sin(x[4]);
  grad_lag[4] = -sin(x[4]) * (x[2] - x_ref) + cos(x[4]) * (x[3] - y_ref);
  grad_lag[5] = 0.0;
  grad_lag[6] = 0.0;
  grad_lag[7] = 0.0;
}

}  // namespace cmpc