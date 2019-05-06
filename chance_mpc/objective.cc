#include "objective.h"
#include <iostream>
#include "nlopt.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

const double EPS = 0.0001;

namespace cmpc {
obj::obj() {}

double obj::step_cost(const Eigen::VectorXd& coeff, const double& t_sample,
                      const Spline* ref_path_x, const Spline* ref_path_y,
                      const float& length, const double* x,
                      vector<double>& grad) {
  VectorXd terms(6);
  vector<double> grad_contour, grad_lag;
  double e_contour, e_lag;
  obj::error(x, ref_path_x, ref_path_y, e_contour, e_lag, grad_contour,
             grad_lag);

  terms(0) = e_contour * e_contour;
  terms(1) = e_lag * e_lag;

  terms(2) = x[6] * t_sample;  // progress
  terms(3) = x[0] * x[0];      // action
  terms(4) = x[1] * x[1];      // action

  double theta_d = x[6] / length * tan(x[5]);
  terms(5) = theta_d * theta_d;  // yaw rate

  // grad
  double d_theta_d_v = tan(x[5]) / length;
  double d_theta_d_delta = x[6] / length / pow(cos(x[5]), 2);

  grad.resize(8);
  grad[0] = coeff(3) * 2 * x[0];  // \part cost / \part steer_v
  grad[1] = coeff(4) * 2 * x[1];  // \part cost / \part throttle
  grad[2] = 2 * (coeff(0) * e_contour * grad_contour[2] +
                 coeff(1) * e_lag * grad_lag[2]);
  grad[3] = 2 * (coeff(0) * e_contour * grad_contour[3] +
                 coeff(1) * e_lag * grad_lag[3]);
  grad[4] = 2 * (coeff(0) * e_contour * grad_contour[4] +
                 coeff(1) * e_lag * grad_lag[4]);
  grad[5] = 2 * coeff(5) * theta_d * d_theta_d_delta;
  grad[6] = coeff(2) * t_sample + 2 * coeff(5) * theta_d * d_theta_d_v;
  grad[7] = 2 * (coeff(0) * e_contour * grad_contour[7] +
                 coeff(1) * e_lag * grad_lag[7]);
  return coeff.dot(terms);
}

void obj::error(const double* x, const Spline* ref_path_x,
                const Spline* ref_path_y, double& e_contour, double& e_lag,
                vector<double>& grad_contour, vector<double>& grad_lag) {
  double s = x[7];
  double x_ref = (*ref_path_x)(s);
  double y_ref = (*ref_path_y)(s);

  double dx_ref = ref_path_x->prime(s);
  double ddx_ref =
      (ref_path_x->prime(s + EPS) - ref_path_x->prime(s - EPS)) / (2 * EPS);
  double dy_ref = ref_path_y->prime(s);
  double ddy_ref =
      (ref_path_y->prime(s + EPS) - ref_path_y->prime(s - EPS)) / (2 * EPS);

  double tangent = sqrt(dx_ref * dx_ref + dy_ref * dy_ref);
  double cos_longit = dx_ref / tangent;
  double sin_longit = dy_ref / tangent;

  e_contour = sin_longit * (x[2] - x_ref) - cos_longit * (x[3] - y_ref);
  e_lag = cos_longit * (x[2] - x_ref) + sin_longit * (x[3] - y_ref);

  // grad
  double term1 = pow(dx_ref * dx_ref + dy_ref * dy_ref, -1.5);
  double term2 = pow(dx_ref * dx_ref + dy_ref * dy_ref, -0.5);

  double d_sin_longit = -dx_ref * dy_ref * term1 * ddx_ref +
                        (term2 - dy_ref * dy_ref * term1) * ddy_ref;
  double d_cos_longit = -dx_ref * dy_ref * term1 * ddy_ref +
                        (term2 - dx_ref * dx_ref * term1) * ddx_ref;
  grad_contour.resize(8);
  grad_contour[0] = 0.0;
  grad_contour[1] = 0.0;
  grad_contour[2] = sin_longit;
  grad_contour[3] = -cos_longit;
  grad_contour[4] = 0.0;
  grad_contour[5] = 0.0;
  grad_contour[6] = 0.0;
  grad_contour[7] = d_sin_longit * (x[2] - x_ref) - sin_longit * dx_ref -
                    d_cos_longit * (x[3] - y_ref) + cos_longit * dy_ref;

  grad_lag.resize(8);
  grad_lag[0] = 0.0;
  grad_lag[1] = 0.0;
  grad_lag[2] = cos_longit;
  grad_lag[3] = sin_longit;
  grad_lag[4] = 0.0;
  grad_lag[5] = 0.0;
  grad_lag[6] = 0.0;
  grad_lag[7] = d_cos_longit * (x[2] - x_ref) - cos_longit * dx_ref +
                d_sin_longit * (x[3] - y_ref) - sin_longit * dy_ref;
}

double obj::grad_error(double* x, const Spline* ref_path_x,
                       const Spline* ref_path_y) {
  double e_cont, e_lag;
  vector<double> grad_cont, grad_lag;
  obj::error(x, ref_path_x, ref_path_y, e_cont, e_lag, grad_cont, grad_lag);

  // NUmerical gradient
  vector<double> grad_cont_num(8), grad_lag_num(8);
  vector<double> grad1, grad2;
  double e1, e2;

  for (int i = 0; i < 8; ++i) {
    x[i] += EPS;
    obj::error(x, ref_path_x, ref_path_y, e1, e2, grad1, grad2);
    x[i] -= 2 * EPS;
    obj::error(x, ref_path_x, ref_path_y, e_cont, e_lag, grad1, grad2);

    grad_cont_num[i] = (e1 - e_cont) / (2 * EPS);
    grad_lag_num[i] = (e2 - e_lag) / (2 * EPS);
    x[i] += EPS;
  }

  cout << "Numerical gradient: " << endl;
  for (int i = 0; i < 8; ++i) {
    cout << grad_cont_num[i] << " ";
  }
  cout << endl;
  for (int i = 0; i < 8; ++i) {
    cout << grad_lag_num[i] << " ";
  }
  cout << endl;

  cout << "Analytical gradient: " << endl;
  for (int i = 0; i < 8; ++i) {
    cout << grad_cont[i] << " ";
  }
  cout << endl;
  for (int i = 0; i < 8; ++i) {
    cout << grad_lag[i] << " ";
  }
  cout << endl;
  double diff_cont = 0, diff_lag = 0;
  for (int i = 0; i < 8; ++i) {
    diff_cont += abs(grad_cont_num[i] - grad_cont[i]);
    diff_lag += abs(grad_lag_num[i] - grad_lag[i]);
  }
  cout << "Difference: "
       << " contour: " << diff_cont << " lag: " << diff_lag << endl;

  return diff_cont + diff_lag;
}

double obj::grad(const Spline* ref_path_x, const Spline* ref_path_y,
                 double* x) {
  Eigen::VectorXd coeff(6);
  coeff << 1.0, 1.0, -1.0, 0.8, 1.0, 1.0;
  double t = 0.01;
  float length = 4.0f;
  vector<double> grad;
  obj::step_cost(coeff, t, ref_path_x, ref_path_y, length, x, grad);

  // NUmerical gradient
  vector<double> grad_num(8);
  vector<double> g;
  for (int i = 0; i < 8; ++i) {
    x[i] += EPS;
    double c1 = obj::step_cost(coeff, t, ref_path_x, ref_path_y, length, x, g);
    x[i] -= 2 * EPS;
    double c2 = obj::step_cost(coeff, t, ref_path_x, ref_path_y, length, x, g);

    grad_num[i] = (c1 - c2) / (2 * EPS);
    x[i] += EPS;
  }

  double diff = 0;
  for (int i = 0; i < 8; ++i) {
    diff += abs(grad[i] - grad_num[i]);
  }

  cout << "Difference: " << diff << endl;

  cout << "Numerical:" << endl;
  for_each(grad_num.begin(), grad_num.end(),
           [](double g) { cout << g << endl; });
  cout << "Analytic:" << endl;
  for_each(grad.begin(), grad.end(), [](double g) { cout << g << endl; });

  return diff;
}

}  // namespace cmpc