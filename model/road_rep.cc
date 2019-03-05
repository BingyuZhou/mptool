#include "road_rep.h"
#include "Clothoid.hh"

#include <fstream>

using namespace std;

road_rep::road_rep() {}

void road_rep::build_representation(
    const vector<double>& x_way, const vector<double>& y_way,
    const vector<double>& theta_way, int num_sample_seg,
    vector<boost::math::barycentric_rational<double>*>& ref, double& s_max) {
  int num_waypoints = x_way.size();
  G2lib::ClothoidCurve* clothoid = new G2lib::ClothoidCurve();

  vector<double> sample_x{x_way[0]}, sample_y{y_way[0]}, sample_s{0.0};

  for (int i = 0; i < num_waypoints - 1; ++i) {
    clothoid->build_G1(x_way[i], y_way[i], theta_way[i], x_way[i + 1],
                       y_way[i + 1], theta_way[i + 1]);

    vector<double> sample_x_i(num_sample_seg), sample_y_i(num_sample_seg);
    clothoid->pointsOnClothoid(x_way[i], y_way[i], theta_way[i],
                               clothoid->kappaBegin(), clothoid->dkappa(),
                               clothoid->length(), num_sample_seg, sample_x_i,
                               sample_y_i);

    sample_x.insert(sample_x.end(), sample_x_i.begin(), sample_x_i.end());
    sample_y.insert(sample_y.end(), sample_y_i.begin(), sample_y_i.end());

    for (int j = 0; j < num_sample_seg; ++j)
      sample_s.push_back(sample_s.back() + clothoid->length() / num_sample_seg);
  }

  auto ref_x = new boost::math::barycentric_rational<double>(
      sample_s.data(), sample_x.data(), sample_x.size());
  auto ref_y = new boost::math::barycentric_rational<double>(
      sample_s.data(), sample_y.data(), sample_y.size());
  ref[0] = ref_x;
  ref[1] = ref_y;
  s_max = sample_s.back();
}

void road_rep::plot(const boost::math::barycentric_rational<double>* ref_x,
                    const boost::math::barycentric_rational<double>* ref_y,
                    const double& s_max) {
  std::ofstream file;
  file.open("cmpc_path.csv");

  for (double s = 0; s < s_max; s += s_max / 80)
    file << (*ref_x)(s) << "\t" << (*ref_y)(s) << "\n";

  file.close();
}