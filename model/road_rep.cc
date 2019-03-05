#include "road_rep.h"

#include <ostream>

using namespace std;

road_rep::road_rep() {}

void road_rep::build_representation(
    const vector<double>& x_way, const vector<double>& y_way,
    const vector<double>& theta_way, int num_sample_seg, bool record_path,
    boost::math::barycentric_rational<double>& ref_x,
    boost::math::barycentric_rational<double>& ref_y) {
  int num_waypoints = x_way.size();
  G2lib::ClothoidCurve* clothoid = new G2lib::ClothoidCurve();

  vector<double> sample_x{x_way[0]}, sample_y{y_way[0]}, sample_s{0.0};

  if (record_path) {
    ofstream file;
    file.open("clothoid.csv");
  }

  for (int i = 0; i < num_waypoints - 1; ++i) {
    clothoid->build_G1(x_way[i], y_way[i], theta_way[i], x_way[i + 1],
                       y_way[i + 1], theta_way[i + 1]);

    vector<double> sample_x_i(num_sample_seg), sample_y_i(num_sample_seg);
    clothoid->pointsOnClothoid(x_way[i], y_way[i], theta_way[i],
                               clothoid->kappaBegin(), clothoid->dkappa(),
                               clothoid->length(), num_sample_seg, sample_x_i,
                               sample_y_i);
    if (record_path) {
      for (int z = 0; z < num_sample_seg; ++z)
        file << sample_x_i[z] << "\t" << sample_y_i[z] << "\n";
    }

    sample_x.insert(sample_x.end(), sample_x_i.begin(), sample_x_i.end());
    sample_y.insert(sample_y.end(), sample_y_i.begin(), sample_y_i.end());

    for (int j = 0; j < num_sample_seg; ++j)
      sample_s.push_back(sample_s.back() + clothoid->length() / num_sample_seg);
  }
  if (record_path) file.close();

  ref_x(sample_s.data(), sample_x.data(), sample_x.size());
  ref_y(sample_s.data(), sample_y.data(), sample_y.size());
}