#include "Clothoid.hh"
#include "base_car.h"
#include "boost/math/interpolators/barycentric_rational.hpp"
#include "boost/math/interpolators/cubic_b_spline.hpp"
#include "objective.h"
#include "road_rep.h"
#include "solver.h"

#include <GUnit.h>
#include <fstream>
#include <iostream>
#include <numeric>

GTEST("test_cmpc") {
  using namespace std;
  using namespace cmpc;
  using namespace G2lib;

  opt_set* opt_data = new opt_set;

  opt_data->action_dim = 2;
  opt_data->state_dim = 6;
  opt_data->state_action_dim = 8;
  opt_data->sample = 0.1;
  opt_data->horizon = 20;
  double lb[8] = {-5.0, -30.0, -500.0, -200.0, -2 * M_PI, -1.0, 0.0, 0.0};
  double ub[8] = {5.0, 30.0, 500.0, 200.0, 2 * M_PI, 1.0, 16.0, 100.0};
  vector<double> lb_vec, ub_vec;
  for (int i = 0; i < opt_data->horizon; ++i) {
    lb_vec.insert(lb_vec.end(), lb, lb + 8);
    ub_vec.insert(ub_vec.end(), ub, ub + 8);
  }
  assert(lb_vec.size() == opt_data->state_action_dim * opt_data->horizon);
  opt_data->lb = lb_vec.data();
  opt_data->ub = ub_vec.data();

  opt_data->road_lb = sqrt(2) - 2;
  opt_data->road_ub = 5 - sqrt(2);

  opt_data->width = 2.0f;
  opt_data->length = 4.0f;

  Eigen::VectorXd weights(6);
  weights << 0.1, 0.1, -1.0, 0.5, 0.1, 1.0;
  opt_data->weights = weights;  // e_c, e_lag, progress, acc, steer_v, yaw

  SHOULD("check_optimizer_setting") {
    EXPECT_EQ(opt_data->state_action_dim, 8);
    EXPECT_EQ(opt_data->lb[4], -2 * M_PI);
    EXPECT_EQ(opt_data->ub[4], 2 * M_PI);
  }

  SHOULD("cubic_spline_representation") {
    vector<float> x_way(10, 2.0f);
    vector<float> y_way{-2.0f, 2.0f,  3.0f,  4.0f,  6.0f,
                        8.0f,  10.0f, 11.0f, 12.0f, 13.0f};
    boost::math::cubic_b_spline<float> ref_x(x_way.data(), x_way.size(), 0.0f,
                                             5.0f);
    boost::math::cubic_b_spline<float> ref_y(y_way.data(), y_way.size(), 0.0f,
                                             5.0f);
    EXPECT_EQ(ref_x(0.0), 2.0f);
    EXPECT_EQ(ref_x(10.0f), 2.0f);

    EXPECT_FLOAT_EQ(ref_y(0.0f), -2.0f);
    EXPECT_FLOAT_EQ(ref_y(15.0f), 4.0f);
  }

  SHOULD("bary_line_representation") {
    int num_waypoints = 4;
    vector<double> x_way{27.0, 27.0, 17.0, -100.0};
    vector<double> y_way{-50.0, -2.0, 8.0, 8.0};
    vector<double> theta_way{M_PI_2, M_PI_2, M_PI, M_PI};

    ClothoidCurve* clothoid = new ClothoidCurve();

    int num_sample_seg = 20;
    vector<double> sample_x{x_way[0]}, sample_y{y_way[0]}, sample_s{0.0};
    std::ofstream file;
    file.open("clothoid.csv");
    for (int i = 0; i < num_waypoints - 1; ++i) {
      clothoid->build_G1(x_way[i], y_way[i], theta_way[i], x_way[i + 1],
                         y_way[i + 1], theta_way[i + 1]);

      vector<double> sample_x_i(num_sample_seg), sample_y_i(num_sample_seg);
      clothoid->pointsOnClothoid(x_way[i], y_way[i], theta_way[i],
                                 clothoid->kappaBegin(), clothoid->dkappa(),
                                 clothoid->length(), num_sample_seg, sample_x_i,
                                 sample_y_i);

      for (int z = 0; z < num_sample_seg; ++z)
        file << sample_x_i[z] << "\t" << sample_y_i[z] << "\n";

      sample_x.insert(sample_x.end(), sample_x_i.begin(), sample_x_i.end());
      sample_y.insert(sample_y.end(), sample_y_i.begin(), sample_y_i.end());

      for (int j = 0; j < num_sample_seg; ++j)
        sample_s.push_back(sample_s.back() +
                           clothoid->length() / num_sample_seg);
    }
    file.close();

    boost::math::barycentric_rational<double> ref_x(
        sample_s.data(), sample_x.data(), sample_x.size());
    boost::math::barycentric_rational<double> ref_y(
        sample_s.data(), sample_y.data(), sample_y.size());

    file.open("ref_path.csv");

    for (double s = 0; s < sample_s.back(); s += 5.0)
      file << ref_x(s) << "\t" << ref_y(s) << "\n";

    file.close();

    delete clothoid;
  }

  SHOULD("road_segment_gradient") {
    int num_waypoints = 4;
    vector<double> x_way{27.0, 27.0, 17.0, -100.0};
    vector<double> y_way{-50.0, -2.0, 8.0, 8.0};
    vector<double> theta_way{M_PI_2, M_PI_2, M_PI, M_PI};

    road_rep road;
    vector<boost::math::barycentric_rational<double>*> ref(2);
    double length;
    road.build_representation(x_way, y_way, theta_way, 20, ref, length);

    double* x = new double[8];
    x[0] = 0;
    x[1] = 0;
    x[2] = 27;
    x[3] = -45;
    x[4] = M_PI_2;
    x[5] = 0;
    x[6] = 5;
    x[7] = 5;
    double diff = obj::grad_error(x, ref[0], ref[1]);
    delete[] x;

    EXPECT_LE(diff, 0.01);
  }

  SHOULD("objective_gradient") {
    int num_waypoints = 4;
    vector<double> x_way{27.0, 27.0, 17.0, -100.0};
    vector<double> y_way{-50.0, -2.0, 8.0, 8.0};
    vector<double> theta_way{M_PI_2, M_PI_2, M_PI, M_PI};

    road_rep road;
    vector<boost::math::barycentric_rational<double>*> ref(2);
    double length;
    road.build_representation(x_way, y_way, theta_way, 20, ref, length);

    double* x = new double[8];
    x[0] = 0;
    x[1] = 0;
    x[2] = 27;
    x[3] = -45;
    x[4] = M_PI_2;
    x[5] = 0;
    x[6] = 5;
    x[7] = 5;
    double diff = obj::grad(ref[0], ref[1], x);
    delete[] x;

    EXPECT_LE(diff, 0.01);
  }

  SHOULD("cmpc_planning") {
    int num_waypoints = 4;
    vector<double> x_way{27.0, 27.0, 17.0, -100.0};
    vector<double> y_way{-50.0, -2.0, 8.0, 8.0};
    vector<double> theta_way{M_PI_2, M_PI_2, M_PI, M_PI};

    road_rep road;
    vector<boost::math::barycentric_rational<double>*> ref(2);
    double length;
    road.build_representation(x_way, y_way, theta_way, 20, ref, length);
    road.plot(ref[0], ref[1], length);

    opt_data->ref_path_x = ref[0];
    opt_data->ref_path_y = ref[1];

    opt_data->init_dis = 0;
    opt_data->init_pose = {x_way[0], y_way[0], theta_way[0]};
    opt_data->init_steer = 0;
    opt_data->init_v = 5;

    opt_data->yaw_max = 50;

    double* result = new double[opt_data->horizon * opt_data->state_action_dim];

    // initial guess
    for (int i = 0; i < opt_data->horizon; ++i) {
      // x[i * opt_data->state_action_dim + 0] = 0;
      // x[i * opt_data->state_action_dim + 1] = 0;
      result[i * opt_data->state_action_dim + 2] = opt_data->init_pose.x;
      result[i * opt_data->state_action_dim + 3] = opt_data->init_pose.y;
      result[i * opt_data->state_action_dim + 4] = opt_data->init_pose.heading;
      // x[i * opt_data->state_action_dim + 5] = opt_data->init_steer;
      // x[i * opt_data->state_action_dim + 6] = opt_data->init_v;
      result[i * opt_data->state_action_dim + 7] = opt_data->init_dis;
    }

    car* ego_car = new car(opt_data->length, opt_data->width, opt_data->sample);
    ego_car->set_initial_state(opt_data->init_pose, opt_data->init_steer,
                               opt_data->init_v, opt_data->init_dis);

    for (int i = 0; i < 100; ++i) {
      cmpc::solve(result, opt_data);

      // Update ego-veh
      ego_car->step(result[0], result[1]);
      opt_data->update(ego_car->get_state());

      ofstream f;
      f.open("traj" + to_string(i) + ".csv");

      for (int i = 0; i < opt_data->horizon; ++i) {
        int slice = i * opt_data->state_action_dim;
        f << "v_steer: " << result[slice] << " acc: " << result[slice + 1]
          << " x: " << result[slice + 2] << " y: " << result[slice + 3]
          << " theta: " << result[slice + 4]
          << " steer angle: " << result[slice + 5]
          << " v: " << result[slice + 6] << " distance: " << result[slice + 7]
          << endl;
      }

      double* tmp = new double[opt_data->horizon * opt_data->state_action_dim];
      uint16_t last_col = opt_data->state_action_dim * (opt_data->horizon - 1);

      memcpy(tmp, result + opt_data->state_action_dim,
             sizeof(double) * last_col);
      memcpy(tmp + last_col, result + last_col,
             sizeof(double) * opt_data->state_action_dim);
      delete[] result;
      result = tmp;
    }
  }
}