#include "car/base_car.h"
#include "solver.h"

#include <GUnit.h>
#include <iostream>

GTEST("test_cmpc") {
  using namespace std;
  using namespace cmpc;
  opt_set* opt_data = new opt_set;

  opt_data->action_dim = 2;
  opt_data->state_dim = 6;
  opt_data->state_action_dim = 8;
  opt_data->sample = 0.1;
  opt_data->horizon = 50;
  double lb[8] = {-5.0, -30.0, -500.0, -200.0, -2 * M_PI, -1.0, 0.3, 0.0};
  double ub[8] = {5.0, 30.0, 500.0, 200.0, 2 * M_PI, 1.0, 16.0, 800.0};

  opt_data->lb = lb;
  opt_data->ub = ub;

  opt_data->road_lb = sqrt(2) - 2;
  opt_data->road_ub = 5 - sqrt(2);

  opt_data->width = 2.0f;
  opt_data->length = 4.0f;

  Eigen::VectorXd weights(6);
  weights << 1.0, 1.0, -1.0, 0.8, 1.0, 1.0;
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

    std::ofstream file;
    file.open("ref_path.csv");

    for (double s = 0; s < 50.0; s += 1.0)
      file << ref_x(s) << "\t" << ref_y(s) << "\n";

    file.close();
  }

  SHOULD("cmpc_planning") {
    vector<float> x_way{27.0, 27.0, 17.0, -400.0};
    vector<float> y_way{-12.2, -2.0, 8.0, 8.0};
    boost::math::cubic_b_spline<float> ref_x(x_way.data(), x_way.size(), 0.0f,
                                             5.0f);
    boost::math::cubic_b_spline<float> ref_y(y_way.data(), y_way.size(), 0.0f,
                                             5.0f);
    opt_data->ref_path_x
  }
}