#include "base_car.h"
#include "solver.h"

#include <GUnit.h>
#include <iostream>

using namespace std;
using namespace cmpc;

GTEST("test_cmpc") {
  cmpc::cmpc cmpc_solver;
  opt_set opt_data;
  cmpc_solver.solve();
}