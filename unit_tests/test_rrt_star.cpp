#include <array>
#include "rrt_star.h"

typedef std::array<float, 2> point_2d;

int main() {
  std::vector<int> state_space{0, 10, -10, 10};
  obstacle *obs1 =
      new obstacle(std::make_pair(1.0f, 9.0f), std::make_pair(3.0f, 7.0f));
  obstacle *obs2 =
      new obstacle(std::make_pair(9.0f, 3.0f), std::make_pair(10.0f, -10.0f));
  obstacle *obs3 =
      new obstacle(std::make_pair(2.0f, 0.0f), std::make_pair(4.0f, -3.0f));
  obstacle *obs4 =
      new obstacle(std::make_pair(8.0f, 9.0f), std::make_pair(9.0f, 7.0f));

  std::vector<obstacle *> obstacles{obs1, obs2, obs3, obs4};

  point_2d initial_point{5.0f, -9.0f};
  point_2d goal{0.0f, 10.0f};
  int radius = 2;
  int dim = 2;
  rrt_star<point_2d> rrt_star_solver(state_space, obstacles, dim, initial_point,
                                     goal, radius);
  bool reached = rrt_star_solver.run(1000);

  if (reached) {
    std::vector<point_2d> p;
    rrt_star_solver.get_path(p);
    std::ofstream file;
    file.open("path_star.csv");
    std::for_each(p.begin(), p.end(), [&](point_2d point) {
      file << point[0] << " " << point[1] << "\n";
    });
    file.close();
  }
  rrt_star_solver.output_rrt_json("rrt_star");
}
