#include "../rrt_star/rrt_star.h"
#include <array>

typedef std::array<float, 2> point_2d;

int main()
{
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
    int radius = 1;
    int dim = 2;
    rrt_star<point_2d> rrt_solver(state_space, obstacles, dim, initial_point, goal,
                                  radius);
}