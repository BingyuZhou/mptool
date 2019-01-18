#include "rrt.h"
/**
 * RRT*
 * asymptotically optimal path
 * */

/** ------------------------------------------
 * -------------RRT star node----------------
 * -----------------------------------------*/
template <class PointType>
class rrt_star_node : public rrt_node<PointType>
{
    float m_cost = 0.0f;

  public:
    using rrt_node<PointType>::rrt;
    void set_cost(float);
    float get_cost();
};

template <class PointType>
void rrt_star_node<PointType>::set_cost(float cost)
{
    m_cost = cost;
}

template <class PointType>
float rrt_star_node<PointType>::get_cost()
{
    return m_cost;
}

/* ------------------------------------------
 * ----------------rrt_star------------------
 * ------------------------------------------*/
template <class PointType>
class rrt_star : public rrt<PointType>
{

  public:
    using rrt<PointType>::rrt;
    using rrt<PointType>::extend;

    bool extend(const PointType &sampled_node);
};

template <class PointType>
bool rrt_star<PointType>::extend(const PointType &sample_node) {}