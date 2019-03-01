#include "obs.h"

using namespace std;
using namespace Eigen;

namespace cmpc {
obs::obs(const std::array<double, 2>& uncertain) : uncertainty(uncertain){};

void obs::set_size(const double& l, const double& w) {
  m_l = l;
  m_w = w;
};

void obs::pred(){};

double obs::get_l() const { return m_l; }

double obs::get_w() const { return m_w; }

uint16_t obs::get_num_policies() const { return m_num_policies; }

}  // namespace cmpc