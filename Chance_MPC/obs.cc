#include "obs.h"

using namespace std;
using namespace Eigen;

namespace cmpc {
obs::obs() {}

void obs::set_size(const double& l, const double& w) {
  m_l = l;
  m_w = w;
};

void obs::pred(trajs& pred_trajs){};

double obs::get_l() const { return m_l; }

double obs::get_w() const { return m_w; }

uint16_t obs::get_num_policies() const { return m_num_policies; }

}  // namespace cmpc