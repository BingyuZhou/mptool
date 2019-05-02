#pragma once

/** Logger
 * logger function specifically for optimization
 */
#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <string>

class logger {
 public:
  logger() {}
  ~logger() {
    m_perf.close();
    m_result.close();
    m_count = 0;
  }
  logger(const std::string &path) : m_path(path), m_freq(1) {
    m_perf.open(path + "log.txt");
    m_perf << boost::format("%10d | %10.2f | %10.2f | %10.2f") % "Step" %
                  "Cost" % "Inequality" % "Equaity"
           << std::endl;
    m_result.open(path + "result.txt");
  }
  void set_freq(int freq) { m_freq = freq; }
  void add_cost(double cost) { m_cost = cost; }
  void add_ineq(double cost) { m_ineq = cost; }
  void add_eq(double cost) { m_eq = cost; }
  void add_x(const double *x) { m_x = x; }
  void dump() {
    if (m_count % m_freq == 0) {
      m_perf << boost::format("%10d | %10.2f | %10.2f | %10.2f") % m_count %
                    m_cost % m_ineq % m_eq
             << std::endl;
      m_result << "--------------------------" << m_count
               << "-----------------------" << std::endl;
      for (int i = 0; i < m_horizon; ++i) {
        m_result << boost::format(
                        "%10.2f | %10.2f | %10.2f | %10.2f | %10.2f | %10.2f | "
                        "%10.2f | %10.2f") %
                        m_x[i * m_s_a_num] % m_x[i * m_s_a_num + 1] %
                        m_x[i * m_s_a_num + 2] % m_x[i * m_s_a_num + 3] %
                        m_x[i * m_s_a_num + 4] % m_x[i * m_s_a_num + 5] %
                        m_x[i * m_s_a_num + 6] % m_x[i * m_s_a_num + 7]
                 << std::endl;
      }
    }

    ++m_count;
  }
  void print() {
    if (m_count % m_freq == 0) {
      std::cout << "---------------------------" << std::endl;
      std::cout << "Step: " << m_count << "\n"
                << "Cost: " << m_cost << "\n"
                << "Ineq cost: " << m_ineq << "\n"
                << "Eq cost: " << m_eq << "\n"
                << "x: " << m_x[2] << "\n"
                << "y: " << m_x[3] << "\n"
                << "theta: " << m_x[4] << "\n"
                << "dis: " << m_x[7] << std::endl;
    }
  }

  void set_size(int horizon, int state_action_num) {
    m_horizon = horizon;
    m_s_a_num = state_action_num;
  }

 private:
  std::string m_path;
  std::ofstream m_perf, m_result;
  double m_cost, m_ineq, m_eq;
  const double *m_x;
  int m_count = 0;
  int m_horizon, m_s_a_num;
  int m_freq;
};
