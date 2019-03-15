#pragma once

/** Logger
 * logger function specifically for optimization
 */
#include <fstream>
#include <iostream>
#include <string>

class logger {
 public:
  logger() {}
  ~logger() { m_out.close(); }
  logger(const std::string &path) : m_path(path) {
    m_perf.open(path + "log.txt");
    m_perf << "Step\t"
           << "Cost\t"
           << "Inequality violation\t"
           << "Equality violation\t"
           << "State\t"
           << "Action\n";
    m_result.open(path + "result.txt");
  }

  void add_cost(double cost) { m_cost = cost; }
  void add_ineq(double cost) { m_ineq = cost; }
  void add_eq(double cost) { m_eq = cost; }
  void add_x(const double *x) { m_x = x; }
  void dump() {
    m_perf << m_count << "\t" << m_cost << "\t" << m_ineq << "\t" << m_eq
           << std::endl;
    ++m_count;
    for (int i; i < m_horizon; ++i) {
      m_result << m_x[i] << "\t" << m_x[i + 1] << "\t" << m_x[i + 2] << "\t"
               << m_x[i + 3] << "\t" << m_x[i + 4] << "\t" << m_x[i + 5] << "\t"
               << m_x[i + 6] << "\t" << m_x[i + 7] << std::endl;
    }
    m_result << "--------------------------" << std::endl;
  }
  void print() {
    std::cout << "---------------------------" << std::endl;
    std::cout << "Step: " << m_count << "\n"
              << "Cost: " << m_cost << "\n"
              << "Ineq cost: " << m_ineq << "\n"
              << "Eq cost: " << m_eq << "\n"
              << "x: " << m_x[2] << "\n"
              << "y: " << m_x[3] << "\n"
              << "theta: " << m_x[4] << std::endl;
  }

 private:
  std::string m_path;
  std::ofstream m_perf, m_result;
  double m_cost, m_ineq, m_eq;
  const double *m_x;
  int m_count = 0;
  int m_horizon;
};
