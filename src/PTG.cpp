//
// Created by ian zhang on 7/25/17.
//

#include "PTG.h"

PTG::PTG() {}

/**
 * Calculates Jerk Minimizing Trajectory for start, end and T
 * @param start
 * @param end
 * @param T
 * @return
 */
vector<double> PTG::JMT(vector<double> start, vector<double> end, double T) {
  VectorXd B = VectorXd::Zero(3);

  B(0) = end[0] - start[0] + start[1] * T + start[2] * pow(T, 2) / 2.0;
  B(1) = end[1] - start[1] + 2 * start[2] * T;
  B(2) = end[2] - 2 * start[2];

  MatrixXd A = MatrixXd::Zero(3, 3);
  A << pow(T, 3),     pow(T, 4),     pow(T, 5),
       3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
       6 * T,         12* pow(T, 2), 20* pow(T, 3);

  VectorXd C = A.inverse() * B;
  vector<double> result;
  result = {start[0], start[1], .5 * start[2]};

  for (auto i = 0; i < C.size(); i+=1) {
    result.push_back(C.data()[i]);
  }
  return result;
}

