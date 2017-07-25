//
// Created by ian zhang on 7/17/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <cmath>
#include "spline.h"
#include "constants.h"

using namespace std;

/**
 * Lane center d magnitude in simulator
 * Lane width is 4 (meter)
 */
// What is the right coordination?
const uint8_t LANE1 = 2;
const uint8_t LANE2 = 2 + 4;
const uint8_t LANE3 = 2 + 4 + 4;

class utils {

public:
  double logistic(double x);
  double eval_equation(double t, s_t coefficients);
  std::vector<double> differentiate(s_t cofficients);
//  double nearest_approach_to_any_vehicle();
  double get_f_and_N_derivatives(s_t coeffs, int N);
  // Serializer
  vector<double> s_t_to_vec(s_t s_state);
  vector<double> d_t_to_vec(d_t d_state);
  vector<double> combine_states(s_t s_state, d_t d_state);
};


#endif //PATH_PLANNING_UTILS_H
