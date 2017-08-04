//
// Created by ian zhang on 7/17/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include <cmath>
#include <functional>
#include "spline.h"
#include "constants.h"
#include "Vehicle.h"

using namespace std;

/**
 * Lane center d magnitude in simulator
 * Lane width is 4 (meter)
 */
// What is the right coordination?
const uint8_t LANE1 = 2;
const uint8_t LANE2 = 2 + 4;
const uint8_t LANE3 = 2 + 4 + 4;

// Static is required
static double from_lane_to_d(int lane) {
  return 2 + lane * 4;
}

static double l2dist(vector<double> A, vector<double> B) {
	return sqrt(pow((A[0] - B[0]), 2) + pow((A[1], B[1]), 2));
}

class utils {

public:
  double logistic(double x);
  std::vector<double> differentiate(const vector<double>& cofficients);
//  double nearest_approach_to_any_vehicle();
  // Serializer
  vector<double> s_t_to_vec(s_t s_state);
  vector<double> d_t_to_vec(d_t d_state);

  s_t vec_to_s_t(vector<double>& s_vec);
  d_t vec_to_d_t(vector<double>& d_vec);

  vector<double> combine_states(s_t s_state, d_t d_state);

  function<double(double)> to_equation(const vector<double>& coefficients);

  /**
   * What is trajectory data structure
   * @param traj
   * @return
   */
  double nearest_appraoch(vector<double> traj, vector<Vehicle> vehicles);
  vector<function<double(double)> >
  get_f_and_N_derivatitves(const vector<double>& coeffs);

  double evalute_function(const vector<double>& coeffs, double x);
};


#endif //PATH_PLANNING_UTILS_H
