//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <cmath>
#include <vector>
#include "constants.h"

using namespace std;

class Vehicle {
public:
  Vehicle(state_t& start);

  Vehicle(vector<double> start);

  state_t state_in(double t);

  vector<double> state_in_vec(double t);

private:
  state_t _start_state;

  vector<double> _start_state_vec;

  s_t _state_to_s_state();

  d_t _state_to_d_state();
};


#endif //PATH_PLANNING_VEHICLE_H
