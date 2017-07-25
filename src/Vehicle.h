//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <cmath>
#include "constants.h"

class Vehicle {
public:
  Vehicle(state_t& start);

  state_t state_in(double t);

private:
  state_t _start_state;

  s_t _state_to_s_state();

  d_t _state_to_d_state();
};


#endif //PATH_PLANNING_VEHICLE_H
