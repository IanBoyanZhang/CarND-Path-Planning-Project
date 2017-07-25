//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

struct weighted_cost_func{
  double time_diff_cost = 1;
  double s_diff_cost = 1;
  double d_diff_cost = 1;
  double efficiency_cost = 1;
  double max_jerk_cost = 1;
  double total_jerk_cost = 1;
  double collision_cost = 10;
  double buffer_cost = 1;
  double max_accel_cost = 1;
  double total_accel_cost = 1;
};

// Declaration
// https://stackoverflow.com/questions/4266914/how-does-a-const-struct-differ-from-a-struct
// const struct weigthed_cost_func WEIGHTED_COST_FUNCTIONS;
#endif //PATH_PLANNING_CONSTANTS_H