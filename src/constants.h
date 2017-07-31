//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include <vector>
struct weighted_cost_func_t{
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

// For start_s & start_d
struct s_t{
  double s;
  double s_dot;
  double s_ddot;
};

struct d_t{
  double d;
  double d_dot;
  double d_ddot;
};

struct state_t{
  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;
};

// TODO: design a better data structure for storing trajectory values
struct ptg_t {
  std::vector<double> s;
  std::vector<double> d;
  double t;
};

struct planner_cost_t {
  double COLLISION;
  double DANGER;
  double REACH_GOAL;
  double COMFORT;
  double EFFICIENCY;

  double DESIRED_BUFFER;
  double PLANNING_HORIZON;
};

// vs^2 + vd^2 = car_speed^2?
struct vehicle_t {
  double s;
  double d;
  double vs;
  double vd;
  double car_speed;
  double yaw;
};
#endif //PATH_PLANNING_CONSTANTS_H