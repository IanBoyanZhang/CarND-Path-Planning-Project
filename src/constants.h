//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

#include <vector>
#include <limits>
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

// Actual track length?
static double _MAX_S = 6914.14925765991;

// Threshold
static double DETECT_DIST_THRESHOLD = 100;
static double COLLIDE_THRESHOLD = 2.5;
// center difference in 1 meter, considering the two vehicles in same lane
// Overlap
static double SAME_LANE_DETECTION_THRESHOLD = 1;
static double NO_COLLISION_THRESHOLD = 1e6; //s
static double TARGET_SPEED = 49; // mph // should it be under SI?
static double DESIRED_DISTANCE_BUFFER = 6; // 6meter

// Trajectory generation
// s_dot max over a second
// static double MAX_SPEED = 24; // mps to 55 mph
static double MAX_SPEED = 23; // mps to 51.5 mph
//static double MAX_ACCEL = ?
// Max s, d accel?
// Pertubation
static double N_SAMPLES = 10;
//SIGMA_S = [10.0, 4.0, 2.0] # s, s_dot, s_double_dot
//SIGMA_D = [1.0, 1.0, 1.0]
static double SIGMA_S[3] = {10.0, 4.0, 2.0};
static double SIGMA_D[3] = {1.0, 1.0, 1.0};

// Weights
static double MOVE_TO_LEFT_LANE = 5;
static double COLLISION = 1e6;
static double DESIRED_BUFFER = 2; // timesteps
static double DANGER = 1e5;

// States definition
/*
 * "KL" - Keep Lane
 * The vehicle will attempt to drive its target speed, unless there is
 * traffic in front of it, in which case it will slow down.
 *
 * "KLF" - Keep Lane Following
 *
 * "LCL" or "LCR" - Lane Change Left / Right
 * The vehicle will IMMEDIATELY change lanes and then follow longitudinal
 * behavior for the "KL" state in the new lane.
 *
 * "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
 * The vehicle will find the nearest vehicle in the adjacent lane which is
 * BEHIND itself and will adjust speed to try to get behind that vehicle.
 */
// 0, 1, 2, 3, 4
enum State {KL, KLF, LCL, LCR, PLCL, PLCR};

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
  double x;
  double y;
  double yaw;
};

struct traj_sd_t {
  std::vector<double> s;
  std::vector<double> d;
};
#endif //PATH_PLANNING_CONSTANTS_H