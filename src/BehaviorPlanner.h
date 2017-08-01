//
// Created by ian zhang on 7/30/17.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include <vector>
#include <math.h>
#include "spline.h"
#include "constants.h"
#include "utils.h"
// TODO: predictions from sensor fusion

using namespace std;

class BehaviorPlanner {
public:
  BehaviorPlanner();
  void updateSplines(tk::spline spline_dx, tk::spline spline_dy,
                  tk::spline spline_x, tk::spline spline_y);
  void setCostCoeffs(planner_cost_t plannerCost);
  void updateSensorReading(const vector<vector<double> > sensor_fusion);
  vector<vector<double> > _getTargetFrenetVelocity();
  vector<vector<double> > _filter(vehicle_t ego, vector<vector<double> > predictions);
  void plan(vehicle_t ego, traj_sd_t trajectory, double t_inc, double T);

private:
  vector< vector< double> > _sensor_fusion;
  tk::spline _spline_x;
  tk::spline _spline_y;
  tk::spline _spline_dx;
  tk::spline _spline_dy;
  vector<double> _get_d_norm(const double s);
  vector<double> _get_vs_vd(const vector<double> d_norm, const double vx, const double vy);

  planner_cost_t _plannerCost;
  // Cost functions
  double _distance_from_goal_lane(vehicle_t ego, int lane);
  double _inefficiency_cost(vehicle_t ego, double target_speed);
  double _collision_cost(double time_till_collision);
  double _buffer_cost(double shortest_dist_in_movement, double shortest_time_to_min_buffer);
  double _get_buffer_dist(vehicle_t ego, vehicle_t other);
  bool _collides_with(vehicle_t ego, vehicle_t other);
  // TODO: remove usage of ego
  vector<double> _will_collide_at(traj_sd_t trajectory, double t_inc, double T,
                          vector<vector<double> > target_list);

  // TODO:
  double _calculate_cost();
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
