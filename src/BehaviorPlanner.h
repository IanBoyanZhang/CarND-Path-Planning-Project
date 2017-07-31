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
  BehaviorPlanner(const tk::spline spline_dx, const tk::spline spline_dy);
  // TODO: put in struct
  void setCostCoeffs(planner_cost_t plannerCost);
  void updateSensorReading(const vector<vector<double> > sensor_fusion);
  vector<vector<double> > getTargetFrenetVelocity();
  // TODO: predict
  void predict();

private:
  vector< vector< double> > _sensor_fusion;
  tk::spline _spline_dx;
  tk::spline _spline_dy;
  vector<double> _get_d_norm(const double s);
  vector<double> _get_vs_vd(const vector<double> d_norm, const double vx, const double vy);

  planner_cost_t _plannerCost;
  // Cost functions
  double _distance_from_goal_lane(vehicle_t ego, ptg_t trajectory, int lane);
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
