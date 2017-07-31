//
// Created by ian zhang on 7/30/17.
//

#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include <vector>
#include "spline.h"
#include <math.h>
// TODO: predictions from sensor fusion

using namespace std;

class BehaviorPlanner {
public:
  BehaviorPlanner();
  BehaviorPlanner(const tk::spline spline_dx, const tk::spline spline_dy);
  void updateSensorReading(const vector<vector<double> > sensor_fusion);
  vector<vector<double> > predict(double t);
private:
  vector< vector< double> > _sensor_fusion;
  tk::spline _spline_dx;
  tk::spline _spline_dy;
  vector<double> _get_d_norm(const double s);
  vector<double> _get_vs_vd(const vector<double> d_norm, const double vx, const double vy);
};


#endif //PATH_PLANNING_BEHAVIORPLANNER_H
