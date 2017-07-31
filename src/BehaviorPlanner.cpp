//
// Created by ian zhang on 7/30/17.
//

#include "BehaviorPlanner.h"

BehaviorPlanner::BehaviorPlanner(const tk::spline spline_dx, const tk::spline spline_dy) {
  _spline_dx = spline_dx;
  _spline_dy = spline_dy;
}

void BehaviorPlanner::updateSensorReading(const vector< vector<double> > sensor_fusion) {
  _sensor_fusion = sensor_fusion;
}
/**
 * TODO: filtering car when it is too far away,
 * and within reasonable time frame
 * @return
 */
vector<double> BehaviorPlanner::predict() {
  vector<double> predictions;

  int car_id;
  double x, y, vx, vy, s, d;
  for (auto i = 0; i < _sensor_fusion.size(); i+=1) {
    car_id = _sensor_fusion[i][0];
    x = _sensor_fusion[i][1];
    y = _sensor_fusion[i][2];
    vx = _sensor_fusion[i][3];
    vy = _sensor_fusion[i][4];
    s = _sensor_fusion[i][5];
    d = _sensor_fusion[i][6];
  }
}
/**
 * Get normal vector perpendicular to the road curvature
 * @param s
 * @return
 */
vector<double> BehaviorPlanner::_get_d_norm(const double s) {
  double dx = _spline_dx(s);
  double dy = _spline_dx(s);

  return {dx, dy};
}

vector<double> BehaviorPlanner::_get_vd_vs(const vector<double> d_norm,
                                           const double vx, const double vy) {
  double dx = d_norm[0];
  double dy = d_norm[1];

  // The reason for this is {dx, dy} will be interpolated from spline, dx^2 + dy^2 != 1
  double theta = atan2(dy, dx);

  double vd = vy * cos(theta) + vx * sin(theta);
  double vs = vy * sin(theta) + vx * cos(theta);

  return {vd, vs};
}
