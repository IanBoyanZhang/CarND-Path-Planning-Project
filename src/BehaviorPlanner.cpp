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
 * @param
 * @return
 */
vector<vector<double> > BehaviorPlanner::getTargetFrenetVelocity(){
  // Should we use a struct here?
  vector<vector<double> > predictions;

  double car_id;
  double x, y, vx, vy, s, d;
  vector<double> v_sd;
  for (auto i = 0; i < _sensor_fusion.size(); i+=1) {
    car_id = _sensor_fusion[i][0];
    x = _sensor_fusion[i][1];
    y = _sensor_fusion[i][2];
    vx = _sensor_fusion[i][3];
    vy = _sensor_fusion[i][4];
    s = _sensor_fusion[i][5];
    d = _sensor_fusion[i][6];

    v_sd = _get_vs_vd(_get_d_norm(s), vx, vy);
    predictions.push_back({car_id, s, v_sd[0], d, v_sd[1]});
  }

  return predictions;
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

/**
 *
 * @param d_norm
 * @param vx
 * @param vy
 * @return
 */
vector<double> BehaviorPlanner::_get_vs_vd(const vector<double> d_norm,
                                           const double vx, const double vy) {
  double dx = d_norm[0];
  double dy = d_norm[1];

  // The reason for this is {dx, dy} will be interpolated from spline, dx^2 + dy^2 != 1
  double theta = atan2(dy, dx);

  double vd = vy * cos(theta) + vx * sin(theta);
  double vs = vy * sin(theta) + vx * cos(theta);

  return {vs, vd};
}

void BehaviorPlanner::predict() {

}

void BehaviorPlanner::setCostCoeffs(planner_cost_t plannerCost) {
  _plannerCost = plannerCost;
}

double BehaviorPlanner::_distance_from_goal_lane(vehicle_t ego, ptg_t trajectory, int lane) {
  // TODO: calculate distance to end goal lane
  double distance = abs(ego.d - from_lane_to_d(lane));
  distance = max(distance, 1.0);

  // Only consider d direction movement
  double time_to_goal = (double) distance/ego.vd;

  // Punish movement towards right lanes
  double multiplier = (double)(5 * lane / time_to_goal);

  return multiplier * _plannerCost.REACH_GOAL;
}
