//
// Created by ian zhang on 7/30/17.
//

#include "BehaviorPlanner.h"

void BehaviorPlanner::updateSplines(tk::spline spline_dx, tk::spline spline_dy,
                                    tk::spline spline_x, tk::spline spline_y) {
  _spline_dx = spline_dx;
  _spline_dy = spline_dy;
  _spline_x = spline_x;
  _spline_y = spline_y;
}

void BehaviorPlanner::updateSensorReading(const vector< vector<double> > sensor_fusion) {
  _sensor_fusion = sensor_fusion;
}

void BehaviorPlanner::setCostCoeffs(planner_cost_t plannerCost) {
  _plannerCost = plannerCost;
}

/**
 * TODO: filtering car when it is too far away,
 * and within reasonable time frame
 * @param
 * @return
 */
vector<vector<double> > BehaviorPlanner::_getTargetFrenetVelocity(){
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
    predictions.push_back({car_id, s, v_sd[0], d, v_sd[1], x, y});
  }

  return predictions;
}

/**
 * Return all target vehicles location snapshot in (s, d) at t
 * @param ego
 * @param t
 * @return
 */
vector<vector<double> > BehaviorPlanner::_filter(vehicle_t ego,
                                                 vector<vector<double> > predictions) {
  vector<vector<double> > filtered_target_list;

  // Filtering prediction list
  for (auto i = 0; i < predictions.size(); i+=1) {
    // On the RHS lanes
    if (predictions[i][3] < 0) {
      continue;
    }

    // Distance > xx skip
    if (l2dist({ego.x, ego.y}, {predictions[i][5], predictions[i][6]})
        > DETECT_DIST_THRESHOLD) {
      continue;
    }


    filtered_target_list.push_back(predictions[i]);
  }

  return filtered_target_list;
}

void BehaviorPlanner::plan(vehicle_t ego, traj_sd_t trajectory, double t_inc, double T) {
  vector<vector<double> > predictions = _getTargetFrenetVelocity();
  predictions = _filter(ego, predictions);
  // TODO: cost calculation
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

// TODO: Get vs vd is wrong, we need to fix this?
// Maybe introduce new variable as car_yaw?
vector<double> BehaviorPlanner::_get_vs_vd(const vector<double> d_norm,
                                           const double vx, const double vy) {
  double dx = d_norm[0];
  double dy = d_norm[1];

  // The reason for this is {dx, dy} will be interpolated from spline, dx^2 + dy^2 != 1
  double theta = atan2(vy, vx) - atan2(dy, dx);

  double vd = vy * cos(theta) + vx * sin(theta);
  double vs = -vy * sin(theta) + vx * cos(theta);

  return {vs, vd};
}

double BehaviorPlanner::_distance_from_goal_lane(vehicle_t ego, int lane) {
  double distance = abs(ego.d - from_lane_to_d(lane));
  distance = max(distance, 1.0);

  // Only consider d direction movement
  double time_to_goal = (double) distance/ego.vd;

  // Punish movement towards right lanes
  double multiplier = (double)(MOVE_TO_LEFT_LANE * lane / time_to_goal);

  return multiplier * _plannerCost.REACH_GOAL;
}

double BehaviorPlanner::_inefficiency_cost(vehicle_t ego, double target_speed) {
  double diff = ego.car_speed - target_speed;
  double pct = (double)diff/target_speed;
  double multiplier = pow(pct, 2);
  return multiplier * _plannerCost.EFFICIENCY;
}

double BehaviorPlanner::_collision_cost(double time_til_collision) {
  if (time_til_collision == NO_COLLISION_THRESHOLD) { return 0; }
  double exponent = pow(time_til_collision, 2);
  double mult = exp(exponent);
  return mult * COLLISION;
}

double BehaviorPlanner::_buffer_cost(double shortest_dist_in_movement,
                                     double shortest_time_to_min_buffer) {
  if (shortest_dist_in_movement == 0) { return 10 * DANGER; }
  if (shortest_time_to_min_buffer > DESIRED_BUFFER) { return 0; }
  double multiplier = 1.0 - pow((shortest_time_to_min_buffer / DESIRED_BUFFER), 2);
  return multiplier * DANGER;
}

/**
 * Only consider vehicles in same lane or will be in same lane
 * @param ego
 * @param other
 * @return
 */
double BehaviorPlanner::_get_buffer_dist(vehicle_t ego, vehicle_t other) {
  if (abs(ego.d - other.d) < SAME_LANE_DETECTION_THRESHOLD) {
    return abs(ego.s - other.s);
  }
  return NO_COLLISION_THRESHOLD;
}

bool BehaviorPlanner::_collides_with(vehicle_t ego, vehicle_t other){
  return abs(ego.d - other.d) < SAME_LANE_DETECTION_THRESHOLD &&
          abs(ego.s - other.s) <= COLLIDE_THRESHOLD;
}

/**
 * car_id, s, v_sd[0], d, v_sd[1], x, y
 * @param ego
 * @param trajectory
 * @param t_inc
 * @param T
 */
vector<double> BehaviorPlanner::_will_collide_at(traj_sd_t trajectory, double t_inc, double T,
                                         vector<vector<double> >target_list) {
  double t_steps = T/t_inc;
  double curr_time = 0;

  double shortest_collision_time = NO_COLLISION_THRESHOLD;

  double shortest_distance = NO_COLLISION_THRESHOLD;
  double shortest_time_to_min_buffer = NO_COLLISION_THRESHOLD;
  vehicle_t ego_at_t;
  vehicle_t other;
  // Progress in time
  for (auto i = 0; i < t_steps; i+=1) {
    curr_time += t_inc;

    ego_at_t.s = trajectory.s[i];
    ego_at_t.d = trajectory.d[i];

    // check if they will collide
    for (auto i = 0; i < target_list.size(); i+=1) {
      other.s = target_list[i][1] + target_list[i][2] * curr_time;
      other.d = target_list[i][3] + target_list[i][4] * curr_time;

      // Will collide
      if (_collides_with(ego_at_t, other)) {
        shortest_collision_time = min(curr_time, shortest_collision_time);
        shortest_distance = 0;
        shortest_time_to_min_buffer = shortest_collision_time;
      }

      // Calculate buffer zone
      double dist = _get_buffer_dist(ego_at_t, other);
      if (dist < shortest_distance) {
        shortest_distance = dist;
        shortest_time_to_min_buffer = curr_time;
      }
    }
  }
  return {shortest_collision_time, shortest_distance, shortest_time_to_min_buffer};
}

double BehaviorPlanner::_calculate_cost(vehicle_t ego, int lane, double target_speed,
                                        traj_sd_t trajectory, double t_inc, double T,
                                        vector<vector<double> > target_list) {
  // We will do it
  double distance_cost = _distance_from_goal_lane(ego, lane);
  double inefficiency_cost = _inefficiency_cost(ego, target_speed);
//  double collision_cost = _collision_cost();
  return 0;
}
