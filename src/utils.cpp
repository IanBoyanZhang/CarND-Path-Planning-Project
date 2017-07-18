//
// Created by ian zhang on 7/17/17.
//
#include "utils.h"

void utils::globalCart2local(const double &car_x, const double &car_y,
                             const std::vector<double> &map_waypoints_x,
                             const std::vector<double> &map_waypoints_y,
                             std::vector<double> &local_waypoints_x,
                             std::vector<double> &local_waypoints_y) {

  for (auto map_x:map_waypoints_x) {
    local_waypoints_x.push_back(map_x - car_x);
  }

  for (auto map_y:map_waypoints_y) {
    local_waypoints_y.push_back(map_y - car_y);
  }
}

tk::spline utils::curve_fit(const std::vector<double> &local_waypoints_x,
                            const std::vector<double> &local_waypoints_y){
  tk::spline s;
  s.set_points(local_waypoints_x, local_waypoints_y);
  return s;
}
