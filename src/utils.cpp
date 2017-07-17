//
// Created by ian zhang on 7/17/17.
//
#include "utils.h"

void utils::smoother() {
  tk::spline s;
}

void utils::globalCart2local(const double &car_x, const double &car_y,
                             const double &map_waypoint_x,
                             const double &map_waypoint_y,
                             double &local_waypoint_x, double &local_waypoint_y) {
  local_waypoint_x = map_waypoint_x - car_x;
  local_waypoint_y = map_waypoint_y - car_y;
}
