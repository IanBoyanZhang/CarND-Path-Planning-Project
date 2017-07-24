//
// Created by ian zhang on 7/17/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include "spline.h"

/**
 * Lane center d magnitude in simulator
 * Lane width is 4 (meter)
 */
// What is the right coordination?
const uint8_t LANE1 = 2;
const uint8_t LANE2 = 2 + 4;
const uint8_t LANE3 = 2 + 4 + 4;

class utils {

public:
  void globalCart2local(const double& car_x, const double& car_y,
                        const std::vector<double>& map_waypoints_x,
                        const std::vector<double>& map_waypoints_y,
                        std::vector<double>& local_waypoints_x,
                        std::vector<double>& local_waypoints_y);

  // Cubic Spline interpolation
  tk::spline curve_fit(const std::vector<double>& local_waypoints_x,
                const std::vector<double>& local_waypoints_y);


};


#endif //PATH_PLANNING_UTILS_H
