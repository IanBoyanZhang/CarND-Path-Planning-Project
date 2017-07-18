//
// Created by ian zhang on 7/17/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>
#include "spline.h"

class utils {

public:
  void globalCart2local(const double& car_x, const double& car_y,
                        const std::vector<double>& map_waypoints_x,
                        const std::vector<double>& map_waypoints_y,
                        std::vector<double>& local_waypoints_x,
                        std::vector<double>& local_waypoints_y);

  tk::spline curve_fit(const std::vector<double>& local_waypoints_x,
                const std::vector<double>& local_waypoints_y);


};


#endif //PATH_PLANNING_UTILS_H
