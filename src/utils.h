//
// Created by ian zhang on 7/17/17.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include "spline.h"

class utils {

public:
  void smoother();

  void globalCart2local(const double& car_x, const double& car_y,
                        const double& map_waypoint_x,
                        const double& map_waypoint_y,
                        double& local_waypoint_x,
                        double& local_waypoint_y);

};


#endif //PATH_PLANNING_UTILS_H
