//
// Created by ian zhang on 7/25/17.
//

#ifndef PATH_PLANNING_PTG_H
#define PATH_PLANNING_PTG_H

#include "constants.h"
#include "Eigen-3.3/Eigen/Dense"
#include <vector>
#include <map>
#include "Vehicle.h"
#include "Eigen-3.3/Eigen/src/Core/Matrix.h"
/*
    Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).

    arguments:
     start_s - [s, s_dot, s_ddot]

     start_d - [d, d_dot, d_ddot]

     target_vehicle - id of leading vehicle (int) which can be used to retrieve
       that vehicle from the "predictions" dictionary. This is the vehicle that
       we are setting our trajectory relative to.

     delta - a length 6 array indicating the offset we are aiming for between us
       and the target_vehicle. So if at time 5 the target vehicle will be at
       [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
       state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
       goal of "follow 10 meters behind and 4 meters to the right of target vehicle"

     T - the desired time at which we will be at the goal (relative to now as t=0)

     predictions - dictionary of {v_id : vehicle }. Each vehicle has a method
       vehicle.state_in(time) which returns a length 6 array giving that vehicle's
       expected [s, s_dot, s_ddot, d, d_dot, d_ddot] state at that time.

    return:
     (best_s, best_d, best_t) where best_s are the 6 coefficients representing s(t)
     best_d gives coefficients for d(t) and best_t gives duration associated w/
     this trajectory.
 */

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class PTG {
public:
  PTG();
  PTG(vector<double>start_s, vector<double>start_d,
      int target_vehile, vector<double>delta, int T, map<int, Vehicle>);
  //run();
  vector<double> JMT(vector<double> start, vector<double> end, double T);
};


#endif //PATH_PLANNING_PTG_H
