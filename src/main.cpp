#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//#include "BehaviorPlanner.h"
#include <limits>
#include "spline.h"
#include "PTG.h"
#include "utils.h"

using namespace std;

// Constants
//[ id, x, y, vx, vy, s, d]
#define CAR_ID 0
#define CAR_X 1
#define CAR_Y 2
#define CAR_VX 3
#define CAR_VY 4
#define CAR_S 5
#define CAR_D 6

#define P_CAR_ID 0
#define P_CAR_X 1
#define P_CAR_Y 2

const int NUMS_OF_CARS = 12;
const double SAME_LANE = 1;
const double CLOSE_DISTANCE = 23;
const double DETECTION_DISTANCE = 50;
const double COLLISION_DISTANCE = 2.5;

const double MAX_DIST_DIFF = 0.427;

const double PID_P = 0.01;

// Four types of incidents
// COLLISION
// MAX ACCEL
// MAX JERK
// MAX SPEED
// OUT OF LANE

// static double COLLISION = 1e6;
struct ego_xy_t {
  double x;
  double y;
};

struct ego_sd_t {
	double s;
	double d;
};

struct car_telemetry_t {
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
};

// Trajectory in (s, d) space
//struct traj_sd_t {
//	vector<double> s;
//	vector<double> d;
//};

// Trajectory in (x, y) space
struct traj_xy_t {
	vector<double> x;
	vector<double> y;
};

struct traj_params_t {
	double car_vs;
	double car_vd;
	double d_end;
	double target_vs;
};

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

/**
 *
 * @tparam T
 * @tparam Compare
 * @param vec
 * @param compare
 * @return
 */
template <typename T, typename Compare>
std::vector<std::size_t> sort_permutation(
				const std::vector<T>& vec,
				Compare compare)
{
	std::vector<std::size_t> p(vec.size());
	std::iota(p.begin(), p.end(), 0);
	std::sort(p.begin(), p.end(),
						[&](std::size_t i, std::size_t j){ return compare(vec[i], vec[j]); });
	return p;
}

/**
 * @tparam T
 * @param vec
 * @param p
 * @return
 */
template <typename T>
std::vector<T> apply_permutation(
				const std::vector<T>& vec,
				const std::vector<std::size_t>& p)
{
	std::vector<T> sorted_vec(vec.size());
	std::transform(p.begin(), p.end(), sorted_vec.begin(),
								 [&](std::size_t i){ return vec[i]; });
	return sorted_vec;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// Transform from Frenet s,d coordinates to Cartesian X,Y using different local strategy
vector<tk::spline> fitXY(const double s, const vector<double> maps_s,
										 const vector<double> maps_x, const vector<double> maps_y,
										 const vector<double> maps_dx, const vector<double> maps_dy,
											const double max_s) {
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1)))
	{
		prev_wp++;
	}

	vector<double> wp_s;
  vector<double> wp_x;
	vector<double> wp_y;
	vector<double> wp_dx;
	vector<double> wp_dy;
	int wp_id;

	int back_track_id = -10;
	for (int i = back_track_id; i < 15; i+=1) {
	//for (int i = 0, len = 7; i < len; i+=1) {
		wp_id = (prev_wp + i)%maps_x.size();

		if (wp_id < 0) {
			wp_id += maps_s.size();
		}

		wp_s.push_back(maps_s[wp_id]);
		wp_x.push_back(maps_x[wp_id]);
		wp_y.push_back(maps_y[wp_id]);
		// For better numerically stability
		// wp_dx.push_back(maps_dx[wp_id]);
		wp_dx.push_back(maps_dx[wp_id]);
		// wp_dy.push_back(maps_dy[wp_id]);
		wp_dy.push_back(maps_dy[wp_id]);
	}

	// Sort for dealing with track wrap around
	// TODO: smooth transition using relative distance

	// TODO: Wrapping around inconsistancy is still an issue?
  auto p = sort_permutation(wp_s, less<double>());

	wp_s = apply_permutation(wp_s, p);
  wp_x = apply_permutation(wp_x, p);
  wp_y = apply_permutation(wp_y, p);
	wp_dx = apply_permutation(wp_dx, p);
	wp_dy = apply_permutation(wp_dy, p);

	tk::spline wp_sp_x;
	tk::spline wp_sp_y;
	tk::spline wp_sp_dx;
	tk::spline wp_sp_dy;
	wp_sp_x.set_points(wp_s, wp_x);
	wp_sp_y.set_points(wp_s, wp_y);
  wp_sp_dx.set_points(wp_s, wp_dx);
	wp_sp_dy.set_points(wp_s, wp_dy);

	return {wp_sp_x, wp_sp_y, wp_sp_dx, wp_sp_dy};
}

/*****************************************************************
 * Coordinate transformation
 *****************************************************************/
vector<double> mapXY2localXY(const double map_x, const double map_y,
														 const double car_x, const double car_y, const double yaw)
{
	double x = map_x - car_x;
	double y = map_y - car_y;

	return {x * cos(yaw) + y * sin(yaw),
					-x * sin(yaw) + y * cos(yaw)};
}

vector<double> localXY2mapXY(const double car_x, const double car_y,
														 const double l_x, const double l_y, const double yaw){
	return {l_x * cos(yaw) - l_y * sin(yaw) + car_x,
					l_y * sin(yaw) + l_y * cos(yaw) + car_y};
}

vector<double> getTargetXY(const double pos_s, const double d, const vector<tk::spline> wp_sp) {
	const int LANE_WIDTH = 4;
	tk::spline wp_sp_x = wp_sp[0];
	tk::spline wp_sp_y = wp_sp[1];
	tk::spline wp_sp_dx = wp_sp[2];
	tk::spline wp_sp_dy = wp_sp[3];

	double x = wp_sp_x(pos_s);
	double y = wp_sp_y(pos_s);
	double dx = wp_sp_dx(pos_s);
	double dy = wp_sp_dy(pos_s);

	/**
	 * lane = 0, 1, 2
	 */
//	const int d = 2 + lane * 4;

	//return {x + d * dx/1000, y + d * dy/1000};
	return {x + d * dx, y + d * dy};
}

/**************************************************
 * COST FUNCTIONS for Behaviour Planners
 **************************************************/
bool collides_with(double car_x, double car_y, double other_x, double other_y) {
	return distance(car_x, car_y, other_x, other_y) < 3;
}

bool in_same_lane(double car_d, double other_d) {
	return abs(other_d - car_d) <= SAME_LANE;
}

// For now, only consider the front vehicle
int closest_car_in_front(const vector<vector<double>>& sensor_fusion,
														 const double car_s , const double car_d) {

	double closest_to_front = numeric_limits<double>::max();
	double front_distance = 0;
	int min_id = -1;
	double other_car_d;
	for (int i = 0; i < NUMS_OF_CARS; i+=1) {
		other_car_d = sensor_fusion[i][CAR_D];
    // Below may not be necessary it seems the simulator sends
		// Only the same direction traveling vehicles
//		if (other_car_d < 0) { continue; }
		if (!in_same_lane(car_d, other_car_d)) { continue; }
		front_distance = sensor_fusion[i][CAR_S] - car_s;
		if (front_distance < 0) { continue; }
    if (front_distance < closest_to_front) {
			closest_to_front = front_distance;
			min_id = i;
		}
	}
	return min_id;
}

double collision_cost(const vector<vector<double> >& sensor_fusion_snapshot,
											const double t_inc, const double T, const ego_xy_t ego) {
  double future_x, future_y;

	double time_till_collision = numeric_limits<double>::max();
	for (auto i = 0; i < sensor_fusion_snapshot.size(); i+=1) {
		future_x = sensor_fusion_snapshot[i][CAR_X];
		future_y = sensor_fusion_snapshot[i][CAR_Y];
		if (collides_with(future_x, future_y, ego.x, ego.y) <= COLLISION_DISTANCE) {
			time_till_collision = min(t_inc * i, time_till_collision);
		}
	}
	double exponent = pow(time_till_collision, 2);
	double mult = exp(-exponent);
	return mult * COLLISION;
}

double max_accel_cost(const vector<vector<double> >& sensor_fusion_snapshot,
											const double T, const double t_inc, const int i, traj_xy_t ego_traj) {
	// Going through trajectory to find max accels
	// calculate movement in Map Cartesian X-Y
	int steps = ego_traj.x.size();
	for (auto i = 0; i < steps; i+=1) {

	}
  return 0;
}

double buffer_cost(const vector<vector<double> >& sensor_fusion_snapshot,
									 const double T, const double t_inc, const int i, const ego_xy_t ego) {
	return 0;
}

double calculate_all_costs(const vector<vector<double> >& sensor_fusion_snapshot,
													 const double T, const double t_inc, const int i, const ego_xy_t ego) {
  return 0;
}

// Generating prediction table for all
// Design predicted table data structure
// 3 dimensional data structure with T?
double predict(const vector<vector<double> >& sensor_fusion,
							 const double t_inc, const double T, traj_xy_t ego_traj,
							 const vector<tk::spline>wp_sp) {
	// vector< vector<double> > filtered;
	// To achieve that
	double x, y, vx, vy, s, d, id;

	double future_x, future_y;
	vector<vector<double> > sensor_fusion_snapshot;

	int nums_steps = (int)T/t_inc;
	double t = 0;

  // Calculate multiple costs
	double cost = 0;
	ego_xy_t ego;

	for (int i = 0; t < nums_steps; i+=1) {
		t = i * t_inc;
    ego = {ego_traj.x[i], ego_traj.y[i]};
  	for (auto i = 0; i < sensor_fusion.size(); i+=1) {
			id = sensor_fusion[i][CAR_ID];
			x = sensor_fusion[i][CAR_X];
			y = sensor_fusion[i][CAR_Y];
			vx = sensor_fusion[i][CAR_VX];
			vy = sensor_fusion[i][CAR_VY];
			s = sensor_fusion[i][CAR_S];
			d = sensor_fusion[i][CAR_D];
			// check lane distance
    	// Adjacent lane cars
			// Check when collision will happen
			vector<double> ego_xy = {ego_traj.x[t], ego_traj.y[t]};
			// Check L2 distanceHopefully, there are more opportunities in the future.
			if (distance(x, y, ego_xy[0], ego_xy[1]) > DETECTION_DISTANCE) { continue; }
			future_y = y + vy * t;
      future_x = x + vx * t;

			sensor_fusion_snapshot.push_back({id, future_x, future_y});
			// Going through all cost functions
			cost += collision_cost(sensor_fusion_snapshot, t_inc, T, ego);
		}
	}

  // Return cost
  return cost;
}

// FSM
// lane or d?
// sensor_fusion or sensor_fusion_snapshot
/**
 * car_vs should really be from car_telemetry
 * @param car_telemetry
 * @param sensor_fusion
 * @param car_vs
 * @return
 */
// TODO: maybe the self lane keeping method is buggy?
traj_params_t realize_stay_lane(car_telemetry_t car_telemetry,
																const vector<vector<double>>& sensor_fusion,
																double car_vs) {
	double car_s = car_telemetry.car_s;
	double car_d = car_telemetry.car_d;

  double car_vx, car_vy;
	int closest_id = closest_car_in_front(sensor_fusion, car_s, car_d);
	double closest_front;
	double target_velocity;
	if (closest_id != -1) {
		car_vx = sensor_fusion[closest_id][CAR_VX];
		car_vy = sensor_fusion[closest_id][CAR_VY];
		closest_front = (double)sensor_fusion[closest_id][CAR_S] - car_s;
		target_velocity = sqrt(pow(car_vx, 2) + pow(car_vy, 2));
	} else {
		closest_front = numeric_limits<double>::max();
		target_velocity = 49;
	}

	// Define trajectory parameters
	double target_vs = MAX_DIST_DIFF;
	if (closest_front < CLOSE_DISTANCE) {
		target_vs = target_velocity * .44704/50;
	}
	/***********************************************
	 * Define target d_end and vd
   ***********************************************/
	// TODO: Dynamically decide the lane the car will be driving
  int lane = 2;
//	int lane = int((car_d - 2)/4);
	double d_end = 2 + lane * 4;

	double step_dist = d_end - car_d;
	double car_vd = step_dist/200;

	traj_params_t traj_params = { car_vs, car_vd, d_end, target_vs};
  return traj_params;
}

void generate_traj(double& car_s, double &car_d, double& car_vs, double &car_vd,
									 double& d_end, double target_vs, vector<double>& VS,
									 vector<tk::spline> wp_sp, int nums_step, vector<double>& next_x_vals, vector<double>& next_y_vals) {
	VS.clear();
  double vs_diff = 0.0005;
	double pred_car_s = car_s;
  double pred_car_d = car_d;
	vector<vector<double> > traj_res;
  vector<double> ego_xy;
	vector<double> ego_xy_next;

	double dist_inc = 0;
	for (int i = 1; i < nums_step; i+=1) {
    // Use dynamics to predict motion?
		double cte = pred_car_d - d_end;
		// P term
		if (abs(cte) >= 0.7) {
			car_vd = 0.01 * -cte;
		}
		if (abs(cte) > 0.1 && abs(cte) < 0.7) {
			car_vd = PID_P * (-cte);
			if (abs(car_d - 2) > 30) {
				car_d = 0;
			}
		}
		// D term
		pred_car_d += car_vd;
		// S_control
		double s_error = car_vs - target_vs;

		if(s_error < 0) {
			car_vs += vs_diff;
		} else {
			car_vs -= vs_diff;
		}

		/*****************************************************
   	 * Caching velocity predictions for next simulator loop
   	 *****************************************************/
		ego_xy = getTargetXY(pred_car_s + car_vs, pred_car_d + car_vd, wp_sp);
		ego_xy_next = getTargetXY(pred_car_s + car_vs * 2, pred_car_d + car_vd * 2, wp_sp);

		dist_inc = distance(ego_xy[0], ego_xy[1], ego_xy_next[0], ego_xy_next[1]);
		while (dist_inc > MAX_DIST_DIFF) {
			car_vs *= 0.8;
			ego_xy = getTargetXY(pred_car_s + car_vs, pred_car_d + car_vd, wp_sp);
			ego_xy_next = getTargetXY(pred_car_s + car_vs * 2, pred_car_d + car_vd * 2, wp_sp);
			dist_inc = distance(ego_xy[0], ego_xy[1], ego_xy_next[0], ego_xy_next[1]);
		}

		VS.push_back(car_vs);
		pred_car_s += car_vs;
		pred_car_d += car_vd;

		next_x_vals.push_back(next_x_vals[i - 1] + ego_xy_next[0] -  ego_xy[0]);
		next_y_vals.push_back(next_y_vals[i - 1] + ego_xy_next[1] -  ego_xy[1]);
	}
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

	PTG Ptg;
	utils Utils;
//	BehaviorPlanner BP;

	/*********************************************
	 * Caching
	 *********************************************/
	vector<double> prev_JMT_s_coeffs;
	vector<double> prev_JMT_d_coeffs;

	bool initialized = false;
	auto t_begin = chrono::high_resolution_clock::now();
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	vector<double> VS = {0};

  h.onMessage([&max_s, &initialized, &t_begin, &Ptg, &Utils,
											&prev_JMT_s_coeffs,
											&prev_JMT_d_coeffs,
					&VS,
											&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

          // CAR_SPEED returned as mps
					if (!initialized) {
						auto dt = 0;
            t_begin = chrono::high_resolution_clock::now();
						initialized = true;
					} else {
						auto end = chrono::high_resolution_clock::now();
						// cast to ns
						auto dt = chrono::duration_cast<std::chrono::milliseconds>(end - t_begin).count();
						t_begin = end;
						// cout << "time difference: " << dt << endl;
					}

					int path_size = previous_path_x.size();

					int nums_step = 50;
					// ~ 49.5mph
					const double max_s_diff = 0.427;

					// Init condition
					double pos_x;
					double pos_y;
					double angle;
					double pos_s;

					if(path_size == 0) {
						pos_x = car_x;
						pos_y = car_y;
						angle = deg2rad(car_yaw);
            pos_s = car_s;
					} else {
						pos_x = previous_path_x[0];
						pos_y = previous_path_y[0];

						double pos_x2 = previous_path_x[1];
            double pos_y2 = previous_path_y[1];
						angle = atan2(pos_y2 - pos_y, pos_x2 - pos_x);
						pos_s = getFrenet(pos_x, pos_y, angle,
															map_waypoints_x, map_waypoints_y)[0];
					}

					// Predict with dynamics or not?
					vector<tk::spline> wp_sp;

					wp_sp = fitXY(pos_s, map_waypoints_s,
												map_waypoints_x, map_waypoints_y,
												map_waypoints_dx, map_waypoints_dy,
												max_s);

          // Proposed start Horizon
					next_x_vals.push_back(car_x);
					next_y_vals.push_back(car_y);

					// Refining dt with real time calc?
//					double DT = 0.02;

          /********************
           * Preliminary Behavior Planner
           * To follow single line, going straight, just slowing down
           * 1. check collision in front
           * 2. figure out the right speed
           *
           * sensor[id][0, 1, 2...]
           ********************/
					/***********************************************
  				 * Define vs
  				 ***********************************************/
					double car_vs;
					int consumered_steps = 0;
					if (path_size == 0) {
						consumered_steps = 3;
						car_vs = 0;
					} else {
						consumered_steps = nums_step - path_size;
						// Happen to be the very next timestamp
						car_vs = VS[consumered_steps];
					}

          car_telemetry_t car_telemetry = {car_x, car_y, car_s, car_d, car_yaw, car_speed};
					traj_params_t traj_params = realize_stay_lane(car_telemetry, sensor_fusion, car_vs);

					/***********************************************
					 * Trajectory Generation
					 ***********************************************/
					generate_traj(car_s, car_d, car_vs,
												traj_params.car_vd, traj_params.d_end, traj_params.target_vs, VS,
												wp_sp, nums_step, next_x_vals, next_y_vals);

//					cout << "path size: " << path_size << endl;
//          cout << "end of packets <<<<<<<<<< " << endl;

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
