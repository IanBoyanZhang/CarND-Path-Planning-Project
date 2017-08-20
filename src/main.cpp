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
#include <limits>
#include "spline.h"

#include "constants.h"

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

#define LANE_WIDTH 4
#define MAX_LANE_ID 2;
#define MIN_LANE_ID 0;

const double SAME_LANE = 3.8;
const double CLOSE_DISTANCE = 30;
const double BUFFER_DISTANCE = 40;
const double DETECTION_DISTANCE = 80;
const double COLLISION_DISTANCE = 6;
//const double COLLISION_BUFFER = 60;
const double COLLISION_BUFFER = 30;

const double SPEED_AUG = 1.2;

const double MAX_DIST_DIFF = 0.427;

const double PID_P = 0.05;

// In constants.h
//const double MAX_SPEED = 46;

// Four types of incidents
// COLLISION
// MAX ACCEL
// MAX JERK
// MAX SPEED
// OUT OF LANE

// static double COLLISION = 1e6;
static double EFFICIENCY = 1e3;
static const double CHANGE_LANE_COST = 0.1;
// static double MOVE_TO_LEFT_LANE = 5;
//static double DESIRED_BUFFER = 80;

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
  vector<double> previous_path_x;
	vector<double> previous_path_y;
	double end_path_s;
	double end_path_d;
};

// Trajectory in (x, y) space
struct traj_xy_t {
	vector<double> x;
	vector<double> y;
  double velocity;
	double cost = numeric_limits<double>::max();
};

struct car_pose_t {
	double x;
	double y;
	double yaw;
};

struct map_waypoints_t {
	vector<double> x;
	vector<double> y;
	vector<double> s;
	vector<double> dx;
	vector<double> dy;
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



int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	//heading vector
	double hx = map_x-x;
	double hy = map_y-y;

	//Normal vector:
	double nx = maps_dx[closestWaypoint];
	double ny = maps_dy[closestWaypoint];

	//Vector into the direction of the road (perpendicular to the normal vector)
	double vx = -ny;
	double vy = nx;

	//If the inner product of v and h is positive then we are behind the waypoint so we do not need to
	//increment closestWaypoint, otherwise we are beyond the waypoint and we need to increment closestWaypoint.

	double inner = hx*vx+hy*vy;
	if (inner<0.0) {
		closestWaypoint++;
	}

	return closestWaypoint;
}

//// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
//{
//	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
//
//	int prev_wp;
//	prev_wp = next_wp-1;
//	if(next_wp == 0)
//	{
//		prev_wp  = maps_x.size()-1;
//	}
//
//	double n_x = maps_x[next_wp]-maps_x[prev_wp];
//	double n_y = maps_y[next_wp]-maps_y[prev_wp];
//	double x_x = x - maps_x[prev_wp];
//	double x_y = y - maps_y[prev_wp];
//
//	// find the projection of x onto n
//	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
//	double proj_x = proj_norm*n_x;
//	double proj_y = proj_norm*n_y;
//
//	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
//
//	//see if d value is positive or negative by comparing it to a center point
//
//	double center_x = 1000-maps_x[prev_wp];
//	double center_y = 2000-maps_y[prev_wp];
//	double centerToPos = distance(center_x,center_y,x_x,x_y);
//	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
//
//	if(centerToPos <= centerToRef)
//	{
//		frenet_d *= -1;
//	}
//
//	// calculate s value
//	double frenet_s = 0;
//	for(int i = 0; i < prev_wp; i++)
//	{
//		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
//	}
//
//	frenet_s += distance(0,0,proj_x,proj_y);
//
//	return {frenet_s,frenet_d};
//}

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
vector<vector<double>> get_wp_in_map(const double s, double d_start, const double d, const vector<double> maps_s,
										 const vector<double> maps_x, const vector<double> maps_y) {
//	double N = 4;
//	double d_diff = (d - d_start)/N;

	d_start = d;
	double d_diff = 0;
	vector<double> next_wp0 = getXY(s + 30, d_start += d_diff, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(s + 60, d_start += d_diff, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(s + 90, d_start += d_diff, maps_s, maps_x, maps_y);
	vector<double> next_wp3 = getXY(s + 120, d_start += d_diff, maps_s, maps_x, maps_y);

	vector<vector<double>> next_wps;
  next_wps.push_back(next_wp0);
	next_wps.push_back(next_wp1);
	next_wps.push_back(next_wp2);
	next_wps.push_back(next_wp3);

  return next_wps;
}

int which_lane(double d) {
	return int(d / LANE_WIDTH);
}

inline double lane_to_d(int lane) {
	return double(2 + lane * LANE_WIDTH);
}

bool has_left_lane(double d) {
	return (which_lane(d) - 1) >= MIN_LANE_ID;
}

bool has_right_lane(double d) {
	return (which_lane(d) + 1) <= MAX_LANE_ID;
}

/*****************************************************************
 * Coordinate transformation
 *****************************************************************/
car_telemetry_t get_telemetry(json j) {
	car_telemetry_t car_telemetry = {
					j[1]["x"],
					j[1]["y"],
					j[1]["s"],
					j[1]["d"],
					j[1]["yaw"],
					j[1]["speed"],
					// Previous path data given to the Planner
					j[1]["previous_path_x"],
					j[1]["previous_path_y"],
					// Previous path's end s and d values
					j[1]["end_path_s"],
					j[1]["end_path_d"]
	};
  return car_telemetry;
}
vector<car_pose_t> get_init_car_poses(const car_telemetry_t& c) {

	double car_x = c.car_x;
	double car_y = c.car_y;
	double car_yaw = c.car_yaw;

	double ref_x = car_x, ref_y = car_y;
	double ref_x_prev, ref_y_prev;
	double ref_yaw = deg2rad(car_yaw);

	int prev_size = c.previous_path_x.size();

	if (prev_size < 2) {
		ref_x_prev = car_x - cos(car_yaw);
		ref_y_prev = car_y - sin(car_yaw);
	}
	else {
		ref_x = c.previous_path_x[prev_size - 1];
		ref_y = c.previous_path_y[prev_size - 1];
		ref_x_prev = c.previous_path_x[prev_size - 2];
		ref_y_prev = c.previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
	}

	car_pose_t car_pose = {ref_x, ref_y, ref_yaw};
  car_pose_t prev_car_pose = {ref_x_prev, ref_y_prev, ref_yaw};
	return {prev_car_pose, car_pose};
}

double estimate_true_car_speed(const vector<car_pose_t>& car_poses) {
	return distance(car_poses[1].x, car_poses[1].y, car_poses[0].x, car_poses[1].y) / 0.02 * 2.237;
};

vector<double> map2local(const double map_x, const double map_y, const car_pose_t car_pose)
{
	double x = map_x - car_pose.x;
	double y = map_y - car_pose.y;

	return {x * cos(0 - car_pose.yaw) - y * sin(0 - car_pose.yaw),
					x * sin(0 - car_pose.yaw) + y * cos(0 - car_pose.yaw)};
}

//vector<double> local2map(const car_pose_t car_pose, const double l_x, const double l_y){
//	return {l_x * cos(car_pose.yaw) - l_y * sin(car_pose.yaw) + car_pose.x,
//					l_y * sin(car_pose.yaw) + l_y * cos(car_pose.yaw) + car_pose.y};
//}

tk::spline fit_xy(vector<double>& ptsx, vector<double>& ptsy,
									const car_pose_t car_pose) {
	vector<double> local_xy;
	for(int i = 0; i < ptsx.size(); i+=1) {
		local_xy = map2local(ptsx[i], ptsy[i], car_pose);
		ptsx[i] = local_xy[0];
		ptsy[i] = local_xy[1];
		// 0.68 - 0.70 mile track warping
		if (i && ptsx[i] <= ptsx[i - 1]) {
			ptsx[i] += 1e6 * numeric_limits<double>::epsilon();
		}
	}
	tk::spline s;
	s.set_points(ptsx, ptsy);
	return s;
}

/**************************************************
 * Utils functions for Behaviour Planners
 **************************************************/
bool collides_with(double car_x, double car_y, double other_x, double other_y) {
	return distance(car_x, car_y, other_x, other_y) < COLLISION_DISTANCE;
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
	for (int i = 0; i < sensor_fusion.size(); i+=1) {
		other_car_d = sensor_fusion[i][CAR_D];
    // This may not be necessary it seems the simulator sends
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

double collision_cost(double time_till_collision) {
	double exponent = pow(time_till_collision, 2);
	double mult = exp(-exponent);
	return mult * COLLISION;
}

double change_lane_cost(double d_end, double ego_d) {
	double diff = abs(d_end - ego_d);
	return diff * CHANGE_LANE_COST;
}

//double max_accel_cost(const vector<vector<double> >& sensor_fusion_snapshot,
//											const double T, const double t_inc, const int i, traj_xy_t ego_traj) {
//	 Going through trajectory to find max accels
//	 calculate movement in Map Cartesian X-Y
//	int steps = ego_traj.x.size();
//	for (auto i = 0; i < steps; i+=1) {
//
//	}
//  return 0;
//}

double collision_distance_cost(double min_distance) {
  if (min_distance > COLLISION_BUFFER) { return 0; }

  if (min_distance > COLLISION_BUFFER) { return 0.0; }
	double multiplier = 1.0 - pow((min_distance/COLLISION_BUFFER), 2);
	return multiplier * DANGER;
}

//double calculate_all_costs(const vector<vector<double> >& sensor_fusion_snapshot,
//													 const double T, const double t_inc, const int i, const ego_xy_t ego) {
//  return 0;
//}

double inefficiency_cost(const double ref_vel) {
	double pct = (MAX_SPEED - ref_vel) / MAX_SPEED;
	double multiplier = pow(pct, 2);
	return multiplier * EFFICIENCY;
}

//double lane_preference_cost(double d) {
//	int lane = which_lane(d);
//	return MOVE_TO_LEFT_LANE * (lane - 0);
//}

double propose_lane_velocity(car_telemetry_t c, double d,
														 const vector<vector<double>>& sensor_fusion, double ref_vel) {
	double ego_s = c.car_s;
//	double ego_d = c.car_d;
	double ego_d = d;
	double car_x, car_y;
	double car_vx, car_vy;
	int closest_id = closest_car_in_front(sensor_fusion, ego_s, ego_d);
	double closest_front = numeric_limits<double>::max();
	double target_velocity = MAX_SPEED;
	if (closest_id != -1) {
		car_vx = sensor_fusion[closest_id][CAR_VX];
		car_vy = sensor_fusion[closest_id][CAR_VY];
		closest_front = (double)sensor_fusion[closest_id][CAR_S] - ego_s;
		// mps to mph
		target_velocity = sqrt(pow(car_vx, 2) + pow(car_vy, 2)) * 2.237;
	}

	// Define trajectory parameters
	// Too close slow down!
//	if (closest_front < BUFFER_DISTANCE && closest_front >= CLOSE_DISTANCE) {
//		ref_vel += PID_P * (target_velocity - ref_vel);
//		ref_vel -= .224;
//		ref_vel -= .112;
//	}

//	if (closest_front < CLOSE_DISTANCE) {
//    ref_vel -= .10;
//    ref_vel -= .224;
//	}

//	if (closest_front >= BUFFER_DISTANCE && ref_vel <= 49.5 && ref_vel >= 20){
//	if (closest_front >= BUFFER_DISTANCE&& ref_vel <= MAX_SPEED){
//    ref_vel += PID_P * (MAX_SPEED - ref_vel);
//		ref_vel += .224;
//	}

  // low speed/cold start
//	if (closest_front >= BUFFER_DISTANCE && ref_vel <= 20) {
		// .3 gives you about 9m/s^2 s acceleration
//		ref_vel += .224;
//	}

	if (closest_front < BUFFER_DISTANCE) {
		ref_vel = target_velocity;
	}

	cout << "closest_front: " << closest_front << endl;
	cout << "ref_vel: " << ref_vel << endl;

  return ref_vel;
}

traj_xy_t generate_trajectory(const car_telemetry_t& c, tk::spline& s, car_pose_t car_pose, double ref_vel) {
	traj_xy_t traj_xy;

  double car_speed = c.car_speed;
	// smooth again doesn't really help seems
	for (int i = 0; i < c.previous_path_x.size(); i+=1) {
		traj_xy.x.push_back(c.previous_path_x[i]);
		traj_xy.y.push_back(c.previous_path_y[i]);
	}

//  double target_x = 30.0;
	double target_x = 50.0;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

	double x_add_on = 0.0;

	double prev_x = 0;
	double prev_y = 0;

	if (c.previous_path_x.size() > 0) {
		prev_x = c.previous_path_x.back();
    prev_y = c.previous_path_y.back();
	}

	double speed = 0;

  for (int i = 0; i < 50 - c.previous_path_x.size(); i+=1) {
		if (ref_vel > car_speed) {
			car_speed += .224;
		} else {
			car_speed -= .224;
		}

		double N = (target_dist/(0.02*car_speed*0.44704));
		double x_point = x_add_on + (target_x)/N;
		double y_point = s(x_point);
		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// back to global?
		x_point = (x_ref * cos(car_pose.yaw) - y_ref * sin(car_pose.yaw)) + car_pose.x;
		y_point = (x_ref * sin(car_pose.yaw) + y_ref * cos(car_pose.yaw)) + car_pose.y;

		// TODO: CHECK SPEED AND ACCL
		speed = distance(prev_x, prev_y, x_point, y_point);
    prev_x = x_point;
		prev_y = y_point;

//		cout << "x_point" << x_point << endl;
//    cout << "y_point" << y_point << endl;
		traj_xy.x.push_back(x_point);
		traj_xy.y.push_back(y_point);
	}

//  for (int i = 0; i < )
	return traj_xy;
}

double predict(const vector<vector<double>>& sensor_fusion, traj_xy_t ego_traj, const double ref_vel) {

	double x, y, vx, vy, s, d, id;

	double future_x, future_y;

	// Calculate multiple costs
	double cost = 0;

	double time_till_collision = numeric_limits<double>::max();
	double min_distance = numeric_limits<double>::max();
	for (int i = 0; i < 50; i+=1) {
		vector<double> ego_xy = {ego_traj.x[i], ego_traj.y[i]};
		for (auto j = 0; j < sensor_fusion.size(); j+=1) {
			id = sensor_fusion[j][CAR_ID];
			x = sensor_fusion[j][CAR_X];
			y = sensor_fusion[j][CAR_Y];
			vx = sensor_fusion[j][CAR_VX];
			vy = sensor_fusion[j][CAR_VY];
			s = sensor_fusion[j][CAR_S];
			d = sensor_fusion[j][CAR_D];
			// check lane distance
			// Check L2 distanceHopefully, there are more opportunities in the future.
			if (distance(x, y, ego_xy[0], ego_xy[1]) > DETECTION_DISTANCE) { continue; }
			future_y = y + SPEED_AUG * vy * i * 0.02;
			future_x = x + SPEED_AUG * vx * i * 0.02;

      // Collision detection/prediction under x-y
			if (collides_with(future_x, future_y, ego_xy[0], ego_xy[1])) {
				time_till_collision = min(i * 0.02, time_till_collision);
			}
      double dist = distance(future_x, future_y, ego_xy[0], ego_xy[1]);
			if ( dist < min_distance ) {
				min_distance = dist;
			}
		}
	}

	cost += collision_cost(time_till_collision);
  cost += inefficiency_cost(ref_vel);
	cost += collision_distance_cost(min_distance);
	return cost;
}

traj_xy_t propose_trajectory(const car_telemetry_t c, car_pose_t car_pose, const double d_start, const double d,
														 const vector<vector<double> >& sensor_fusion, double ref_vel, const map_waypoints_t& map_wps,
														 vector<double> ptsx, vector<double>ptsy) {
	double car_s = c.car_s;
	vector<vector<double> > next_wps = get_wp_in_map(car_s, d_start, d, map_wps.s, map_wps.x, map_wps.y);
	for (auto i = 0; i < next_wps.size(); i+=1) {
		ptsx.push_back(next_wps[i][0]);
		ptsy.push_back(next_wps[i][1]);
	}
	tk::spline s = fit_xy(ptsx, ptsy, car_pose);
//  double _ref_vel = propose_lane_velocity(c, d, sensor_fusion, ref_vel);

	// from new planner
	double _ref_vel = ref_vel;
	traj_xy_t traj_xy = generate_trajectory(c, s, car_pose, _ref_vel);
  double cost = predict(sensor_fusion, traj_xy, _ref_vel);
	traj_xy.velocity = _ref_vel;
  traj_xy.cost = cost;

	return traj_xy;
}

//traj_xy_t plan(car_telemetry_t& c, car_pose_t car_pose, const vector<vector<double> >& sensor_fusion, double ref_vel,
//							 const map_waypoints_t& map_wps, vector<double>ptsx, vector<double>ptsy) {
//
//	int lane = which_lane(c.car_d);
//	cout << "lane: " << lane << endl;
//
//  double d = lane_to_d(lane);
//  traj_xy_t traj_xy_1 = propose_trajectory(c, car_pose, d, d, sensor_fusion, ref_vel, map_wps, ptsx, ptsy);
//
//	cout << "cost 1: " << traj_xy_1.cost << endl;
//
//	traj_xy_t traj_xy = traj_xy_1;
//	if (has_left_lane(c.car_d)) {
//
//		lane = which_lane(c.car_d) - 1;
//		d = lane_to_d(lane);
//		traj_xy_t traj_xy_left = propose_trajectory(c, car_pose, c.car_d, d, sensor_fusion, ref_vel, map_wps, ptsx, ptsy);
//		cout << "cost GO_LEFT: " << traj_xy_left.cost << endl;
//		// change lane!
//
//		cout << "car_d: " << c.car_d << endl;
//
//		if (traj_xy_left.cost < traj_xy.cost) {
//			traj_xy = traj_xy_left;
//		}
//	}
//
//	if (has_right_lane(c.car_d)) {
//
//		lane = which_lane(c.car_d) + 1;
//		d = lane_to_d(lane);
//		traj_xy_t traj_xy_right = propose_trajectory(c, car_pose, c.car_d, d, sensor_fusion, ref_vel, map_wps, ptsx, ptsy);
//
//		cout << "GO RIGHT: " << traj_xy_right.cost << endl;
//		cout << "car_d: " << c.car_d << endl;
//		// change lane!
////		traj_xy_right.cost += 50;
//
//		if (traj_xy_right.cost < traj_xy.cost) {
//			traj_xy = traj_xy_right;
//		}
//	}
//
//	// Decision
//  return traj_xy;
//}

double new_plan(car_telemetry_t& c, car_pose_t car_pose, const vector<vector<double>>& sensor_fusion,
									 const map_waypoints_t& map_wps, int& lane, int& lane_change_wp) {
	int next_wp = -1;
	int prev_size = c.previous_path_x.size();

  double car_s = c.car_s;

	double ref_x = car_pose.x;
  double ref_y = car_pose.y;
	double ref_yaw = car_pose.yaw;

	double car_speed = c.car_speed;

	double ref_vel = 49.5;

	if(prev_size < 2)
	{
		next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_wps.x,map_wps.y,map_wps.dx, map_wps.dy);
	}
	else
	{
		ref_x = c.previous_path_x[prev_size-1];
		double ref_x_prev = c.previous_path_x[prev_size-2];
		ref_y = c.previous_path_y[prev_size-1];
		double ref_y_prev = c.previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
		next_wp = NextWaypoint(ref_x,ref_y,ref_yaw,map_wps.x,map_wps.y, map_wps.dx, map_wps.dy);

		car_s = c.end_path_s;

		car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
	}

	//find ref_v to use
	double closestDist_s = 100000;
	bool change_lanes = false;
	for(int i = 0; i < sensor_fusion.size(); i++)
	{
		//car is in my lane
		float d = sensor_fusion[i][6];
		if(d < (2+4*lane+2) && d > (2+4*lane-2) )
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensor_fusion[i][5];
			check_car_s+=((double)prev_size*.02*check_speed);
			//check s values greater than mine and s gap
			if((check_car_s > car_s) && ((check_car_s-car_s) < 30) && ((check_car_s-car_s) < closestDist_s ) )
			{

				closestDist_s = (check_car_s - car_s);

				if((check_car_s-car_s) > 20)
				{

					//match that cars speed
					ref_vel = check_speed*2.237;
					change_lanes = true;
				}
				else
				{
					//go slightly slower than the cars speed
					ref_vel = check_speed*2.237-5;
					change_lanes = true;

				}
			}


		}
	}

	//try to change lanes if too close to car in front
	if(change_lanes && ((next_wp-lane_change_wp)%map_wps.x.size() > 2))
	{
		bool changed_lanes = false;
		//first try to change to left lane
		if(lane != 0 && !changed_lanes)
		{
			bool lane_safe = true;
			for(int i = 0; i < sensor_fusion.size(); i++)
			{
				//car is in left lane
				float d = sensor_fusion[i][6];
				if(d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) )
				{
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx+vy*vy);

					double check_car_s = sensor_fusion[i][5];
					check_car_s+=((double)prev_size*.02*check_speed);
					double dist_s = check_car_s-car_s;
					if(dist_s < 20 && dist_s > -20)
					{
						lane_safe = false;
					}
				}
			}
			if(lane_safe)
			{
				changed_lanes = true;
				lane -= 1;
				lane_change_wp = next_wp;
			}
		}
		//next try to change to right lane
		if(lane != 2 && !changed_lanes)
		{
			bool lane_safe = true;
			for(int i = 0; i < sensor_fusion.size(); i++)
			{
				//car is in right lane
				float d = sensor_fusion[i][6];
				if(d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) )
				{
					double vx = sensor_fusion[i][3];
					double vy = sensor_fusion[i][4];
					double check_speed = sqrt(vx*vx+vy*vy);

					double check_car_s = sensor_fusion[i][5];
					check_car_s+=((double)prev_size*.02*check_speed);
					double dist_s = check_car_s-car_s;
					if(dist_s < 20 && dist_s > -10)
					{
						lane_safe = false;
					}
				}
			}
			if(lane_safe)
			{
				changed_lanes = true;
				lane += 1;
				lane_change_wp = next_wp;
			}

		}
	}
	return ref_vel;
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

	/*********************************************
	 * Caching
	 *********************************************/
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

	double ref_vel = 0.0;
	map_waypoints_t map_wps = { map_waypoints_x,
															map_waypoints_y,
															map_waypoints_s,
															map_waypoints_dx,
															map_waypoints_dy };

	int lane = 1;
	int lane_change_wp = 0;

  h.onMessage([&ref_vel, &lane, &lane_change_wp, &map_wps, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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


//					vector<double> ptsx;
//					vector<double> ptsy;
//					int prev_size = previous_path_x.size();
//
//					car_telemetry_t car_telemetry = get_telemetry(j);
//
//					vector<car_pose_t> car_poses = get_init_car_poses(car_telemetry);
//
//					ptsx.push_back(car_poses[0].x);
//					ptsx.push_back(car_poses[1].x);
//					ptsy.push_back(car_poses[0].y);
//					ptsy.push_back(car_poses[1].y);
//
//					car_pose_t car_pose = car_poses[1];
//          if (prev_size >= 2) {
//						car_s = end_path_s;
//						car_telemetry.car_s = car_s;
//						car_telemetry.car_speed = estimate_true_car_speed(car_poses);
//					}

					double ref_vel = 49.5; //mph

					int prev_size = previous_path_x.size();

					int next_wp = -1;
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);
					car_telemetry_t car_telemetry = get_telemetry(j);
					if(prev_size < 2)
					{
						next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
					}
					else
					{
						ref_x = previous_path_x[prev_size-1];
						double ref_x_prev = previous_path_x[prev_size-2];
						ref_y = previous_path_y[prev_size-1];
						double ref_y_prev = previous_path_y[prev_size-2];
						ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
						next_wp = NextWaypoint(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);

						car_s = end_path_s;
						car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
					}

          get_init_car_poses(car_telemetry);
					vector<car_pose_t> car_poses = get_init_car_poses(car_telemetry);

//					car_pose_t car_pose = car_poses[1];
					car_pose_t car_pose = {ref_x, ref_y, ref_yaw};


					if (prev_size >= 2) {
						car_s = end_path_s;
						car_telemetry.car_s = car_s;
						car_telemetry.car_speed = car_speed;
					}

					vector<double> ptsx;
					vector<double> ptsy;

					if(prev_size < 2)
					{
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(car_x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(car_y);
					}
					else
					{
						ptsx.push_back(previous_path_x[prev_size-2]);
						ptsx.push_back(previous_path_x[prev_size-1]);

						ptsy.push_back(previous_path_y[prev_size-2]);
						ptsy.push_back(previous_path_y[prev_size-1]);


					}

          //
//					cout << "Estimated car_speed" << car_telemetry.car_speed <<endl;
//					traj_xy_t traj_xy = plan(car_telemetry, car_pose, sensor_fusion, ref_vel, map_wps, ptsx, ptsy);

					// Cache velocity for next simulator loop
//					ref_vel = traj_xy.velocity;

					ref_vel = new_plan(car_telemetry, car_pose, sensor_fusion, map_wps, lane, lane_change_wp);
					vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

					ptsx.push_back(next_wp0[0]);
					ptsx.push_back(next_wp1[0]);
					ptsx.push_back(next_wp2[0]);

					ptsy.push_back(next_wp0[1]);
					ptsy.push_back(next_wp1[1]);
					ptsy.push_back(next_wp2[1]);

//					double d_ = 2 + 4*lane;
//          traj_xy_t traj_xy = propose_trajectory(car_telemetry, car_pose, d_, d_,
//																								 sensor_fusion, ref_vel, map_wps, ptsx, ptsy);
//					generate_trajectory(car_telemetry, );



					ref_yaw = car_pose.yaw;
          ref_x = car_pose.x;
					ref_y = car_pose.y;
					for (int i = 0; i < ptsx.size(); i++ )
					{

						//shift car reference angle to 0 degrees
						double shift_x = ptsx[i]-ref_x;
						double shift_y = ptsy[i]-ref_y;

						ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
						ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

					}
					tk::spline s;


					s.set_points(ptsx,ptsy);


          traj_xy_t traj_xy;

					for(int i = 0; i < previous_path_x.size(); i++)
					{
						traj_xy.x.push_back(previous_path_x[i]);
						traj_xy.y.push_back(previous_path_y[i]);
					}

					car_speed = car_telemetry.car_speed;

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

					double x_add_on = 0;

					for (int i = 1; i <= 50-previous_path_x.size(); i++) {

						if(ref_vel > car_speed)
						{
							car_speed+=.224;
						}
						else if(ref_vel < car_speed)
						{
							car_speed-=.224;
						}


						double N = (target_dist/(.02*car_speed/2.24));
						double x_point = x_add_on+(target_x)/N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
						y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;


						traj_xy.x.push_back(x_point);
						traj_xy.y.push_back(y_point);
					}
					cout << "<<<<<<<<<<<<<< " << endl;

          json msgJson;
					msgJson["next_x"] = traj_xy.x;
					msgJson["next_y"] = traj_xy.y;

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
