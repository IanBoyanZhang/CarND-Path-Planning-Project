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

const int NUMS_OF_CARS = 12;
const double SAME_LANE = 2;
const double CLOSE_DISTANCE = 20;
const double BUFFER_DISTANCE = 25;
const double DETECTION_DISTANCE = 50;
const double COLLISION_DISTANCE = 4.2;

const double MAX_DIST_DIFF = 0.427;

const double PID_P = 0.01;

// Four types of incidents
// COLLISION
// MAX ACCEL
// MAX JERK
// MAX SPEED
// OUT OF LANE

// static double COLLISION = 1e6;
static double EFFICIENCY = 1e2;
// static double MOVE_TO_LEFT_LANE = 5;
//static double DESIRED_BUFFER = 40;


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

// Trajectory in (s, d) space
//struct traj_sd_t {
//	vector<double> s;
//	vector<double> d;
//};

// Trajectory in (x, y) space
struct traj_xy_t {
	vector<double> x;
	vector<double> y;
	vector<double> _VS;
	vector<double> _VD;
};

struct traj_params_t {
	double d_end;
	double ref_vel;
};

struct car_pose_t {
	double x;
	double y;
	double yaw;
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
void get_wp_in_map(const double s, const double d, const vector<double> maps_s,
										 const vector<double> maps_x, const vector<double> maps_y,
												 vector<double>& ptsx, vector<double>& ptsy) {
	vector<double> next_wp0 = getXY(s + 30, d, maps_s, maps_x, maps_y);
	vector<double> next_wp1 = getXY(s + 60, d, maps_s, maps_x, maps_y);
	vector<double> next_wp2 = getXY(s + 90, d, maps_s, maps_x, maps_y);
	vector<double> next_wp3 = getXY(s + 120, d, maps_s, maps_x, maps_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsx.push_back(next_wp3[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);
	ptsy.push_back(next_wp3[1]);
}

int which_lane(double d) {
	return int(d / LANE_WIDTH);
}

/**
 *
 * @param lane
 * @return lane center d
 */
inline double lane_to_d(int lane) {
	return double(2 + lane * LANE_WIDTH);
}

bool can_go_left(double d) {
	return (which_lane(d) - 1) >= MIN_LANE_ID;
}

bool can_go_right(double d) {
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
	} else {
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
	}
	tk::spline s;
	s.set_points(ptsx, ptsy);
	return s;
}

/**************************************************
 * COST FUNCTIONS for Behaviour Planners
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

double collision_cost(double time_till_collision) {
	double exponent = pow(time_till_collision, 2);
	double mult = exp(-exponent);
	return mult * COLLISION;
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

double buffer_to_front_cost(const vector<vector<double> >& sensor_fusion,
														car_telemetry_t car_telemetry, double d_end) {
	double car_s = car_telemetry.car_s;

	int closest_id = closest_car_in_front(sensor_fusion, car_s, d_end);
  double closest_front = numeric_limits<double>::max();

	if (closest_id != -1) {
    closest_front = (double)sensor_fusion[closest_id][CAR_S] - car_s;
	}
  if (closest_front > DESIRED_BUFFER) { return 0.0; }
	double multiplier = 1.0 - pow((closest_front/DESIRED_BUFFER), 2);
	return multiplier * DANGER;
}

//double calculate_all_costs(const vector<vector<double> >& sensor_fusion_snapshot,
//													 const double T, const double t_inc, const int i, const ego_xy_t ego) {
//  return 0;
//}

double inefficiency_cost(const double ref_vel) {
	double pct = (49.5 - ref_vel) / 49.5;
	double multiplier = pow(pct, 2);
	return multiplier * EFFICIENCY;
}

//double inefficiency_cost(traj_xy_t traj_xy) {
//  double average_speed = accumulate(traj_xy._VS.begin(), traj_xy._VS.end(), 0.0)/traj_xy._VS.size();
//  double speed_diff = MAX_DIST_DIFF - average_speed;
//
//	double pct = speed_diff / MAX_DIST_DIFF;
//  double multiplier = pow(pct, 2);
//	return multiplier * EFFICIENCY;
//}

double lane_preference_cost(double d) {
	int lane = which_lane(d);
	return MOVE_TO_LEFT_LANE * (lane - 0);
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
// Causing the car to jump from time to time? Acceleration
double propose_lane_veloctiy(car_telemetry_t c, double d, car_pose_t car_pose,
																const vector<vector<double>>& sensor_fusion) {
	double ego_s = c.car_s;
//	double ego_d = c.car_d;
	double ego_d = d;
	double car_x, car_y;
	double car_vx, car_vy;
	int closest_id = closest_car_in_front(sensor_fusion, ego_s, ego_d);
	double closest_front = numeric_limits<double>::max();
	double target_velocity = 49.5;
  double ref_vel = target_velocity;
	if (closest_id != -1) {
		car_vx = sensor_fusion[closest_id][CAR_VX];
		car_vy = sensor_fusion[closest_id][CAR_VY];
		closest_front = (double)sensor_fusion[closest_id][CAR_S] - ego_s;
		target_velocity = sqrt(pow(car_vx, 2) + pow(car_vy, 2));
	}

	// Define trajectory parameters
	// Too close slow down!
	if (closest_front < BUFFER_DISTANCE) {
		ref_vel = c.car_speed * 0.9;
	}
	if (closest_front < CLOSE_DISTANCE) {
		ref_vel = target_velocity - 1;
	}

	cout << "closest_front: " << closest_front << endl;
	cout << "ref_vel: " << ref_vel << endl;

  return ref_vel;
}

traj_xy_t generate_trajectory(const car_telemetry_t& c, tk::spline& s, car_pose_t car_pose, double ref_vel) {
		traj_xy_t traj_xy;
	for (int i = 0; i < c.previous_path_x.size(); i+=1) {
		traj_xy.x.push_back(c.previous_path_x[i]);
		traj_xy.y.push_back(c.previous_path_y[i]);
	}

  double target_x = 40.0;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

	double x_add_on = 0.0;
//	double ref_vel = 49.5;

	for (int i = 0; i < 50 - c.previous_path_x.size(); i+=1) {
		double N = (target_dist/(0.02*ref_vel*0.44704));
		double x_point = x_add_on + (target_x)/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// back to global?
		x_point = (x_ref * cos(car_pose.yaw) - y_ref * sin(car_pose.yaw)) + car_pose.x;
		y_point = (x_ref * sin(car_pose.yaw) + y_ref * cos(car_pose.yaw)) + car_pose.y;
		traj_xy.x.push_back(x_point);
		traj_xy.y.push_back(y_point);
	}
	return traj_xy;
}

// Planner
double predict(const vector<vector<double>>& sensor_fusion, traj_xy_t ego_traj, const double ref_vel) {

	// vector< vector<double> > filtered;
	// To achieve that
	double x, y, vx, vy, s, d, id;

	double future_x, future_y;
	double t = 0;

	// Calculate multiple costs
	double cost = 0;

	double time_till_collision = numeric_limits<double>::max();
	for (int i = 0; i < 50; i+=1) {
		t = i * 1/50;
		for (auto i = 0; i < sensor_fusion.size(); i+=1) {
			id = sensor_fusion[i][CAR_ID];
			x = sensor_fusion[i][CAR_X];
			y = sensor_fusion[i][CAR_Y];
			vx = sensor_fusion[i][CAR_VX];
			vy = sensor_fusion[i][CAR_VY];
			s = sensor_fusion[i][CAR_S];
			d = sensor_fusion[i][CAR_D];
			// check lane distance
			vector<double> ego_xy = {ego_traj.x[t], ego_traj.y[t]};
			// Check L2 distanceHopefully, there are more opportunities in the future.
			if (distance(x, y, ego_xy[0], ego_xy[1]) > DETECTION_DISTANCE) { continue; }
			future_y = y + vy * t;
			future_x = x + vx * t;
			// In same lane or not
			if (collides_with(future_x, future_y, ego_traj.x[t], ego_traj.y[t])) {
				time_till_collision = min(t, time_till_collision);
			}
		}
	}

	cost += collision_cost(time_till_collision);
  cost += inefficiency_cost(ref_vel);
	return cost;
}

// From trajectories to lane?
traj_xy_t plan(car_telemetry_t& c, car_pose_t car_pose, tk::spline& s, const vector<vector<double> >& sensor_fusion) {

	int lane = 1;
	double ref_vel = propose_lane_veloctiy(c, lane_to_d(lane), car_pose, sensor_fusion);
	traj_xy_t traj_xy = generate_trajectory(c, s, car_pose, ref_vel);
//  predict(sensor_fusion, traj_xy,)
  return traj_xy;
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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


					vector<double> ptsx;
					vector<double> ptsy;
					int prev_size = previous_path_x.size();
          cout << "prev size: " << prev_size << endl;
          cout << "<<<<<<<<<<<<<< " << endl;

					car_telemetry_t car_telemetry = get_telemetry(j);

					vector<car_pose_t> car_poses = get_init_car_poses(car_telemetry);
					ptsx.push_back(car_poses[0].x);
					ptsx.push_back(car_poses[1].x);
					ptsy.push_back(car_poses[0].y);
					ptsy.push_back(car_poses[1].y);

					car_pose_t car_pose = car_poses[1];

          int lane = 1;
          get_wp_in_map(car_s, lane_to_d(lane),
												map_waypoints_s, map_waypoints_x, map_waypoints_y, ptsx, ptsy);

					tk::spline s = fit_xy(ptsx, ptsy, car_pose);

					traj_xy_t traj_xy = plan(car_telemetry, car_pose, s, sensor_fusion);
//					double ref_vel = 49.5;
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
