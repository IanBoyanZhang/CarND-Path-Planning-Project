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
#include "spline.h"
#include "PTG.h"
#include "utils.h"

using namespace std;


vector<double> _get_vs_vd(const vector<double> d_norm,
           const double vx, const double vy) {
  double dx = d_norm[0];
  double dy = d_norm[1];

  // The reason for this is {dx, dy} will be interpolated from spline, dx^2 + dy^2 != 1
  double theta = atan2(vy, vx) - atan2(-dx, dy);

  double vd = vy * cos(theta) + vx * sin(theta);
  double vs = -vy * sin(theta) + vx * cos(theta);

  return {vs, vd};
}

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
		wp_dx.push_back(maps_dx[wp_id] * 1000);
		// wp_dy.push_back(maps_dy[wp_id]);
		wp_dy.push_back(maps_dy[wp_id] * 1000);
	}

	// Sort for dealing with track wrap around
	// TODO: smooth transition using relative distance

	// TODO: Wrapping around inconsitancy is still an issue
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

vector<double> getTargetXY(const double pos_s, const int lane, const vector<tk::spline> wp_sp) {
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
	const int d = 2 + lane * 4;

	//return {x + d * dx/1000, y + d * dy/1000};
	return {x + d * dx/1000, y + d * dy/1000};
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

  h.onMessage([&max_s, &initialized, &t_begin, &Ptg, &Utils,
											&prev_JMT_s_coeffs,
											&prev_JMT_d_coeffs,
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
//          cout << "car_speed: " << car_speed << endl;

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

					// Init condition
					double pos_x;
					double pos_y;
					double angle;
					double pos_s;

					int path_size = previous_path_x.size();

//          if (path_size != 0) {
//						path_size = 10;
//					}

					if(path_size == 0) {
						pos_x = car_x;
						pos_y = car_y;
						angle = deg2rad(car_yaw);
            pos_s = car_s;
					} else {
/*						pos_x = previous_path_x[path_size - 1];
						pos_y = previous_path_y[path_size - 1];

						double pos_x2 = previous_path_x[path_size - 2];
						double pos_y2 = previous_path_y[path_size - 2];
            // Which one is right?
						angle = atan2(pos_y2 - pos_y, pos_x2 - pos_x);
						// angle = deg2rad(car_yaw);
            pos_s = end_path_s;*/
						pos_s = getFrenet(pos_x, pos_y, angle,
															map_waypoints_x, map_waypoints_y)[0];
						pos_x = previous_path_x[0];
						pos_y = previous_path_y[0];

						double pos_x2 = previous_path_x[1];
            double pos_y2 = previous_path_y[1];
						angle = atan2(pos_y2 - pos_y, pos_x2 - pos_x);
						pos_s = getFrenet(pos_x, pos_y, angle,
															map_waypoints_x, map_waypoints_y)[0];
					}

					// Predict with dynamics or not?
					// Sample coordinate transform
          double s_diff = 0.30;
					vector<double> container;

					vector<tk::spline> wp_sp;

/*					cout << "pos_s" << pos_s << endl;
					cout << "pos_x" << pos_x << endl;
					cout << "pos_y" << pos_y << endl;*/

					wp_sp = fitXY(pos_s, map_waypoints_s,
												map_waypoints_x, map_waypoints_y,
												map_waypoints_dx, map_waypoints_dy,
												max_s);

					vector<double> s_start, d_start;
					vector<double> s_dot_start, s_ddot_start;
					vector<double> d_dot_start, d_ddot_start;

					// JMT
					double T = 1;
					vector<double> s_end, d_end;
					// To mps
					car_speed *= 0.44704;
					vector<double> ego_vs_vd = _get_vs_vd({wp_sp[2](pos_s), wp_sp[3](pos_s)},
																								car_speed * cos(car_yaw),
																								car_speed * sin(car_yaw));

					int lane = 1;
					double t_inc = 0.02;
					double s_inc;

					if (path_size == 0) {
						s_start = {pos_s, 0, 0};
						// Middle lane
						d_start = {6, 0, 0};
						s_end = {pos_s + 15, 20, 0};
						d_end = {6, 0, 0};
					} else {
						// TODO: something wrong with the start, end estimation
						// differentiate previous JMT
						/*s_dot_start = Utils.differentiate(prev_JMT_s_coeffs);
            s_ddot_start = Utils.differentiate(s_dot_start);

						d_dot_start = Utils.differentiate(prev_JMT_d_coeffs);
						d_ddot_start = Utils.differentiate(d_dot_start);

						double init_velo = Utils.evaluate_function(s_dot_start, 0);
						double init_acc = Utils.evaluate_function(s_ddot_start, 0);
            s_start = {pos_s, init_velo, init_acc};
            d_start = {6, 0, 0};

						// Using previous JMT to estimate final location

            s_end = {pos_s + init_velo * T/t_inc, 20, 0};
						d_end = {6, 0, 0};*/
						// TODO: my ego speed is not right
						s_start = {pos_s, s_diff * 50 , 0};
						d_start = {6, 0, 0};

						cout << "ego speed vs: " << ego_vs_vd[0] << endl;
						s_end = {pos_s + s_diff * 50 * T, 20, 0};
						d_end = {6, 0, 0};
					}

					vector<double> s_coeffs = Ptg.JMT(s_start, s_end, T);
					vector<double> d_coeffs = Ptg.JMT(d_start, d_end, T);

					prev_JMT_s_coeffs = s_coeffs;
					prev_JMT_d_coeffs = d_coeffs;

//					for (int i = 0; i < path_size; i+=1) {
//						next_x_vals.push_back(previous_path_x[i]);
//						next_y_vals.push_back(previous_path_y[i]);
//					}

//					if (path_size == 0) {
						next_x_vals.push_back(pos_x);
						next_y_vals.push_back(pos_y);
//					}

          vector<double> container_next;

/*					next_x_vals.push_back(last_x);
					next_y_vals.push_back(last_y);*/
//					for (int i = 1; i < 50 - path_size; i+=1) {
					for (int i = 1; i < 50; i+=1) {
						container = getTargetXY(pos_s + s_diff * (i), 1, wp_sp);
						container_next = getTargetXY(pos_s + s_diff * (i + 1), 1, wp_sp);
            next_x_vals.push_back(next_x_vals[i - 1] + container_next[0] - container[0]);
						next_y_vals.push_back(next_y_vals[i - 1] + container_next[1] - container[1]);
//						next_x_vals.push_back(container[0]);
//						next_y_vals.push_back(container[1]);
					}


/*          for (auto i = 0; i < T/t_inc - path_size; i+=1) {
						s_inc = Utils.evaluate_function(s_coeffs, t_inc * (i + 1));
						container = getTargetXY(s_inc, lane, wp_sp);
						cout << "s_inc:" << s_inc << endl;
						cout << "x: " << container[0] << endl;
						cout << "y: " << container[1] << endl;
            next_x_vals.push_back(container[0]);
						next_y_vals.push_back(container[1]);
					}*/

          cout << "path size: " << path_size << endl;
          cout << "end of packets <<<<<<<<<< " << endl;
					// TODO: second filtering for stitching the path?

					tk::spline prev_sp;
					vector<double> prev_s;
					double temp_s;

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
