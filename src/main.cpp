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

#include "prediction.hpp"

//using namespace std;

// for convenience
using json = nlohmann::json;

/**
 * Basic parameter helpers
 */
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mps2mph(double x) { return x * 2.23694; }
double mph2mps(double x) { return x / 2.23694; }
double mps2pointdist(double x) { return x * 0.020; }

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/**
 * Calculate Cartesian distance between two (x,y) points
 */
double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

/**
 * Find closest waypoint ahead or behind
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {
	
  double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

/**
 * Find next waypoint ahead
 */
int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y), (map_x-x));

	double angle = fabs(theta-heading);
  angle = std::min(2*pi()-angle, angle);

  if (angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

/**
 * Transform a position from Cartesian x,y coordinates to Frenet s,d coordinates
 */
std::vector<double> getFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {

  // Find previous and next waypoints from map data
  int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);
	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
    prev_wp  = maps_x.size() - 1;
	}

  // Waypoint vector x and y components from prev waypoint to next waypoint
	double vx_wp = maps_x[next_wp] - maps_x[prev_wp];
	double vy_wp = maps_y[next_wp] - maps_y[prev_wp];

  // Position vector x and y components from prev waypoint to position coord
  double vx_Pos = x - maps_x[prev_wp];
	double vy_Pos = y - maps_y[prev_wp];

	// Find the projection of position vector onto waypoint vector
	double proj_length = ((vx_Pos * vx_wp + vy_Pos * vy_wp) /
                        (vx_wp * vx_wp + vy_wp * vy_wp));
	
  // Find the endpoint (x,y) of projection onto waypoint vector
  double proj_endpt_x = proj_length * vx_wp;
	double proj_endpt_y = proj_length * vy_wp;
  
  // Frenet d value is the normal distance from the endpoint to the position coord
	double frenet_d = distance(vx_Pos, vy_Pos, proj_endpt_x, proj_endpt_y);

	// See if d value is positive or negative by comparing it to a center point (1000, 2000)
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, vx_Pos, vy_Pos);
	double centerToRef = distance(center_x, center_y, proj_endpt_x, proj_endpt_y);
  if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// Calculate s value
	double frenet_s = 0;
  // Accumulate s values up to the previous waypoint
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
	}
  // Add length of the position coord's projection along waypoint vector
	frenet_s += distance(0, 0, proj_endpt_x, proj_endpt_y);

	return {frenet_s, frenet_d};
}


std::vector<double> getFrenetVel(double x, double y, double vx, double vy,
                                 const std::vector<double> &maps_x,
                                 const std::vector<double> &maps_y,
                                 const std::vector<double> &maps_dx,
                                 const std::vector<double> &maps_dy) {
  
  int closest_wp = ClosestWaypoint(x, y, maps_x, maps_y);
  double dx = maps_dx[closest_wp];
  double dy = maps_dy[closest_wp];
  
  double frenet_d_dot = (vx * dx + vy * dy);
  double frenet_s_dot = sqrt(vx*vx + vy*vy - frenet_d_dot*frenet_d_dot);
  
  /*
  std::cout << "vx:" << vx << ",vy:" << vy << ",dx:" << dx << ",dy:" << dy << ",d_dot:" << frenet_d_dot << ",s_dot:" << frenet_s_dot << std::endl;
  */
  
  return {frenet_s_dot, frenet_d_dot};
}

/**
 * Transform from Frenet s,d coordinates to Cartesian x,y
 */
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
	
  int prev_wp = -1;
	while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi()/2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};
}

/**
 * Main loop
 */
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
  	std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          /**
           * Sensor Fusion
           */

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

          // List of detected cars on same side of road
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          //std::cout << sensor_fusion << std::endl;
        
          Vehicle car_ego = Vehicle(-1);
          car_ego.UpdateState(car_x, car_y, car_s, car_d, car_speed, 0, car_s, car_d);
          
          /**
           * Prediction
           */
          
          std::cout << "Detected cars:" << std::endl;
          constexpr double kSensorRange = 100.; // m
          std::vector<Vehicle> cars_detected;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            double sensed_x = sensor_fusion[i][1];
            double sensed_y = sensor_fusion[i][2];
            double dist_to_sensed = distance(car_x, car_y, sensed_x, sensed_y);
            
            if (dist_to_sensed < kSensorRange) {
              int sensed_id = sensor_fusion[i][0];
              double sensed_vx = sensor_fusion[i][3];
              double sensed_vy = sensor_fusion[i][4];
              double sensed_s = sensor_fusion[i][5];
              double sensed_d = sensor_fusion[i][6];
              // Calculate s_dot and d_dot
              std::vector<double> v_frenet = getFrenetVel(sensed_x, sensed_y, sensed_vx, sensed_vy, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);
              double calc_s_dot = v_frenet[0];
              double calc_d_dot = v_frenet[1];
              
              Vehicle sensed_car = Vehicle(sensed_id);
              sensed_car.UpdateState(sensed_x, sensed_y, sensed_s, sensed_d, calc_s_dot, calc_d_dot, car_s, car_d);
              
              cars_detected.push_back(sensed_car);
            }
          }
          
          std::sort(cars_detected.begin(), cars_detected.end(),
                    [ ](const Vehicle &lhs, const Vehicle &rhs)
                       { return lhs.s_rel > rhs.s_rel; } );
          
          // Print out diagram of sensed cars for debugging
          std::cout << std::endl;
          std::string lane_mark;
          for (int i = 100; i > -100; i = i - 10) {
            for (int j_lane = 1; j_lane <= 3; ++j_lane) {
              std::cout << "|";
              lane_mark = "  ";
              for (int k = 0; k < cars_detected.size(); ++k) {
                if ((cars_detected[k].s_rel <= i+4) && (cars_detected[k].s_rel > i-6) && (cars_detected[k].lane == j_lane)) {
                  if (cars_detected[k].veh_id < 10) {
                    lane_mark = "0" + std::to_string(cars_detected[k].veh_id);
                  }
                  else {
                    lane_mark = std::to_string(cars_detected[k].veh_id);
                  }
                }
                else if ((i == 0) && (j_lane == car_ego.lane)) {
                  lane_mark = "@@";
                }
              }
              std::cout << lane_mark;
            }
            std::cout << "|" << std::endl;
          }
          std::cout << std::endl;
          
          /*
          for (int i = 0; i < cars_detected.size(); ++i) {
            std::cout << "ID: " << cars_detected[i].veh_id
                      << ", Lane: " << cars_detected[i].lane
                      << ", s_rel: " << cars_detected[i].s_rel
                      << ", s_dot: " << mps2mph(cars_detected[i].s_dot)
                      << std::endl;
          }
          */
          
          /**
           * Behavior Planning
           */

          
          /**
           * Trajectory Generation
           */
          
          
          /**
           * Control
           */
          
          // Output vector of (x,y) path trajectory values to json message
          json msgJson;
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          // The car will visit each (x,y) point sequentially every .02 seconds
          double target_speed = 49.;
          double dist_inc = mps2pointdist(mph2mps(target_speed));
          for(int i = 0; i < 50; i++)
          {
            double new_car_s = car_s + (i+1) * dist_inc;
            double new_car_d = car_d;
            std::vector<double> new_car_xy = getXY(new_car_s, new_car_d,
                                                   map_waypoints_s,
                                                   map_waypoints_x,
                                                   map_waypoints_y);
            next_x_vals.push_back(new_car_xy[0]);
            next_y_vals.push_back(new_car_xy[1]);
          }
          
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
  // program doesn't compile :-(
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
    //ws.close();
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
