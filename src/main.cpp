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

#include "path_helper.hpp"
#include "vehicle.hpp"
#include "prediction.hpp"
#include "behavior.hpp"
#include "trajectory.hpp"

//using namespace std;

// for convenience
using json = nlohmann::json;

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
 * Debug print output of the road lanes with detected vehicle positions
 */
void DebugPrintRoad(const std::vector<DetectedVehicle> cars_detected,
                    const EgoVehicle ego_car) {
  std::cout << "Detected cars:" << std::endl;
  std::cout << std::endl;
  std::string lane_mark;
  for (int i = 100; i > -100; i = i - 10) {
    for (int j_lane = 1; j_lane <= 3; ++j_lane) {
      std::cout << "|";
      lane_mark = "  ";
      for (int k = 0; k < cars_detected.size(); ++k) {
        if ((cars_detected[k].s_rel_ <= i+4) && (cars_detected[k].s_rel_ > i-6) && (cars_detected[k].lane_ == j_lane)) {
          if (cars_detected[k].veh_id_ < 10) {
            lane_mark = "0" + std::to_string(cars_detected[k].veh_id_);
          }
          else {
            lane_mark = std::to_string(cars_detected[k].veh_id_);
          }
        }
        else if ((i == 0) && (j_lane == ego_car.lane_)) {
          lane_mark = "@@";
        }
      }
      std::cout << lane_mark;
    }
    std::cout << "|" << std::endl;
  }
  std::cout << std::endl;
}

/**
 * Main loop
 */
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and dx,dy normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  
  std::string map_file_ = "../../data/highway_map.csv";
  //constexpr double kMaxS = 6945.554; // max before track wraps back to 0

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

  // Instantiate ego car object to hold its data
  EgoVehicle ego_car = EgoVehicle();
  
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ego_car]
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
           *   1. Update ego car's state
           *   2. Process detected cars within sensor range
           *   3. Sort detected cars to start from farthest ahead of ego car
           * Output:
           *   ego_car, cars_detected
           */

        	// Main car's localization Data
          const double car_x = j[1]["x"];
          const double car_y = j[1]["y"];
          const double car_s = j[1]["s"];
          const double car_d = j[1]["d"];
          const double car_yaw = j[1]["yaw"];
          const double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          const auto previous_path_x = j[1]["previous_path_x"];
          const auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's end s and d values
          //const double end_path_s = j[1]["end_path_s"];
          //const double end_path_d = j[1]["end_path_d"];

          // List of detected cars on same side of road
          const auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Update ego car's state
          std::vector<double> calc_sd = GetFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          
          // DEBUG check accuracy of sensed vs calculated s,d
          //std::cout << "car_s: " << car_s << ", calc_s: " << calc_sd[0] << ", gap: " << car_s - calc_sd[0] << std::endl;
          //std::cout << "car_d: " << car_d << ", calc_d: " << calc_sd[1] << ", gap: " << car_d - calc_sd[1] << std::endl;
          
          // TODO Calculate s_dot, d_dot using speed and yaw
          const double car_s_dot = car_speed; // DUMMY use speed as s_dot
          const double car_d_dot = 0; // DUMMY fix d_dot to zero
          
          ego_car.UpdateState(car_x, car_y, car_s, car_d, car_s_dot, car_d_dot);
          
          // Process detected cars within sensor range
          constexpr double kSensorRange = 100.; // m
          std::vector<DetectedVehicle> cars_detected;
          
          // Check all vehicles for distance
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            const double sensed_x = sensor_fusion[i][1];
            const double sensed_y = sensor_fusion[i][2];
            const double dist_to_sensed = Distance(car_x, car_y, sensed_x, sensed_y);
            
            // Only process vehicles within sensor range
            if (dist_to_sensed < kSensorRange) {
              const int sensed_id = sensor_fusion[i][0];
              const double sensed_vx = sensor_fusion[i][3];
              const double sensed_vy = sensor_fusion[i][4];
              const double sensed_s = sensor_fusion[i][5];
              const double sensed_d = sensor_fusion[i][6];
              
              // Calculate s_dot and d_dot
              const int sensed_closest_wp = ClosestWaypoint(sensed_x, sensed_y,
                                                            map_waypoints_x,
                                                            map_waypoints_y);
              const std::vector<double> v_frenet = GetFrenetVelocity(sensed_vx, sensed_vy, sensed_closest_wp, map_waypoints_dx, map_waypoints_dy);
              const double calc_s_dot = v_frenet[0];
              const double calc_d_dot = v_frenet[1];
              
              // Build detected car object and add to array
              DetectedVehicle sensed_car = DetectedVehicle(sensed_id);
              sensed_car.UpdateState(sensed_x, sensed_y, sensed_s, sensed_d,
                                     calc_s_dot, calc_d_dot);
              sensed_car.UpdateRelDist(ego_car.s_, ego_car.d_);
              
              cars_detected.push_back(sensed_car);
            }
          }
          
          // Sort detected cars to start from farthest ahead of ego car
          std::sort(cars_detected.begin(), cars_detected.end(),
                    [ ](const DetectedVehicle &lhs, const DetectedVehicle &rhs)
                       { return lhs.s_rel_ > rhs.s_rel_; } ); // lambda sort
          
          /*
          // DEBUG print detected car stats
          for (int i = 0; i < cars_detected.size(); ++i) {
            std::cout << "ID: " << cars_detected[i].veh_id
                      << ", Lane: " << cars_detected[i].lane
                      << ", s_rel: " << cars_detected[i].s_rel
                      << ", s_dot: " << mps2mph(cars_detected[i].s_dot)
                      << std::endl;
          }
          */
          
          
          /**
           * Prediction
           *   1. Predict detected car behaviors and sort by lane
           *   2. Predict trajectories for each vehicle over fixed time horizon
           * Output:
           *   veh_preds_lanetoleft, veh_preds_curlane, veh_preds_lanetoright
           */
          std::vector<DetectedVehicle> veh_preds_lanetoleft;
          std::vector<DetectedVehicle> veh_preds_curlane;
          std::vector<DetectedVehicle> veh_preds_lanetoright;
          
          
          /**
           * Behavior Planning
           *   1. Use FSM to decide behavior ("KL", "PLCL", "PLCR", "LCL", "LCR")
           *   Example criteria to decide lane change:
           *     Check distance and speed of current preceding vehicle,
           *     Find lane with fastest preceding vehicles,
           *     Find lane with farthest preceding vehicles
           * Output:
           *   car_target_behavior [FSM state, car ahead, target lane, target time to achieve target]
           */
          CarBehavior car_target_behavior;
          
          /**
           * Trajectory Generation
           *   1. Generate potential end states for target behavior with perturbations
           *   2. Generate potential JMT trajectories for each end state
           *   3. Evaluate trajectories with cost function to select best trajectory
           *       (keep following distance, prevent collisions, keep lane center,
           *        minimize lateral jerk, etc)
           *   4. Rate limit best trajectory
           *   5. Convert best trajectory from (s,d) to (x,y)
           * Output:
           *   next_xy_vals [trajectory_x, trajectory_y]
           */
          
          // DUMMY trajectory path (x,y) coordinates
          Trajectory next_xy_vals = GetTrajectory(car_s, car_d, map_waypoints_x, map_waypoints_y, map_waypoints_s);
          
          
          /**
           * Control
           *   1. Pack and send JSON path trajectory coordinates
           */
          
          // Output vector of (x,y) path trajectory values to json message
          json msgJson;
          msgJson["next_x"] = next_xy_vals.coord1;
          msgJson["next_y"] = next_xy_vals.coord2;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          // Slow down path planning process loop
          //std::this_thread::sleep_for(std::chrono::milliseconds(100));
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          // DEBUG print out diagram of sensed cars for debugging
          DebugPrintRoad(cars_detected, ego_car);
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
