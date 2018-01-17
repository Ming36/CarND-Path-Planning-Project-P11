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

#include "path_common.hpp"
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
void DebugPrintRoad(const std::map<int, DetectedVehicle> &detected_cars,
                    const EgoVehicle &ego_car) {
  
  std::cout << "Detected cars:" << std::endl;
  std::cout << std::endl;
  std::string lane_mark;
  
  for (double i = 100; i > -100; i = i - 10) {
    for (int j_lane = 1; j_lane <= 3; ++j_lane) {
      std::cout << "|";
      lane_mark = "  ";
      
      if ((i == 0) && (j_lane == ego_car.lane_)) {
        lane_mark = "@@"; // mark ego car
      }
      else {
        // Detected cars as map
        for (std::map<int, DetectedVehicle>::const_iterator it =
             detected_cars.begin(); it != detected_cars.end(); ++it) {
          if ((it->second.s_rel_ <= i+4) && (it->second.s_rel_ > i-6) &&
              (it->second.lane_ == j_lane)) {
            if (it->second.veh_id_ < 10) {
              lane_mark = "0" + std::to_string(it->second.veh_id_);
            }
            else {
              lane_mark = std::to_string(it->second.veh_id_);
            }
          }
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

  // Instantiate ego car object and map of detected cars to hold their data
  EgoVehicle ego_car = EgoVehicle(-1);
  std::map<int, DetectedVehicle> detected_cars;
  long int count = 0;
  
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &ego_car, &detected_cars, &count]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
                
    // Store time at start of processing received data for latency estimation
    auto t_msg = std::chrono::high_resolution_clock::now();
    auto t_msg_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(t_msg);
    auto value = t_msg_ms.time_since_epoch();
    long duration = value.count();
                
    //auto t_msg_ms = std::chrono::duration_cast<std::chrono::milliseconds>
    //            (std::chrono::high_resolution_clock::now());
                
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
           *   ego_car, detected_cars
           */

          count++;
          //std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
          //std::cout << "Loop #" << count << std::endl;
          
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
          const double end_path_s = j[1]["end_path_s"];
          const double end_path_d = j[1]["end_path_d"];

          // List of detected cars on same side of road
          const auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Update ego car's state
          //std::vector<double> calc_sd = GetFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          
          // DEBUG check accuracy of sensed vs calculated s,d
          //std::cout << "car_s: " << car_s << ", calc_s: " << calc_sd[0] << ", gap: " << car_s - calc_sd[0] << std::endl;
          //std::cout << "car_d: " << car_d << ", calc_d: " << calc_sd[1] << ", gap: " << car_d - calc_sd[1] << std::endl;
          
          // TODO Calculate s_dot, d_dot using speed and yaw
          /*
          std::cout << "last trajectory size: " << ego_car.trajectory_.s.size() << ", prev path x size: " << previous_path_x.size() << std::endl;
          int end = 0;
          if (ego_car.trajectory_.s.size() > 0) {
            end = ego_car.trajectory_.s.size()-1;
            std::cout << "prev traj end s: " << ego_car.trajectory_.s[end] << ", end path s: " << end_path_s << std::endl;
          }
           */

          double idx_last_s = 0;
          if ((previous_path_x.size() > 0) &&
              (ego_car.trajectory_.s.size() > previous_path_x.size())) {
            idx_last_s = ego_car.trajectory_.s.size() - previous_path_x.size() - 1;
          }
          double car_s_dot = EvalPoly(kSimCycleTime * idx_last_s, ego_car.coeffs_JMT_s_dot_);
          double car_s_dot_dot = EvalPoly(kSimCycleTime * idx_last_s,
                                          ego_car.coeffs_JMT_s_dot_dot_);

          /*
          double car_s_dot = 0;
          for (int i=0; i < ego_car.trajectory_.s.size(); ++i) {
            if (car_s > ego_car.trajectory_.s[i]) {
              car_s_dot = EvalPoly(kSimCycleTime * (i+1), ego_car.coeffs_JMT_s_dot_);
            }
          }
           */
          
          //const double car_s_dot = mph2mps(car_speed); // DUMMY use speed as s_dot
          const double car_d_dot = 0; // DUMMY fix d_dot to zero
          
          //std::cout << "s: " << car_s << ", ego_car s: " << ego_car.s_ << std::endl;
          ego_car.UpdateState(car_x, car_y, car_s, car_d, car_s_dot, car_d_dot, car_s_dot_dot, 0);
          //std::cout << "new ego_car s: " << ego_car.s_ << std::endl;
          
          
          
          // Check all vehicles for distance
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            const int sensed_id = sensor_fusion[i][0];
            const double sensed_x = sensor_fusion[i][1];
            const double sensed_y = sensor_fusion[i][2];
            const double dist_to_sensed = Distance(car_x, car_y, sensed_x, sensed_y);
            
            if (dist_to_sensed < kSensorRange) {
              // Vehicle within sensor range
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
              
              if (detected_cars.count(sensed_id) == 0) {
                // New sensed vehicle, build detected vehicle object to add
                DetectedVehicle sensed_car = DetectedVehicle(sensed_id);
                sensed_car.UpdateState(sensed_x, sensed_y, sensed_s, sensed_d,
                                       calc_s_dot, calc_d_dot, 0, 0);
                sensed_car.UpdateRelDist(ego_car.s_, ego_car.d_);
                
                detected_cars[sensed_car.veh_id_] = sensed_car;
              }
              else {
                // Vehicle already in map, just update values
                detected_cars[sensed_id].UpdateState(sensed_x, sensed_y,
                                                     sensed_s, sensed_d,
                                                     calc_s_dot, calc_d_dot, 0, 0);
                detected_cars[sensed_id].UpdateRelDist(ego_car.s_, ego_car.d_);
              }
            }
            else {
              // Vehicle outside of sensor range, remove it from map
              if (detected_cars.count(sensed_id) > 0) {
                detected_cars.erase(sensed_id);
              }
            }
          }
          
          // Sort detected vehicle id's by lane
          std::map<int, std::vector<int>> car_ids_by_lane;
          for (std::map<int, DetectedVehicle>::iterator it = detected_cars.begin();
               it != detected_cars.end(); ++it) {
            car_ids_by_lane[it->second.lane_].push_back(it->second.veh_id_);
          }
          
          /*
          // DEBUG Print out car id's sorted by lane
          std::cout << "Cars sorted by lane:" << std::endl;
          for (std::map<int, std::vector<int>>::iterator it = car_ids_by_lane.begin();
               it != car_ids_by_lane.end(); ++it) {
            std::cout << "lane #" << it->first << " - ";
            for (int i=0; i < it->second.size(); ++i) {
              std::cout << it->second[i] << ", ";
            }
            std::cout << std::endl;
          }
          std::cout << std::endl;
          */
          
          // Sort detected cars vector to start from farthest ahead of ego car
          /*
          std::sort(cars_detected.begin(), cars_detected.end(),
                    [ ](const DetectedVehicle &lhs, const DetectedVehicle &rhs)
                       { return lhs.s_rel_ > rhs.s_rel_; } ); // lambda sort
          */
          
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
           *   detected_cars, car_ids_by_lane
           */
          
          PredictBehavior(detected_cars, car_ids_by_lane);
          PredictTrajectory(detected_cars, car_ids_by_lane, kPredictTime);
          
          
          /**
           * Behavior Planning
           *   1. Use FSM to decide intent ("KL", "PLCL", "PLCR", "LCL", "LCR")
           *   Example criteria to decide lane change:
           *     Check distance and speed of current preceding vehicle,
           *     Find lane with fastest preceding vehicles,
           *     Find lane with farthest preceding vehicles
           * Output:
           *   car_target_behavior [FSM state, car ahead, target lane, target time to achieve target]
           */
          VehBehavior car_target_behavior;
          
          
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
          GetTrajectory(ego_car, kPredictTime, map_waypoints_x, map_waypoints_y, map_waypoints_s);
          
          
          // DEBUG Basic telemetry output
          std::cout << count << ", t: " << duration << ", idx_last_s: " << idx_last_s
          << ", x: " << car_x
          << ", y: " << car_y
          << ", s: " << car_s
          << ", s_dot: " << ego_car.s_dot_
          << ", s_dot_dot: " << ego_car.s_dot_dot_
          << ", d: " << car_d
          << ", d_dot: " << ego_car.d_dot_
          << ", d_dot_dot: " << ego_car.d_dot_dot_;
          /*
          std::cout << ", traj_x: ";
          for (int i=0; i < ego_car.trajectory_.x.size(); ++i) {
            std::cout << ego_car.trajectory_.x[i] << ";";
          }
          std::cout << ", traj_y: ";
          for (int i=0; i < ego_car.trajectory_.y.size(); ++i) {
            std::cout << ego_car.trajectory_.y[i] << ";";
          }
          */
          std::cout << ", traj_s: ";
          for (int i=0; i < ego_car.trajectory_.s.size(); ++i) {
            std::cout << ego_car.trajectory_.s[i] << ";";
          }
          
          std::cout << ", traj_d: ";
          for (int i=0; i < ego_car.trajectory_.d.size(); ++i) {
            std::cout << ego_car.trajectory_.d[i] << ";";
          }
          
          std::cout << std::endl;
          
          /*
          double dist_inc = 0.5;
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          for(int i = 0; i < 50; i++)
          {
            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
          }
          ego_car.trajectory_.x = next_x_vals;
          ego_car.trajectory_.y = next_y_vals;
          */
          /*
          std::vector<double> start_state_s = {0,0,0};
          std::vector<double> end_state_s = {40,20,0};
          std::vector<double> JMT_test_coeffs = JMT(start_state_s, end_state_s, 4);
          std::cout << "JMT: ";
          for (int i=0; i < JMT_test_coeffs.size(); ++i) {
            std::cout << JMT_test_coeffs[i] << ", ";
          }
          std::cout << std::endl;
          */
          
          /**
           * Control
           *   1. Pack and send JSON path trajectory coordinates
           */
          
          // Output vector of (x,y) path trajectory values to json message
          json msgJson;
          msgJson["next_x"] = ego_car.trajectory_.x;
          msgJson["next_y"] = ego_car.trajectory_.y;
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // Slow down path planning process loop
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          // DEBUG print out diagram of sensed cars for debugging
          //DebugPrintRoad(detected_cars, ego_car);
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
