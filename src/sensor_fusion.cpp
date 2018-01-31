//
//  sensor_fusion.cpp
//  Path_Planning
//
//  Created by Student on 1/30/18.
//

#include "sensor_fusion.hpp"

int GetCurrentTrajIndex(const VehTrajectory &prev_ego_traj,
                        int prev_path_size) {
  
  int idx_current_pt = 0;
  const int traj_size = prev_ego_traj.states.size();
  //const int prev_path_size = previous_path_x.size();
  if (traj_size > prev_path_size) {
    idx_current_pt = (traj_size - prev_path_size - 1);
  }
  
  return idx_current_pt;
}

/**
 *
 */
VehState ProcessEgoState(double car_x, double car_y,
                         int idx_current_pt,
                         VehTrajectory prev_ego_traj,
                         const std::vector<double> &map_interp_s,
                         const std::vector<double> &map_interp_x,
                         const std::vector<double> &map_interp_y) {
  VehState ego_state;
  
  // Get (s,d) from interpolated waypoints
  std::vector<double> car_sd = GetHiResFrenet(car_x, car_y,
                                              map_interp_s,
                                              map_interp_x,
                                              map_interp_y);
  const double car_s = car_sd[0];
  const double car_d = car_sd[1];
  
  // Calculate s_dot, s_dot_dot and d_dot, d_dot_dot by finding index
  // of last processed trajectory point from previous path and evaluate
  // that time with the derivative polynomial coefficients
  double car_s_dot = 0.;
  double car_s_dotdot = 0.;
  double car_d_dot = 0.;
  double car_d_dotdot = 0.;
  if (prev_ego_traj.states.size() > 0) {
    car_s_dot = prev_ego_traj.states[idx_current_pt].s_dot;
    car_s_dotdot = prev_ego_traj.states[idx_current_pt].s_dotdot;
    car_d_dot = prev_ego_traj.states[idx_current_pt].d_dot;
    car_d_dotdot = prev_ego_traj.states[idx_current_pt].d_dotdot;
  }
  
  ego_state.x = car_x;
  ego_state.y = car_y;
  ego_state.s = car_s;
  ego_state.s_dot = car_s_dot;
  ego_state.s_dotdot = car_s_dotdot;
  ego_state.d = car_d;
  ego_state.d_dot = car_d_dot;
  ego_state.d_dotdot = car_d_dotdot;

  // Debug logging
  if (kDBGSensorFusion != 0) {
    // Check (x,y)-(s,d) conversion accuracy
    std::vector<double> car_xy = GetHiResXY(car_s, car_d, map_interp_s,
                                            map_interp_x, map_interp_y);
    
    std::cout << "x: " << ego_state.x << ", y: " << ego_state.y
              << ", xy->s: " << car_s << ", xy->d: " << car_d
              << ", sd->x: " << car_xy[0] << ", sd->y: " << car_xy[1];
    std::cout << "" << std::endl;
  }
  
  return ego_state;
}

/**
 *
 */
void ProcessDetectedCars(std::map<int, DetectedVehicle> &detected_cars,
                         const EgoVehicle &ego_car,
                         const std::vector<std::vector<double>> &sensor_fusion,
                         const std::vector<double> &map_interp_s,
                         const std::vector<double> &map_interp_x,
                         const std::vector<double> &map_interp_y,
                         const std::vector<double> &map_interp_dx,
                         const std::vector<double> &map_interp_dy) {
  
  // Check all sensor fusion vehicles for distance from ego car
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    const int sensed_id = sensor_fusion[i][0];
    const double sensed_x = sensor_fusion[i][1];
    const double sensed_y = sensor_fusion[i][2];
    const double dist_to_sensed = Distance(ego_car.state_.x, ego_car.state_.y,
                                           sensed_x, sensed_y);
    
    if (dist_to_sensed < kSensorRange) {
      // Vehicle within sensor range
      const double sensed_vx = sensor_fusion[i][3];
      const double sensed_vy = sensor_fusion[i][4];
      
      auto det_car_sd = GetHiResFrenet(sensed_x, sensed_y, map_interp_s,
                                       map_interp_x, map_interp_y);
      const double sensed_s = det_car_sd[0];
      const double sensed_d = det_car_sd[1];
      
      // Calculate s_dot and d_dot
      const int sensed_closest_wp = ClosestWaypoint(sensed_x, sensed_y,
                                                    map_interp_x,
                                                    map_interp_y);
      
      const std::vector<double> v_frenet = GetFrenetVelocity(sensed_vx,
                                                             sensed_vy,
                                                             sensed_closest_wp,
                                                             map_interp_dx,
                                                             map_interp_dy);
      const double calc_s_dot = v_frenet[0];
      const double calc_d_dot = v_frenet[1];
      
      VehState new_det_car_state;
      new_det_car_state.x = sensed_x;
      new_det_car_state.y = sensed_y;
      new_det_car_state.s = sensed_s;
      new_det_car_state.s_dot = calc_s_dot;
      new_det_car_state.s_dotdot = 0.;
      new_det_car_state.d = sensed_d;
      new_det_car_state.d_dot = calc_d_dot;
      new_det_car_state.d_dotdot = 0.;
      
      if (detected_cars.count(sensed_id) == 0) {
        // New sensed vehicle, build detected vehicle object to add
        DetectedVehicle sensed_car = DetectedVehicle(sensed_id);
        sensed_car.UpdateState(new_det_car_state);
        sensed_car.UpdateRelDist(ego_car.state_.s, ego_car.state_.d);
        
        detected_cars[sensed_car.veh_id_] = sensed_car;
        
        // Debug logging
        if (kDBGSensorFusion != 0) {
          std::cout << "Added ID: " << detected_cars[sensed_id].veh_id_
                    << std::endl;
        }
      }
      else {
        // Vehicle already in map, just update values
        detected_cars.at(sensed_id).UpdateState(new_det_car_state);
        detected_cars.at(sensed_id).UpdateRelDist(ego_car.state_.s,
                                                  ego_car.state_.d);
        
        // Debug logging
        if (kDBGSensorFusion != 0) {
          std::cout << "Updated ID: " << detected_cars.at(sensed_id).veh_id_
                    << std::endl;
        }
      }
    }
    else {
      // Vehicle outside of sensor range, remove it from map
      if (detected_cars.count(sensed_id) > 0) {
        detected_cars.erase(sensed_id);
        
        // Debug logging
        if (kDBGSensorFusion != 0) {
          std::cout << "Erased ID: " << sensed_id << std::endl;
        }
      }
    }
  }
}

/**
 *
 */
std::map<int, std::vector<int>> SortDetectedCarsByLane(
                         const std::map<int, DetectedVehicle> &detected_cars) {
  
  std::map<int, std::vector<int>> car_ids_by_lane;
  
  // Gather detected car id's in a map by lane number
  for (auto it = detected_cars.begin(); it != detected_cars.end(); ++it) {
    car_ids_by_lane[it->second.lane_].push_back(it->second.veh_id_);
  }
  
  // Lambda sort car id's in each lane by S relative to ego car
  for (auto it = car_ids_by_lane.begin(); it != car_ids_by_lane.end(); ++it) {
    // it->second is a vector of int for the car id's in that lane;
    std::sort(it->second.begin(), it->second.end(),
              [&detected_cars](int &lhs, int &rhs) -> bool {
                auto lhs_s_rel = detected_cars.at(lhs).s_rel_;
                auto rhs_s_rel = detected_cars.at(rhs).s_rel_;
                return lhs_s_rel > rhs_s_rel; // higher s_rel first
              });
  }
  
  // Debug logging
  if (kDBGSensorFusion != 0) {
    // Print out car id's sorted by lane
    std::cout << "Cars sorted by lane:" << std::endl;
    for (auto it = car_ids_by_lane.begin(); it != car_ids_by_lane.end(); ++it) {
      std::cout << "lane #" << it->first << " - ";
      for (int i = 0; i < it->second.size(); ++i) {
        std::cout << it->second[i] << "= "
                  << detected_cars.at(it->second[i]).s_rel_ << ", ";
      }
      std::cout << std::endl;
    }
  }
  
  return car_ids_by_lane;
}
