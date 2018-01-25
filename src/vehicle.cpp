//
//  vehicle.cpp
//  Path_Planning
//
//  Created by Student on 1/11/18.
//

#include "vehicle.hpp"

/**
 * Constructor
 */
Vehicle::Vehicle() {
  
  //intent_ = kUnknown;
}
Vehicle::Vehicle(int veh_id) {
  veh_id_ = veh_id;
  //intent_ = kUnknown;
  traj_.coeffs_JMT_s = {0, 0, 0, 0, 0, 0};
  traj_.coeffs_JMT_s_dot = {0, 0, 0, 0, 0, 0};
  traj_.coeffs_JMT_s_dotdot = {0, 0, 0, 0, 0, 0};
  traj_.coeffs_JMT_d = {0, 0, 0, 0, 0, 0};
  traj_.coeffs_JMT_d_dot = {0, 0, 0, 0, 0, 0};
  traj_.coeffs_JMT_d_dotdot = {0, 0, 0, 0, 0, 0};
}

/**
 * Destructor
 */
Vehicle::~Vehicle() { }

/**
 * Updates vehicle's state values
 */
void Vehicle::UpdateState(double x, double y, double s, double d,
                          double s_dot, double d_dot,
                          double s_dotdot, double d_dotdot) {
  
  // Update state values
  state_.x = x;
  state_.y = y;
  state_.s = s;
  state_.s_dot = s_dot;
  state_.s_dotdot = s_dotdot;
  state_.d = d;
  state_.d_dot = d_dot;
  state_.d_dotdot = d_dotdot;
  
  // Detect lane number (1 is closest to middle of road)
  int lane = 0;
  double n = 0;
  while (d > n * kLaneWidth) {
    lane++;
    n = n + 1.0;
  }
  lane_ = lane;
}


/**
 * Constructor
 */
EgoVehicle::EgoVehicle() : Vehicle() { }
EgoVehicle::EgoVehicle(int id) : Vehicle(id) { }

/**
* Destructor
*/
EgoVehicle::~EgoVehicle() { }


/**
 * Constructor
 */
DetectedVehicle::DetectedVehicle() : Vehicle() { }
DetectedVehicle::DetectedVehicle(int id) : Vehicle(id) { }

/**
 * Destructor
 */
DetectedVehicle::~DetectedVehicle() { }

/**
 * Calculate relative (s,d) from ego car
 */
void DetectedVehicle::UpdateRelDist(double s_ego, double d_ego) {
  double s_rel = state_.s - s_ego;
  // Normalize for track wrap-around
  while (s_rel > kSensorRange) { s_rel -= kMaxS; }
  while (s_rel < -kSensorRange) { s_rel += kMaxS; }
  s_rel_ = s_rel;
  
  d_rel_ = state_.d - d_ego;
}

std::tuple<int, double> GetCarAheadInLane(const int check_lane, const int check_id,
                                          const EgoVehicle &ego_car,
                                          const std::map<int, DetectedVehicle> &detected_cars,
                                          const std::map<int, std::vector<int>> &car_ids_by_lane) {

  int car_id_ahead = check_id;
  //double s_rel_ahead = std::numeric_limits<double>::max();
  double s_rel_ahead = kSensorRange;
  double ref_s_rel;
  std::vector<int> cars_in_check_lane;
  
  // Copy car id's to check lane vector
  if (car_ids_by_lane.count(check_lane) > 0) {
    for (int i = 0; i < car_ids_by_lane.at(check_lane).size(); ++i) {
      cars_in_check_lane.push_back(car_ids_by_lane.at(check_lane)[i]);
    }
  }
  
  // Add ego car id to the check lane for other cars to find
  if ((check_id != ego_car.veh_id_) && (check_lane == ego_car.lane_)) {
    cars_in_check_lane.push_back(ego_car.veh_id_);
  }
  
  // Look for car ahead
  for (int i = 0; i < cars_in_check_lane.size(); ++i) {
    int cur_car_id = cars_in_check_lane[i];
    double cur_s_rel;
    if (cur_car_id != check_id) { // skip checking yourself
      // Set relative S value for the current car in the lane
      if (cur_car_id != ego_car.veh_id_) {
        cur_s_rel = detected_cars.at(cur_car_id).s_rel_;
      }
      else {
        cur_s_rel = 0.; // ego car
      }

      // Set reference relative S value of car doing the check
      if (check_id != ego_car.veh_id_) {
        ref_s_rel = detected_cars.at(check_id).s_rel_;
      }
      else {
        ref_s_rel = 0.; // ego car
      }
      
      // Found closer car ahead if dist is positive and less than prev found
      double cur_dist_ahead = cur_s_rel - ref_s_rel;
      if ((cur_dist_ahead > 0) && (cur_dist_ahead < s_rel_ahead)) {
        s_rel_ahead = cur_dist_ahead;
        car_id_ahead = cur_car_id;
      }
    }
  }
  
  return std::make_tuple(car_id_ahead, s_rel_ahead);
}

std::tuple<int, double> GetCarBehindInLane(const int check_lane, const int check_id,
                                           const EgoVehicle &ego_car,
                                           const std::map<int, DetectedVehicle> &detected_cars,
                                           const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  int car_id_behind = check_id;
  //double s_rel_ahead = -std::numeric_limits<double>::max();
  double s_rel_behind = -kSensorRange;
  double ref_s_rel;
  std::vector<int> cars_in_check_lane;
  
  // Copy car id's to check lane vector
  if (car_ids_by_lane.count(check_lane) > 0) {
    for (int i = 0; i < car_ids_by_lane.at(check_lane).size(); ++i) {
      cars_in_check_lane.push_back(car_ids_by_lane.at(check_lane)[i]);
    }
  }
  
  // Add ego car id to the check lane for other cars to find
  if ((check_id != ego_car.veh_id_) && (check_lane == ego_car.lane_)) {
    cars_in_check_lane.push_back(ego_car.veh_id_);
  }
  
  // Look for car behind
  for (int i = 0; i < cars_in_check_lane.size(); ++i) {
    int cur_car_id = cars_in_check_lane[i];
    double cur_s_rel;
    if (cur_car_id != check_id) { // skip checking yourself
      // Set relative S value for the current car in the lane
      if (cur_car_id != ego_car.veh_id_) {
        cur_s_rel = detected_cars.at(cur_car_id).s_rel_;
      }
      else {
        cur_s_rel = 0.; // ego car
      }
      
      // Set reference relative S value of car doing the check
      if (check_id != ego_car.veh_id_) {
        ref_s_rel = detected_cars.at(check_id).s_rel_;
      }
      else {
        ref_s_rel = 0.; // ego car
      }
      
      // Found closer car behind if dist is positive and less than prev found
      double cur_dist_behind = cur_s_rel - ref_s_rel;
      if ((cur_dist_behind < 0) && (cur_dist_behind > s_rel_behind)) {
        s_rel_behind = cur_dist_behind;
        car_id_behind = cur_car_id;
      }
    }
  }
  
  return std::make_tuple(car_id_behind, s_rel_behind);
}

double EgoCheckSideGap(const VehSides check_side,
                       const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane) {
  double gap_on_side;
  std::vector<int> cars_in_check_lane;
  int car_id_ahead = ego_car.veh_id_;
  int car_id_behind = ego_car.veh_id_;
  double rel_s_ahead;
  double rel_s_behind;
  
  if ((check_side == kRight) && (ego_car.lane_ == kNumLanes)) {
    gap_on_side = 0.;
  }
  else if ((check_side == kLeft) && (ego_car.lane_ == 1)) {
    gap_on_side = 0.;
  }
  else {
    const int check_lane = ego_car.lane_ + int(check_side);
    // Copy car id's to check lane vector
    if (car_ids_by_lane.count(check_lane) > 0) {
      auto car_ahead = GetCarAheadInLane(check_lane, ego_car.veh_id_, ego_car, detected_cars, car_ids_by_lane);
      car_id_ahead = std::get<0>(car_ahead);
      rel_s_ahead = std::get<1>(car_ahead);
      
      auto car_behind = GetCarBehindInLane(check_lane, ego_car.veh_id_, ego_car, detected_cars, car_ids_by_lane);
      car_id_behind = std::get<0>(car_behind);
      rel_s_behind = std::get<1>(car_behind);
      
      if ((abs(rel_s_ahead) < kLaneChangeMinGap)
          || (abs(rel_s_behind) < kLaneChangeMinGap)) {
        gap_on_side = 0.0;
      }
      else {
        gap_on_side = rel_s_ahead - rel_s_behind;
      }
    }
    else {
      // No other cars in right lane
      gap_on_side = kSensorRange;
    }
  }
  
  // DEBUG
  std::cout << "GapOnSide " << check_side << ": " << gap_on_side << ", between car's " << car_id_ahead << " and " << car_id_behind << std::endl;
  
  return gap_on_side;
}
