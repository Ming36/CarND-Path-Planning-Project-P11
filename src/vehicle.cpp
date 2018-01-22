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

std::tuple<int, double> GetCarAheadInLane(const int check_lane, const double s_rel_ref,
                                          const std::map<int, DetectedVehicle> &detected_cars,
                                          const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  int car_id_ahead = -1;
  double s_rel_ahead = std::numeric_limits<double>::max();
  
  if (car_ids_by_lane.count(check_lane) > 0) {
    auto car_ids_in_lane = car_ids_by_lane.at(check_lane);
    for (int i = 0; i < car_ids_in_lane.size(); ++i) {
      int cur_car_id = car_ids_in_lane[i];
      double cur_s_rel = detected_cars.at(cur_car_id).s_rel_;
      if (cur_s_rel < s_rel_ref) {
        break;
      }
      car_id_ahead = cur_car_id;
      s_rel_ahead = cur_s_rel;
    }
  }
  
  return std::make_tuple(car_id_ahead, s_rel_ahead);
}

std::tuple<int, double> GetCarBehindInLane(const int check_lane, const double s_rel_ref,
                                           const std::map<int, DetectedVehicle> &detected_cars,
                                           const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  int car_id_behind = -1;
  double s_rel_behind = std::numeric_limits<double>::min();
  
  if (car_ids_by_lane.count(check_lane) > 0) {
    auto car_ids_in_lane = car_ids_by_lane.at(check_lane);
    for (int i = car_ids_in_lane.size()-1; i >= 0; --i) {
      int cur_car_id = car_ids_in_lane[i];
      double cur_s_rel = detected_cars.at(cur_car_id).s_rel_;
      if (cur_s_rel > s_rel_ref) {
        break;
      }
      car_id_behind = cur_car_id;
      s_rel_behind = cur_s_rel;
    }
  }
  
  return std::make_tuple(car_id_behind, s_rel_behind);
}
