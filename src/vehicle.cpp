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
  
  intent_ = kUnknown;
}
Vehicle::Vehicle(int veh_id) {
  veh_id_ = veh_id;
  intent_ = kUnknown;
  traj_.coeffs_JMT_s_ = {0, 0, 0};
  traj_.coeffs_JMT_s_dot_ = {0, 0, 0};
  traj_.coeffs_JMT_s_dotdot_ = {0, 0, 0};
  traj_.coeffs_JMT_d_ = {0, 0, 0};
  traj_.coeffs_JMT_d_dot_ = {0, 0, 0};
  traj_.coeffs_JMT_d_dotdot_ = {0, 0, 0};
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
EgoVehicle::EgoVehicle() : Vehicle() {
  //tgt_behavior_.tgt_speed = mph2mps(kTargetSpeedMPH);
}

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
  s_rel_ = state_.s - s_ego;
  d_rel_ = state_.d - d_ego;
}
