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
  //state_vals_ = {&s_, &s_dot_, &s_dot_dot_, &d_, &d_dot_, &d_dot_dot_};
}

/**
 * Destructor
 */
Vehicle::~Vehicle() { }

/**
 * Updates vehicle's state values
 */
void Vehicle::UpdateState(double x, double y, double s, double d, double s_dot, double d_dot) {
  
  // Store previous values to calculate s_dot_dot and d_dot_dot later
  s_prev_ = s_;
  s_dot_prev_ = s_dot_;
  d_prev_ = d_;
  d_dot_prev_ = d_dot_;
  
  // Update state values
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  s_dot_ = s_dot;
  d_dot_ = d_dot;
  
  // Calculate s_dot_dot and d_dot_dot from stored previous data
  // Equation: s_dot_dot = s_dot_prev*(s_dot - s_dot_prev) + 0.5*(s_dot - s_dot_prev)^2 / (s - s_prev)
  if ((s - s_prev_) > 0.0001) {
    s_dot_dot_ = (s_dot_prev_ * (s_dot - s_dot_prev_)
                  + 0.5 * pow(s_dot - s_dot_prev_, 2) / (s - s_prev_));
  }
  else {
    s_dot_dot_ = 0;
  }
  
  if ((d - d_prev_) > 0.0001) {
    d_dot_dot_ = (d_dot_prev_ * (d_dot - d_dot_prev_)
                  + 0.5 * pow(d_dot - d_dot_prev_, 2) / (d - d_prev_));
  }
  else {
    d_dot_dot_ = 0;
  }
  
  // Detect lane
  int lane = 0;
  double n = 0;
  constexpr double kLaneWidth = 4.0; // m
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
  s_rel_ = s_ - s_ego;
  d_rel_ = d_ - d_ego;
}
