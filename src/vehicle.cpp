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
}

/**
 * Destructor
 */
Vehicle::~Vehicle() { }

/**
 * Updates vehicle's state values
 */
void Vehicle::UpdateState(double x, double y, double s, double d, double s_dot, double d_dot, double s_dot_dot, double d_dot_dot) {
  
  // Update state values
  x_ = x;
  y_ = y;
  s_ = s;
  s_dot_ = s_dot;
  s_dot_dot_ = s_dot_dot;
  d_ = d;
  d_dot_ = d_dot;
  d_dot_dot_ = d_dot_dot;
  
  // Detect lane
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
  coeffs_JMT_s_ = {0, 0, 0};
  coeffs_JMT_s_dot_ = {0, 0, 0};
  coeffs_JMT_s_dot_dot_ = {0, 0, 0};
  coeffs_JMT_d_ = {0, 0, 0};
  coeffs_JMT_d_dot_ = {0, 0, 0};
  coeffs_JMT_d_dot_dot_ = {0, 0, 0};
}

EgoVehicle::EgoVehicle(int id) : Vehicle(id) {
  coeffs_JMT_s_ = {0, 0, 0};
  coeffs_JMT_s_dot_ = {0, 0, 0};
  coeffs_JMT_s_dot_dot_ = {0, 0, 0};
  coeffs_JMT_d_ = {0, 0, 0};
  coeffs_JMT_d_dot_ = {0, 0, 0};
  coeffs_JMT_d_dot_dot_ = {0, 0, 0};
}

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
