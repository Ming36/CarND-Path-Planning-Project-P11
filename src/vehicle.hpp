//
//  vehicle.hpp
//  Path_Planning
//
//  Created by Student on 1/11/18.
//

#ifndef vehicle_hpp
#define vehicle_hpp

#include <stdio.h>

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include "path_common.hpp"

enum VehIntents {
  kUnknown = -1,
  kKeepLane = 0,
  kPlanLaneChangeLeft = 1,
  kPlanLaneChangeRight = 2,
  kLaneChangeLeft = 3,
  kLaneChangeRight = 4
};

struct VehBehavior {
  VehIntents intent;
  int car_ahead_id;
  int target_lane;
  double target_time;
  std::vector<double> target_state_vals;
};

struct VehTrajectory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> t;
  double probability;
};

class Vehicle {
public:
  
  int veh_id_;
  int lane_;
  double x_;
  double y_;
  
  double s_;
  double s_dot_;
  double s_dot_dot_;
  
  double d_;
  double d_dot_;
  double d_dot_dot_;

  VehIntents intent_;
  VehTrajectory trajectory_;
  
  /**
   * Constructor
   */
  Vehicle();
  Vehicle(int veh_id);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
  void UpdateState(double x, double y, double s, double d,
                   double s_dot, double d_dot,
                   double s_dot_dot, double d_dot_dot);
  
};


class EgoVehicle : public Vehicle {
public:
  
  VehBehavior behavior_;
  
  std::vector<double> coeffs_JMT_s_; // [a0, a1, a2, a3, a4, a5]
  std::vector<double> coeffs_JMT_s_dot_; // [a1, 2*a2, 3*a3, 4*a4, 5*a5]
  std::vector<double> coeffs_JMT_s_dot_dot_; // [2*a2, 6*a3, 12*a4, 20*a5]
  
  std::vector<double> coeffs_JMT_d_; // [a0, a1, a2, a3, a4, a5]
  std::vector<double> coeffs_JMT_d_dot_; // [a1, 2*a2, 3*a3, 4*a4, 5*a5]
  std::vector<double> coeffs_JMT_d_dot_dot_; // [2*a2, 6*a3, 12*a4, 20*a5]

  /**
   * Constructor
   */
  EgoVehicle();
  EgoVehicle(int id);
  
  /**
   * Destructor
   */
  virtual ~EgoVehicle();
};

class DetectedVehicle : public Vehicle {
public:
  
  double s_rel_;
  double d_rel_;
  
  /**
   * Constructor
   */
  DetectedVehicle();
  DetectedVehicle(int id);
  
  /**
   * Destructor
   */
  virtual ~DetectedVehicle();
  
  void UpdateRelDist(double s_ego, double d_ego);
};

#endif /* vehicle_hpp */
