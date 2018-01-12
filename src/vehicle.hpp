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

class Vehicle {
public:
  /*
   std::map<std::string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
   {"LCR", -1}, {"PLCR", -1}};
   */
  
  int veh_id_;
  int lane_;
  double x_;
  double y_;
  
  double s_;
  double s_dot_;
  double s_dot_dot_;
  
  double s_prev_;
  double s_dot_prev_;
  
  double d_;
  double d_dot_;
  double d_dot_dot_;

  double d_prev_;
  double d_dot_prev_;
  
  std::string intent_; // "KL", "LCL", "LCR"
  std::vector<double*> state_; // array of pointers to current state values
  std::vector<double> trajectory_s_;
  std::vector<double> trajectory_d_;
  
  /**
   * Constructor
   */
  Vehicle(int veh_id);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
  void UpdateState(double x, double y, double s, double d, double s_dot, double d_dot);
  
};


class EgoVehicle : public Vehicle {
public:
  
  std::vector<double> coeffs_JMT_; // [a0, a1, a2, a3, a4, a5]
  std::vector<double> coeffs_JMT_dot_; // [a0, a1, a2, a3, a4, a5]
  std::vector<double> coeffs_JMT_dot_dot_; // [a0, a1, a2, a3, a4, a5]

  /**
   * Constructor
   */
  EgoVehicle(int id = -1);
  
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
  DetectedVehicle(int id);
  
  /**
   * Destructor
   */
  virtual ~DetectedVehicle();
  
  void UpdateRelDist(double s_ego, double d_ego);
};

#endif /* vehicle_hpp */
