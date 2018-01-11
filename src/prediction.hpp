//
//  prediction.hpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#ifndef prediction_hpp
#define prediction_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

class Vehicle {
public:
  /*
  std::map<std::string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                               {"LCR", -1}, {"PLCR", -1}};
  */
  
  int veh_id;
  int lane;
  double x;
  double y;
  
  double s;
  double s_dot;
  double s_dot_dot;
  double s_rel;
  double s_prev;
  double s_dot_prev;

  double d;
  double d_dot;
  double d_dot_dot;
  double d_rel;
  double d_prev;
  double d_dot_prev;

  std::string intent; // "KL", "LCL", "LCR"
  std::vector<double> state; // [s, s_dot, s_dot_dot, d, d_dot, d_dot_dot]
  std::vector<double> trajectory_s;
  std::vector<double> trajectory_d;

  /**
   * Constructor
   */
  Vehicle(int veh_id);
  
  /**
   * Destructor
   */
  virtual ~Vehicle();
  
  void UpdateState(double x, double y, double s, double d, double s_dot, double d_dot, double ego_s, double ego_d);

  void PredictBehavior();
  void PredictTrajectory();

  /*
  std::vector<Vehicle> choose_next_state(std::map<int, std::vector<Vehicle>> predictions);
  
  std::vector<std::string> successor_states();
  
  std::vector<Vehicle> generate_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
  
  std::vector<float> get_kinematics(std::map<int, std::vector<Vehicle>> predictions, int lane);
  
  std::vector<Vehicle> constant_speed_trajectory();
  
  std::vector<Vehicle> keep_lane_trajectory(std::map<int, std::vector<Vehicle>> predictions);
  
  std::vector<Vehicle> lane_change_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
  
  std::vector<Vehicle> prep_lane_change_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
  
  void increment(int dt);
  
  float position_at(int t);
  
  bool get_vehicle_behind(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  bool get_vehicle_ahead(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  std::vector<Vehicle> generate_predictions(int horizon=2);
  
  void realize_next_state(std::vector<Vehicle> trajectory);
  
  void configure(std::vector<int> road_data);

  float calculate_cost(const Vehicle & vehicle, const std::map<int, std::vector<Vehicle>> & predictions, const std::vector<Vehicle> & trajectory);
  */
};

#endif /* prediction_hpp */
