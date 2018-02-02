//
//  vehicle.hpp
//  Path_Planning
//
//  Created by Student on 1/11/18.
//

#ifndef vehicle_hpp
#define vehicle_hpp

#include <stdio.h>
#include "path_common.hpp"

enum VehSides {
  kLeft = -1,
  kRight = 1,
  kBack = -2,
  kFront = 2
};

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
  int tgt_lane;
  double tgt_speed;
  double tgt_time;
};

struct VehState {
  double x;
  double y;
  double s;
  double s_dot;
  double s_dotdot;
  double d;
  double d_dot;
  double d_dotdot;
};

struct VehTrajectory {
  std::deque<VehState> states;
  double probability;
  double cost;
};

class Vehicle {
public:
  int veh_id_;
  int lane_;
  VehState state_;
  VehTrajectory traj_;  

  // Constructor/Destructor
  Vehicle();
  virtual ~Vehicle();

  void UpdateStateAndLane(VehState new_state);
};


class EgoVehicle : public Vehicle {
public:
  
  int counter_lane_change;
  VehBehavior tgt_behavior_;
  
  // Constructor/destructor
  EgoVehicle();
  virtual ~EgoVehicle();
};

class DetectedVehicle : public Vehicle {
public:
  
  double s_rel_;
  double d_rel_;
  std::map<VehIntents, VehTrajectory> pred_trajs_;
  
  // Constructor/Destructor
  DetectedVehicle();
  virtual ~DetectedVehicle();
  
  void UpdateRelDist(double s_ego, double d_ego);
};

std::tuple<int, double> FindCarInLane(const VehSides check_side,
                        const int check_lane, const int check_id,
                        const EgoVehicle &ego_car,
                        const std::map<int, DetectedVehicle> &detected_cars,
                        const std::map<int, std::vector<int>> &car_ids_by_lane);
  
double EgoCheckSideGap(const VehSides check_side,
                       const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane);

#endif /* vehicle_hpp */
