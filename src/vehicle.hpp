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
  
  // Constructor/Destructor
  Vehicle();
  virtual ~Vehicle();

  void UpdateState(VehState new_state);
  void ClearTraj();
  void SetTraj(VehTrajectory traj);
  void AppendTraj(VehTrajectory traj);
  void SetID(int veh_id);
  int GetID() const;
  int GetLane() const;
  VehState GetState() const;
  VehTrajectory GetTraj() const;
  
private:
  int veh_id_;
  int lane_;
  VehState state_;
  VehTrajectory traj_;
};


class EgoVehicle : public Vehicle {
public:
  
  // Constructor/destructor
  EgoVehicle();
  virtual ~EgoVehicle();
  
  void SetTgtBehavior(VehBehavior new_tgt_beh);
  int GetLaneChangeCounter() const;
  VehBehavior GetTgtBehavior() const;
  
private:
  int counter_lane_change_;
  int prev_tgt_lane_;
  VehBehavior tgt_behavior_;
};

class DetectedVehicle : public Vehicle {
public:
  
  // Constructor/Destructor
  DetectedVehicle();
  virtual ~DetectedVehicle();
  
  void UpdateRelDist(const EgoVehicle &ego_car);
  std::map<VehIntents, VehTrajectory> GetPredTrajs() const;
  void ClearPredTrajs();
  void SetPredTrajs(std::map<VehIntents, VehTrajectory> pred_trajs);
  double GetRelS() const;

private:
  double s_rel_;
  std::map<VehIntents, VehTrajectory> pred_trajs_;
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
