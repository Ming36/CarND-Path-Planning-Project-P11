//
//  behavior.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "behavior.hpp"



VehBehavior VehBehaviorFSM(EgoVehicle ego_car,
                           std::vector<DetectedVehicle> veh_preds_lanetoleft,
                           std::vector<DetectedVehicle> veh_preds_curlane,
                           std::vector<DetectedVehicle> veh_preds_lanetoright) {
  
  VehBehavior target_behavior;
  target_behavior.intent = kKeepLane;
  target_behavior.target_lane = ego_car.lane_; // DUMMY
  target_behavior.target_state_vals = {30, 0, 0, 6, 0, 0}; // DUMMY
  return target_behavior;
}
