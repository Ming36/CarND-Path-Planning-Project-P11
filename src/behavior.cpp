//
//  behavior.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "behavior.hpp"

void VehBehaviorFSM(EgoVehicle &ego_car,
                    const std::map<int, DetectedVehicle> &detected_cars,
                    const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  ego_car.tgt_behavior_.intent = kKeepLane;
  ego_car.tgt_behavior_.tgt_lane = ego_car.lane_; // DUMMY
  ego_car.tgt_behavior_.tgt_time = kPredictTime;

  if (ego_car.state_.s > 220) {
    ego_car.tgt_behavior_.tgt_lane = 3; // DUMMY
  }
}
