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

  /*
   1. Make cost function for each lane to rank which is the best lane to be in
        #1) Cost by rel_s distance to car ahead
        #2) Cost by speed of car ahead
        #3) Cost by high speed of close car behind ego car (avoid getting rear ended)
        #4) Cost of changing lanes
        #5) Cost of frequent lane changes
   2. Set tgt_behavior intent, tgt_lane, tgt_time, and tgt_speed to get to that lane
   */

  // Set cost for each lane
  std::vector<double> cost_by_lane;
  for (int lane_idx = 0; lane_idx < kNumLanes; ++lane_idx) {
    const int tgt_lane = lane_idx + 1;
    double lane_cost = 0.;
    
    // #1) Cost by rel_s distance to car ahead
    auto car_ahead = GetCarAheadInLane(tgt_lane, ego_car.veh_id_, ego_car,
                                       detected_cars, car_ids_by_lane);
    int car_id_ahead = std::get<0>(car_ahead);
    double rel_s_ahead = kSensorRange;
    if (car_id_ahead != ego_car.veh_id_) {
      rel_s_ahead = detected_cars.at(car_id_ahead).s_rel_;
    }
    lane_cost += kCostDistAhead * (1-LogCost(rel_s_ahead, kSensorRange));
    
    // #2) Cost by speed of car ahead
    double s_dot_ahead = kTargetSpeed;
    if (car_id_ahead != ego_car.veh_id_) {
      s_dot_ahead = detected_cars.at(car_id_ahead).state_.s_dot;
    }
    lane_cost += kCostSpeedAhead * (1-LogCost(s_dot_ahead, kTargetSpeed));
 
    // #3) Cost by high speed of close car behind ego car
    if (tgt_lane == ego_car.lane_) {
      auto car_behind = GetCarBehindInLane(ego_car.lane_, ego_car.veh_id_, ego_car,
                                           detected_cars, car_ids_by_lane);
      int car_id_behind = std::get<0>(car_behind);
      double rel_s_behind = std::get<1>(car_behind);
      
      if (car_id_behind != ego_car.veh_id_) {
        double s_dot_behind = detected_cars.at(car_id_behind).state_.s_dot;
        if ((rel_s_behind > -kTgtMinFollowDist)
            && (s_dot_behind > ego_car.state_.s_dot)) {
          lane_cost += kCostSpeedBehind;
        }
      }
    }
    
    // #4) Cost of changing lanes
    if (tgt_lane != ego_car.lane_) {
      lane_cost += kCostChangeLanes * (abs(ego_car.lane_ - tgt_lane));
    }
    
    // #5) Cost of frequent lane changes
    if ((ego_car.counter_lane_change > 0) && (tgt_lane != ego_car.tgt_behavior_.tgt_lane)) {
      lane_cost += kCostFreqLaneChange * ego_car.counter_lane_change;
    }
    
    cost_by_lane.push_back(lane_cost);
    
    // DEBUG
    std::cout << "Cost function lane: " << tgt_lane << ", cost: " << lane_cost << std::endl;
  }
  
  // Choose lowest cost lane and set target behaviors
  auto it_best_cost = min_element(cost_by_lane.begin(), cost_by_lane.end());
  int best_lane = (it_best_cost - cost_by_lane.begin()) + 1;
  
  // Update frequent lane change counter
  if (ego_car.counter_lane_change > 0) {
    ego_car.counter_lane_change--;
  }
  if (best_lane != ego_car.tgt_behavior_.tgt_lane) {
    // Targeting a new lane change
    ego_car.counter_lane_change = kCounterFreqLaneChange;
  }
  
  // Set target behavior
  ego_car.tgt_behavior_.tgt_lane = best_lane;
  int num_lanes_to_change = abs(ego_car.lane_ - ego_car.tgt_behavior_.tgt_lane);
  if (num_lanes_to_change > 1) {
    ego_car.tgt_behavior_.tgt_time = kNewPathTime * num_lanes_to_change;
  }
  else {
    ego_car.tgt_behavior_.tgt_time = kNewPathTime;
  }
  
  if (best_lane < ego_car.lane_) {
    ego_car.tgt_behavior_.intent = kPlanLaneChangeLeft;
  }
  else if (best_lane > ego_car.lane_) {
    ego_car.tgt_behavior_.intent = kPlanLaneChangeRight;
  }
  else {
    ego_car.tgt_behavior_.intent = kKeepLane;
  }

  double target_speed = kTargetSpeed;
  
  // Check for closest car ahead in current lane
  auto car_ahead = GetCarAheadInLane(ego_car.lane_, ego_car.veh_id_, ego_car,
                                     detected_cars, car_ids_by_lane);
  int car_id_ahead = std::get<0>(car_ahead);
  double rel_s_ahead = std::get<1>(car_ahead);
  
  // Set target speed based on car ahead
  if (car_id_ahead >= 0) {
    //const DetectedVehicle* car_ahead = &detected_cars.at(car_id_ahead);
    
    if (detected_cars.at(car_id_ahead).s_rel_ < kTgtMinFollowDist) {
      target_speed = detected_cars.at(car_id_ahead).state_.s_dot - kTgtSpeedDec;
    }
    else if (detected_cars.at(car_id_ahead).s_rel_ < kTgtFollowDist) {
      target_speed = detected_cars.at(car_id_ahead).state_.s_dot;
    }
  }
  
  // Final min/max guard
  if (target_speed > kTargetSpeed) { target_speed = kTargetSpeed; }
  else if (target_speed < 0.) { target_speed = 0.; }
  
  ego_car.tgt_behavior_.tgt_speed = target_speed;
  
  // DEBUG
  std::cout << "Behavior: " << std::endl;
  std::cout << " intent: " << ego_car.tgt_behavior_.intent << ", target lane: " << best_lane << ", cost: " << *it_best_cost << std::endl;
  std::cout << " tgt_speed (mph) = " << mps2mph(ego_car.tgt_behavior_.tgt_speed) << " car ahead id# " << car_id_ahead << " rel_s = " << rel_s_ahead << std::endl;
  std::cout << " freq lane change counter: " << ego_car.counter_lane_change << std::endl;
  std::cout << std::endl;
}



/*
void VehBehaviorFSMold(EgoVehicle &ego_car,
                    const std::map<int, DetectedVehicle> &detected_cars,
                    const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  // KL: Behavior to keep current lane
  ego_car.tgt_behavior_.intent = kKeepLane;
  ego_car.tgt_behavior_.tgt_lane = ego_car.lane_;
  ego_car.tgt_behavior_.tgt_time = kNewPathTime;
  
  double target_speed = kTargetSpeed;
  
  // Check for closest car ahead in current lane
  auto car_ahead = GetCarAheadInLane(ego_car.lane_, ego_car.veh_id_, ego_car,
                                     detected_cars, car_ids_by_lane);
  int car_id_ahead = std::get<0>(car_ahead);
  double rel_s_ahead = std::get<1>(car_ahead);
  
  // Set target speed based on car ahead
  if (car_id_ahead >= 0) {
    const DetectedVehicle* car_ahead = &detected_cars.at(car_id_ahead);

    if (car_ahead->s_rel_ < kTgtMinFollowDist) {
      target_speed = car_ahead->state_.s_dot - kTgtSpeedDec;
    }
    else if (car_ahead->s_rel_ < kTgtFollowDist) {
      target_speed = car_ahead->state_.s_dot;
    }
    std::cout << "Car ahead id# " << car_id_ahead << " rel_s = " << rel_s_ahead << std::endl;
  }
  
  // Min/max guard
  if (target_speed > kTargetSpeed) { target_speed = kTargetSpeed; }
  else if (target_speed < 0.) { target_speed = 0.; }
  
  ego_car.tgt_behavior_.tgt_speed = target_speed;
  
  // DEBUG
  std::cout << "tgt_speed (mph) = " << mps2mph(ego_car.tgt_behavior_.tgt_speed) << std::endl;
  
  // DUMMY lane change behavior
  auto car_behind = GetCarBehindInLane(ego_car.lane_, ego_car.veh_id_, ego_car,
                                       detected_cars, car_ids_by_lane);
  int car_id_behind = std::get<0>(car_behind);
  double rel_s_behind = std::get<1>(car_behind);
  bool ReqLC = false;
  if (car_id_behind >= 0) {
    std::cout << "Car behind id# " << car_id_behind << " rel_s = " << rel_s_behind << std::endl;
    if ((abs(rel_s_behind) < kTgtMinFollowDist)
        && (detected_cars.at(car_id_behind).state_.s_dot > ego_car.state_.s_dot)) {
      ReqLC = true;
    }
  }
  
  // Change lanes if car ahead is too slow or car behind is coming up too fast
  if (((ego_car.state_.s_dot < kTargetSpeed - mph2mps(10))
       && (rel_s_ahead < kTgtFollowDist))
      || ReqLC) {
    if (ego_car.lane_ > 1) {
      ego_car.tgt_behavior_.tgt_lane--; // DUMMY
    }
    else if (ego_car.lane_ < kNumLanes) {
      ego_car.tgt_behavior_.tgt_lane++; // DUMMY
    }
    
  }
  
}
*/
