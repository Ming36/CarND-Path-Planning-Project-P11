//
//  behavior.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "behavior.hpp"

/**
 * Cost function to set target lane considering:
 *   #1) Cost by rel_s distance to car ahead
 *   #2) Cost by speed of car ahead
 *   #3) Cost of changing lanes
 *   #4) Cost of frequent lane changes
 * Return best lane # with lowest cost
 */
int LaneCostFcn(const EgoVehicle &ego_car,
                const std::map<int, DetectedVehicle> &detected_cars,
                const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  // Set cost for each lane
  std::vector<double> cost_by_lane;
  for (int lane_idx = 0; lane_idx < kNumLanes; ++lane_idx) {
    const int tgt_lane = lane_idx + 1;
    double lane_cost = 0.;
    
    // #1) Cost by rel_s distance to car ahead
    auto car_ahead = FindCarInLane(kFront, tgt_lane, ego_car.GetID(), ego_car,
                                   detected_cars, car_ids_by_lane);
    int car_id_ahead = std::get<0>(car_ahead);
    double rel_s_ahead = kSensorRange;
    if (car_id_ahead != ego_car.GetID()) {
      rel_s_ahead = detected_cars.at(car_id_ahead).GetRelS();
    }
    const double cost_ahead_dist = (1-LogCost(rel_s_ahead, kSensorRange));
    lane_cost += kCostDistAhead * cost_ahead_dist;
    
    // #2) Cost by speed of car ahead
    double s_dot_ahead = kTargetSpeed;
    if (car_id_ahead != ego_car.GetID()) {
      s_dot_ahead = detected_cars.at(car_id_ahead).GetState().s_dot;
    }
    const double cost_ahead_spd = (1-LogCost(s_dot_ahead, kTargetSpeed));
    lane_cost += kCostSpeedAhead * cost_ahead_spd;
    
    // #3) Cost of changing lanes
    if (tgt_lane != ego_car.GetLane()) {
      const double cost_lane_change = (abs(ego_car.GetLane() - tgt_lane));
      lane_cost += kCostChangeLanes * cost_lane_change;
    }
    
    // #4) Cost of frequent lane changes
    if ((ego_car.GetLaneChangeCounter() > 0)
        && (tgt_lane != ego_car.GetTgtBehavior().tgt_lane)) {
      lane_cost += kCostFreqLaneChange * ego_car.GetLaneChangeCounter();
    }
    
    // Save total cost for this lane to vector
    cost_by_lane.push_back(lane_cost);
    
    // Debug logging
    if (kDBGBehavior != 0) {
      std::cout << "Cost function lane: " << tgt_lane << ", cost: "
      << lane_cost << std::endl;
    }
  }
  
  // Choose lowest cost lane and set target behaviors
  auto it_best_cost = min_element(cost_by_lane.begin(), cost_by_lane.end());
  int best_lane = (it_best_cost - cost_by_lane.begin()) + 1;
  
  return best_lane;
}

/**
 * Finite State Machine for behavior intents.  Possible intents are:
 *   1. Keep Lane (KL)
 *   2. Plan Lane Change Left (PLCL)
 *   3. Plan Lane Change Right (PLCR)
 *   4. Lane Change Left (LCL)
 *   5. Lane Change Right (LCR)
 * Returns the target intent.
 */
VehIntents BehaviorFSM(const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  VehIntents tgt_intent;
  const double gap_on_left = EgoCheckSideGap(kLeft, ego_car, detected_cars,
                                             car_ids_by_lane);
  const double gap_on_right = EgoCheckSideGap(kRight, ego_car, detected_cars,
                                              car_ids_by_lane);
  const VehBehavior ego_beh = ego_car.GetTgtBehavior();
  const VehIntents cur_intent = ego_beh.intent;
  const int cur_tgt_lane = ego_beh.tgt_lane;
  const int ego_lane = ego_car.GetLane();
  
  // Behavior Intent FSM
  switch (cur_intent) {
    case kKeepLane: {
      if (cur_tgt_lane < ego_lane) {
        tgt_intent = kPlanLaneChangeLeft; // tgt lane on left, plan LC
      }
      else if (cur_tgt_lane > ego_lane) {
        tgt_intent = kPlanLaneChangeRight; // tgt lane on right, plan LC
      }
      else {
        tgt_intent = kKeepLane; // stay in KL
      }
      break;
    }
    case kPlanLaneChangeLeft: {
      if (gap_on_left > kLaneChangeMinGap) {
        tgt_intent = kLaneChangeLeft; // lane on left is clear, change lanes
      }
      else if (cur_tgt_lane < ego_lane) {
        tgt_intent = kPlanLaneChangeLeft; // stay in PLCL
      }
      else {
        tgt_intent = kKeepLane; // go back to keep lane
      }
      break;
    }
    case kPlanLaneChangeRight: {
      if (gap_on_right > kLaneChangeMinGap) {
        tgt_intent = kLaneChangeRight; // lane on right is clear, change lanes
      }
      else if (cur_tgt_lane > ego_lane) {
        tgt_intent = kPlanLaneChangeRight; // stay in PLCR
      }
      else {
        tgt_intent = kKeepLane; // go back to keep lane
      }
      break;
    }
    case kLaneChangeLeft: {
      if (cur_tgt_lane < ego_lane) {
        tgt_intent = kLaneChangeLeft; // stay in LCL
      }
      else {
        tgt_intent = kKeepLane; // go back to keep lane
      }
      break;
    }
    case kLaneChangeRight: {
      if (cur_tgt_lane > ego_lane) {
        tgt_intent = kLaneChangeRight; // stay in LCR
      }
      else {
        tgt_intent = kKeepLane; // go back to keep lane
      }
      break;
    }
    default: {
      //assert(false);
      tgt_intent = cur_intent; // pass-through, should not occur
    }
  }
  
  return tgt_intent;
}

/**
 * Set behavior target speed based on the target intent
 */
double SetTargetSpeed(const EgoVehicle &ego_car,
                      const std::map<int, DetectedVehicle> &detected_cars,
                      const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  // Set initial target speed to base target speed parameter
  double target_speed = kTargetSpeed;
  
  // Limit target speed based on following car ahead while Keeping Lane
  target_speed = TargetSpeedKL(target_speed, ego_car, detected_cars,
                               car_ids_by_lane);
  
  // Over-ride target speed if planning to change lanes with cars in the way
  if (ego_car.GetTgtBehavior().intent == kPlanLaneChangeLeft) {
    target_speed = TargetSpeedPLC(kLeft, target_speed, ego_car, detected_cars,
                                  car_ids_by_lane);
  }
  else if (ego_car.GetTgtBehavior().intent == kPlanLaneChangeRight) {
    target_speed = TargetSpeedPLC(kRight, target_speed, ego_car, detected_cars,
                                  car_ids_by_lane);
  }
  
  // Final min/max guard
  target_speed = std::max(target_speed, 0.0);
  target_speed = std::min(target_speed, kTargetSpeed);
  
  return target_speed;
}

/**
 * Set target speed for Keep Lane intent (follow car ahead)
 */
double TargetSpeedKL(double base_tgt_spd, const EgoVehicle &ego_car,
                     const std::map<int, DetectedVehicle> &detected_cars,
                     const std::map<int, std::vector<int>> &car_ids_by_lane) {

  double tgt_speed = base_tgt_spd; // default is to pass-through base speed

  // Check for closest car ahead in current lane
  const auto car_ahead = FindCarInLane(kFront, ego_car.GetLane(), ego_car.GetID(),
                                       ego_car, detected_cars, car_ids_by_lane);
  const int car_id_ahead = std::get<0>(car_ahead);
  const double rel_s_ahead = std::get<1>(car_ahead);
  
  // Set base target speed limited by car ahead
  if (car_id_ahead >= 0) {
    if (detected_cars.at(car_id_ahead).GetRelS() < kTgtStartFollowDist) {
      
      // Decrease target speed proportionally to distance of car ahead
      const double dist_ahead = detected_cars.at(car_id_ahead).GetRelS();
      const double spd_ahead = detected_cars.at(car_id_ahead).GetState().s_dot;
      
      const double spd_slope = ((spd_ahead - kTargetSpeed)
                                / (kTgtFollowDist - kTgtStartFollowDist));
      
      tgt_speed = spd_slope * (dist_ahead - kTgtStartFollowDist) + kTargetSpeed;
      
      // Decrease target speed by a fixed decrement if too close
      if (detected_cars.at(car_id_ahead).GetRelS() < kTgtMinFollowDist) {
        tgt_speed = spd_ahead - kMinFollowTgtSpeedDec;
      }
      
      // Debug logging
      if (kDBGBehavior != 0) {
        std::cout << "\nBase target speed = " << mps2mph(tgt_speed)
        << " car ahead id# " << car_id_ahead << " rel_s = " << rel_s_ahead
        << std::endl;
      }
      
      // Min guard target speed to avoid stopping on the freeway
      tgt_speed = std::max(tgt_speed, kTgtMinSpeed);
    }
  }
  
  return tgt_speed;
}

/**
 * Set target speed for Plan Lane Change Left/Right intents (look for gap)
 */
double TargetSpeedPLC(VehSides sidePLC, double base_tgt_spd,
                      const EgoVehicle &ego_car,
                      const std::map<int, DetectedVehicle> &detected_cars,
                      const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  double target_speed = base_tgt_spd; // default is to pass-through base speed
  const int check_lane = ego_car.GetLane() + sidePLC;
  
  // Check for closest car ahead in current lane
  const auto car_ahead = FindCarInLane(kFront, ego_car.GetLane(), ego_car.GetID(),
                                       ego_car, detected_cars, car_ids_by_lane);
  const int car_id_ahead = std::get<0>(car_ahead);
  const double rel_s_ahead = std::get<1>(car_ahead);
  
  // Look for car ahead in lane on PLC side
  const auto car_side_ahead = FindCarInLane(kFront, check_lane, ego_car.GetID(),
                                            ego_car, detected_cars,
                                            car_ids_by_lane);
  const int car_id_side_ahead = std::get<0>(car_side_ahead);
  const double rel_s_side_ahead = std::get<1>(car_side_ahead);
  
  // Look for car behind in lane on PLC side
  const auto car_side_behind = FindCarInLane(kBack, check_lane, ego_car.GetID(),
                                             ego_car, detected_cars,
                                             car_ids_by_lane);
  const int car_id_side_behind = std::get<0>(car_side_behind);
  const double rel_s_side_behind = std::get<1>(car_side_behind);
  
  // Check which cars are close
  bool close_ahead = false;
  bool close_side_ahead = false;
  bool close_side_behind = false;
  if ((detected_cars.count(car_id_ahead) > 0)
      && (rel_s_ahead < kTgtStartFollowDist)) {
    close_ahead = true;
  }
  if ((detected_cars.count(car_id_side_ahead) > 0)
      && (rel_s_side_ahead < kLaneChangeMinGap)) {
    close_side_ahead = true;
  }
  if ((detected_cars.count(car_id_side_behind) > 0)
      && (abs(rel_s_side_behind) < kLaneChangeMinGap)) {
    close_side_behind = true;
  }
  
  // Slow down to find a gap if car ahead is close
  if (close_ahead == true) {
    if ((close_side_ahead == true) && (close_side_behind == true)) {
      
      // Set tgt speed slower than close car behind (close car ahead and behind)
      target_speed = (detected_cars.at(car_id_side_behind).GetState().s_dot
                      - kPlanLCTgtSpeedDec);
      
      // Debug logging
      if (kDBGBehavior != 0) {
        std::cout << " Over-ride tgt speed to car on side behind #"
        << car_id_side_behind << " = " << mps2mph(target_speed) << std::endl;
      }
    }
    else if (close_side_ahead) {
      // Set target speed slower than close car ahead (no close car behind)
      target_speed = (detected_cars.at(car_id_side_ahead).GetState().s_dot
                      - kPlanLCTgtSpeedDec);
      
      // Debug logging
      if (kDBGBehavior != 0) {
        std::cout << " Over-ride target speed to car on side ahead #"
        << car_id_side_ahead << " = " << mps2mph(target_speed) << std::endl;
      }
    }
  }
  // Otherwise, just keep going ahead at original target speed to pass
  
  return target_speed;
}
