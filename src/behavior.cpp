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
    else if (ego_car.lane_ < 3) {
      ego_car.tgt_behavior_.tgt_lane++; // DUMMY
    }
    
  }
  
}
