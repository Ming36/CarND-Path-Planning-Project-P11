//
//  vehicle.cpp
//  Path_Planning
//
//  Created by Student on 1/11/18.
//

#include "vehicle.hpp"

//// Vehicle base class ////

// Constructor/Destructor
Vehicle::Vehicle() { }
Vehicle::~Vehicle() { }

/**
 * Update vehicle's state values and calculate its new lane position
 */
void Vehicle::UpdateState(VehState new_state) {
  
  // Update state values
  state_ = new_state;
  
  // Detect lane number (1 is closest to middle of road)
  int lane = 0;
  double n = 0;
  while (new_state.d > n * kLaneWidth) {
    lane++;
    n = n + 1.0;
  }
  lane_ = lane;
}

int Vehicle::GetID() const { return veh_id_; }

void Vehicle::SetID(int veh_id) { veh_id_ = veh_id; }

int Vehicle::GetLane() const { return lane_; }

VehState Vehicle::GetState() const { return state_; }

VehTrajectory Vehicle::GetTraj() const { return traj_; }

void Vehicle::ClearTraj() {
  traj_.states.clear();
  traj_.probability = 0.;
  traj_.cost = 0.;
}

void Vehicle::SetTraj(VehTrajectory traj) { traj_ = traj; }

void Vehicle::AppendTraj(VehTrajectory traj) {
  for (int i = 0; i < traj.states.size(); ++i) {
    traj_.states.push_back(traj.states[i]);
  }
}


//// EgoVehicle sub-class ////

// Constructor/Destructor
EgoVehicle::EgoVehicle() : Vehicle() {
  counter_lane_change_ = 0;
  tgt_behavior_.intent = kKeepLane;
}
EgoVehicle::~EgoVehicle() { }

int EgoVehicle::GetLaneChangeCounter() const { return counter_lane_change_; }

VehBehavior EgoVehicle::GetTgtBehavior() const { return tgt_behavior_; }

void EgoVehicle::SetTgtBehavior(VehBehavior new_tgt_beh) {
  
  // Store previous target lane
  prev_tgt_lane_ = tgt_behavior_.tgt_lane;

  // Set new target behavior
  tgt_behavior_ = new_tgt_beh;
  
  // Reset lane counter when target lane changes or while changing lanes
  int counter = counter_lane_change_;
  if (counter > 0) {
    counter--; // count down
  }
  if ((tgt_behavior_.tgt_lane != prev_tgt_lane_)
      || (tgt_behavior_.intent == kLaneChangeLeft)
      || (tgt_behavior_.intent == kLaneChangeRight)) {
    counter = kCounterFreqLaneChange; // reset up
  }
  counter_lane_change_ = counter;
  
  // Debug logging
  if (kDBGBehavior != 0) {
    std::cout << "Freq lane change counter: "
              << counter_lane_change_ << std::endl << std::endl;
    std::cout << "Final Target Behavior: " << std::endl;
    std::cout << " intent: " << tgt_behavior_.intent
              << ", target lane: " << tgt_behavior_.tgt_lane
              << ", tgt_speed (mph): "  << mps2mph(tgt_behavior_.tgt_speed)
              << std::endl << std::endl;
  }
}

//// DetectedVehicle sub-class ////
 
// Constructor/Destructor
DetectedVehicle::DetectedVehicle() : Vehicle() { }
DetectedVehicle::~DetectedVehicle() { }

/**
 * Calculate relative s from ego car
 */
void DetectedVehicle::UpdateRelDist(const EgoVehicle &ego_car) {
  
  double s_rel = GetState().s - ego_car.GetState().s;
  // Normalize to within sensor range in case s wraps around the track
  while (s_rel > kSensorRange) { s_rel -= kMaxS; }
  while (s_rel < -kSensorRange) { s_rel += kMaxS; }
  s_rel_ = s_rel;
}

double DetectedVehicle::GetRelS() const { return s_rel_; }

std::map<VehIntents, VehTrajectory> DetectedVehicle::GetPredTrajs() const {
  return pred_trajs_;
}

void DetectedVehicle::ClearPredTrajs() { pred_trajs_.clear(); }

void DetectedVehicle::SetPredTrajs( std::map<VehIntents,
                                             VehTrajectory> pred_trajs) {
  pred_trajs_ = pred_trajs;
}

//// General vehicle functions ////
 
/**
 * For the specified car ID# (check_id), check in the specified
 * lane (check_lane) to find the car ahead/behind (check_side = kFront or kBack)
 * and return a tuple of {car ID, relative s distance}.
 */
std::tuple<int, double> FindCarInLane(const VehSides check_side,
                       const int check_lane, const int check_id,
                       const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  // Set default return values in case of no car ahead
  int car_id_found = check_id; // default to same as check car
  double s_rel_found = kSensorRange; // default to full sensor range
  if (check_side == kBack) {
    s_rel_found *= -1.; // flip sign if looking behind
  }

  // Copy car ID's to check in the lane to a vector
  std::vector<int> cars_in_check_lane;
  if (car_ids_by_lane.count(check_lane) > 0) {
    for (int i = 0; i < car_ids_by_lane.at(check_lane).size(); ++i) {
      cars_in_check_lane.push_back(car_ids_by_lane.at(check_lane)[i]);
    }
  }
  
  // Add ego car ID to the check lane for other cars to find
  if ((check_id != ego_car.GetID()) && (check_lane == ego_car.GetLane())) {
    cars_in_check_lane.push_back(ego_car.GetID());
  }
  
  // Look for car in the lane
  for (int i = 0; i < cars_in_check_lane.size(); ++i) {
    const int cur_car_id = cars_in_check_lane[i];
    double cur_s_rel;
    double ref_s_rel;
    if (cur_car_id != check_id) { // only check if it's not yourself
      // Set relative s value for the current car in the lane
      if (cur_car_id != ego_car.GetID()) {
        cur_s_rel = detected_cars.at(cur_car_id).GetRelS(); // other car
      }
      else {
        cur_s_rel = 0.; // ego car
      }
      
      // Set reference relative s value of car doing the check
      if (check_id != ego_car.GetID()) {
        ref_s_rel = detected_cars.at(check_id).GetRelS(); // car doing the check
      }
      else {
        ref_s_rel = 0.; // ego car
      }
      
      // Found closer car ahead if dist is positive and less than prev found,
      // or behind if dist is negative and greater than prev found
      const double cur_dist = cur_s_rel - ref_s_rel;
      if (((check_side == kFront)
             && (cur_dist > 0.) && (cur_dist < s_rel_found))
          || ((check_side == kBack)
                && (cur_dist < 0.) && (cur_dist > s_rel_found))) {
        s_rel_found = cur_dist;
        car_id_found = cur_car_id;
      }
    }
  }
  
  return std::make_tuple(car_id_found, s_rel_found);
}

/**
 * Check the size of the gap on the left/right side of the ego car
 * (check_side = kLeft or kRight) to the car ahead/behind and return the size
 * of the gap.  Set the gap to 0 if either the car ahead or behind are closer
 * than the minimum distance for lane change kLaneChangeMinGap or there is no
 * lane on that side of the ego car.
 */
double EgoCheckSideGap(const VehSides check_side,
                       const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  double gap_on_side;
  
  // Set default values in case of no car ahead or behind
  int car_id_ahead = ego_car.GetID();
  int car_id_behind = ego_car.GetID();
  double rel_s_ahead = kSensorRange;
  double rel_s_behind = -kSensorRange;
  
  if ((check_side == kRight) && (ego_car.GetLane() == kNumLanes)) {
    gap_on_side = 0.; // No lane to the right
  }
  else if ((check_side == kLeft) && (ego_car.GetLane() == 1)) {
    gap_on_side = 0.; // No lane to the left
  }
  else {
    const int check_lane = ego_car.GetLane() + int(check_side);
    if (car_ids_by_lane.count(check_lane) > 0) {
      // Get car ahead and behind in the check lane
      auto car_ahead = FindCarInLane(kFront, check_lane, ego_car.GetID(),
                                     ego_car, detected_cars, car_ids_by_lane);
      car_id_ahead = std::get<0>(car_ahead);
      rel_s_ahead = std::get<1>(car_ahead);
      
      auto car_behind = FindCarInLane(kBack, check_lane, ego_car.GetID(),
                                      ego_car, detected_cars, car_ids_by_lane);
      car_id_behind = std::get<0>(car_behind);
      rel_s_behind = std::get<1>(car_behind);
      
      if ((abs(rel_s_ahead) < kLaneChangeMinGap)
          || (abs(rel_s_behind) < kLaneChangeMinGap)) {
        gap_on_side = 0.0; // a car is too close on the side
      }
      else {
        gap_on_side = rel_s_ahead - rel_s_behind;
      }
    }
    else {
      // No other cars in side lane
      gap_on_side = 2*kSensorRange;
    }
  }
  
  // Debug logging
  if (kDBGVehicle != 0) {
    std::cout << "GapOnSide " << check_side << ": " << gap_on_side << "m  ("
              << rel_s_ahead << "m to car " << car_id_ahead << " ahead, "
              << rel_s_behind << "m to car " << car_id_behind << " behind)"
              << std::endl;
  }
  
  return gap_on_side;
}
