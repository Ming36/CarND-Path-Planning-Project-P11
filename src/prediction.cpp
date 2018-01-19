//
//  prediction.cpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#include "prediction.hpp"

void PredictBehavior(std::map<int, DetectedVehicle> &detected_cars,
                     const std::map<int, std::vector<int>> &car_ids_by_lane) {
  
  for (std::map<int, std::vector<int>>::const_iterator it = car_ids_by_lane.begin();
       it != car_ids_by_lane.end(); ++it) {
    
    // Loop through each lane's vector of car id's
    for (int i=0; i < it->second.size(); ++i) {
      int cur_car_id = it->second[i];
      DetectedVehicle* cur_car = &detected_cars[cur_car_id];
      
      // Predict behavior for this detected car
      
      if ((cur_car->intent_ == kKeepLane) && (cur_car->state_.d_dot > mph2mps(kLatVelLaneChange))) {
        cur_car->intent_ = kLaneChangeRight;
        
        // DEBUG Print detected lane change
        std::cout << "** Lane change right detected by car #" << cur_car->veh_id_ << std::endl;
      }
      else if ((cur_car->intent_ == kKeepLane) && (cur_car->state_.d_dot < mph2mps(-kLatVelLaneChange))) {
        cur_car->intent_ = kLaneChangeLeft;
        
        // DEBUG Print detected lane change
        std::cout << "** Lane change left detected by car #" << cur_car->veh_id_ << std::endl;
      }
      else if (((cur_car->intent_ == kLaneChangeRight)
                 && (cur_car->state_.d_dot < mph2mps(kLatVelLaneChange)))
               || ((cur_car->intent_ == kLaneChangeLeft)
                   && (cur_car->state_.d_dot > mph2mps(-kLatVelLaneChange)))) {
        cur_car->intent_ = kKeepLane;
        
        // DEBUG Print detected lane change ended
        std::cout << "** End of lane change by car #" << cur_car->veh_id_ << std::endl;
      }
      else if (cur_car->intent_ == kUnknown) {
        cur_car->intent_ = kKeepLane;
      }
    }
  }
}

void PredictTrajectory(std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane,
                       double predict_time) {
  
  for (std::map<int, std::vector<int>>::const_iterator it = car_ids_by_lane.begin();
       it != car_ids_by_lane.end(); ++it) {
    
    // Loop through each lane's vector of car id's
    for (int i=0; i < it->second.size(); ++i) {
      int cur_car_id = it->second[i];
      DetectedVehicle* cur_car = &detected_cars[cur_car_id];
      
      // TODO Generate trajectories for each detected car
      
    }
  }
}
