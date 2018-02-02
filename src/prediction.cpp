//
//  prediction.cpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#include "prediction.hpp"

/**
 *
 */
void PredictBehavior(const EgoVehicle &ego_car,
                     const std::map<int, std::vector<int>> &car_ids_by_lane,
                     const std::vector<double> &map_interp_s,
                     const std::vector<double> &map_interp_x,
                     const std::vector<double> &map_interp_y,
                     std::map<int, DetectedVehicle> *detected_cars) {
  
  // Loop through each lane of veh ID's
  for (auto it = car_ids_by_lane.begin(); it != car_ids_by_lane.end(); ++it) {
    auto car_ids_in_lane = it->second;
    // Loop through each lane's vector of car id's
    for (int i = 0; i < car_ids_in_lane.size(); ++i) {
      int cur_car_id = car_ids_in_lane[i];
      DetectedVehicle* cur_car_ptr = &detected_cars->at(cur_car_id);
      cur_car_ptr->ClearPredTrajs();
      
      // Predict behavior for this detected car
      std::map<VehIntents, VehTrajectory> new_pred_trajs;
      double t_tgt = kPredictTime; // use same prediction time for all intents
      double v_tgt;
      double d_tgt;
      const auto car_ahead = FindCarInLane(kFront, cur_car_ptr->GetLane(),
                                           cur_car_ptr->GetID(), ego_car,
                                           (*detected_cars), car_ids_by_lane);
      const int car_id_ahead = std::get<0>(car_ahead);
      const double s_rel_ahead = std::get<1>(car_ahead);
      
      //// KeepLane intent predicted traj ////
      // Add for all cars with base probability 1.0
      
      // Set speed target as current speed, but limit by car ahead if within
      // expected following distance
      if (s_rel_ahead < kTgtFollowDist) {
        double v_car_ahead;
        if (car_id_ahead != ego_car.GetID()) {
          v_car_ahead = detected_cars->at(car_id_ahead).GetState().s_dot;
        }
        else {
          v_car_ahead = ego_car.GetState().s_dot;
        }
        v_tgt = std::min(cur_car_ptr->GetState().s_dot, v_car_ahead);
      }
      else {
        v_tgt = cur_car_ptr->GetState().s_dot;
      }
      
      // Set target d to keep current value
      d_tgt = cur_car_ptr->GetState().d;
      
      // Generate predicted traj for KeepLane intent
      auto traj_KL = GetTrajectory(cur_car_ptr->GetState(), t_tgt, v_tgt, d_tgt,
                                   kMaxA, map_interp_s, map_interp_x,
                                   map_interp_y);
      traj_KL.probability = 1.0;
      new_pred_trajs[kKeepLane] = traj_KL;
      
      //// LaneChangeLeft intent predicted traj ////
      // Add if lane is open to left and set high prob if car ahead is close

      if (cur_car_ptr->GetLane() > 1) {
        v_tgt = cur_car_ptr->GetState().s_dot; // keep current speed
        d_tgt = tgt_lane2tgt_d(cur_car_ptr->GetLane() - 1); // target left lane
        
        // Generate predicted traj for LaneChangeLeft intent
        auto traj_LCL = GetTrajectory(cur_car_ptr->GetState(), t_tgt, v_tgt, d_tgt,
                                      kMaxA, map_interp_s, map_interp_x,
                                      map_interp_y);

        // LCL probability 0.1 default, 0.3 if close to car ahead, 0.8 if
        // already moving to the left fast enough
        double prob_LCL = 0.1;
        if (s_rel_ahead < kTgtFollowDist) { prob_LCL = 0.3; }
        if (cur_car_ptr->GetState().d_dot < -kLatVelLaneChange) { prob_LCL = 0.8; }
        traj_LCL.probability = prob_LCL;
        
        new_pred_trajs[kLaneChangeLeft] = traj_LCL;
        new_pred_trajs.at(kKeepLane).probability -= prob_LCL;
      }
      
      //// LaneChangeRight intent predicted traj ////
      // Add if lane is open to right and set high prob if car ahead is close
      
      if (cur_car_ptr->GetLane() < kNumLanes) {
        v_tgt = cur_car_ptr->GetState().s_dot; // keep current speed
        d_tgt = tgt_lane2tgt_d(cur_car_ptr->GetLane() + 1); // target right lane
        
        // Generate predicted traj for LaneChangeRight intent
        auto traj_LCR = GetTrajectory(cur_car_ptr->GetState(), t_tgt, v_tgt, d_tgt,
                                      kMaxA, map_interp_s, map_interp_x,
                                      map_interp_y);
        
        // LCR probability 0.1 default, 0.3 if close to car ahead, 0.8 if
        // already moving to the right fast enough
        double prob_LCR = 0.1;
        if (s_rel_ahead < kTgtFollowDist) { prob_LCR = 0.3; }
        if (cur_car_ptr->GetState().d_dot > kLatVelLaneChange) { prob_LCR = 0.8; }
        traj_LCR.probability = prob_LCR;
        
        new_pred_trajs[kLaneChangeRight] = traj_LCR;
        new_pred_trajs.at(kKeepLane).probability -= prob_LCR;
      }
      
      // Set predicted trajectories to current car
      cur_car_ptr->SetPredTrajs(new_pred_trajs);
    }
  }
  
  // Debug logging
  if (kDBGPrediction != 0) {
    std::cout << "Predicted intents:" << std::endl;
    for (auto it = detected_cars->begin(); it != detected_cars->end(); ++it) {
      std::cout << "car #" << it->first << " - ";
      auto predictions = it->second.GetPredTrajs();
      for (auto it2 = predictions.begin();
           it2 != predictions.end(); ++it2) {
        std::cout << it2->first << " = "
        << it2->second.probability << ", ";
      }
      std::cout << std::endl;
    }
  }
}
