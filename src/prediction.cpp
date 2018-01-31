//
//  prediction.cpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#include "prediction.hpp"

void PredictBehavior(std::map<int, DetectedVehicle> &detected_cars,
                     const EgoVehicle &ego_car,
                     const std::map<int, std::vector<int>> &car_ids_by_lane,
                     const std::vector<double> &map_interp_s,
                     const std::vector<double> &map_interp_x,
                     const std::vector<double> &map_interp_y) {
  
  /*
   1. For each detected car, set possible intents.
   2. For each intent, make a predicted trajectory and set a probability
  */
  
  for (auto it = car_ids_by_lane.begin();
            it != car_ids_by_lane.end(); ++it) {
    
    // Loop through each lane's vector of car id's
    for (int i = 0; i < it->second.size(); ++i) {
      int cur_car_id = it->second.at(i);
      DetectedVehicle* cur_car = &detected_cars.at(cur_car_id);
      cur_car->pred_trajs_.clear();
      
      // Predict behavior for this detected car
      double t_tgt = kPredictTime;
      double v_tgt;
      double d_tgt;
      auto car_ahead = GetCarAheadInLane(cur_car->lane_, cur_car->veh_id_,
                                         ego_car, detected_cars,
                                         car_ids_by_lane);
      int car_id_ahead = std::get<0>(car_ahead);
      double s_rel_ahead = std::get<1>(car_ahead);
      
      // KeepLane:
      //   Add for all cars with base probability 1.0
      if (s_rel_ahead < kTgtFollowDist) {
        double v_car_ahead;
        if (car_id_ahead != ego_car.veh_id_) {
          v_car_ahead = detected_cars.at(car_id_ahead).state_.s_dot;
        }
        else {
          v_car_ahead = ego_car.state_.s_dot;
        }
        v_tgt = std::min(cur_car->state_.s_dot, v_car_ahead);
      }
      else {
        v_tgt = cur_car->state_.s_dot;
      }
      d_tgt = cur_car->state_.d;
      VehTrajectory traj_KL = GetTrajectory(cur_car->state_, t_tgt, v_tgt,
                                            d_tgt, kMaxA, map_interp_s,
                                            map_interp_x, map_interp_y);
      traj_KL.probability = 1.0;
      cur_car->pred_trajs_[kKeepLane] = traj_KL;
      
      // LaneChangeLeft:
      //   Add if lane is open to left and set high prob if car ahead is close
      if (cur_car->lane_ > 1) {
        v_tgt = cur_car->state_.s_dot;
        d_tgt = tgt_lane2tgt_d(cur_car->lane_ - 1);
        VehTrajectory traj_LCL = GetTrajectory(cur_car->state_, t_tgt, v_tgt,
                                               d_tgt, kMaxA, map_interp_s,
                                               map_interp_x, map_interp_y);
        
        double prob_LCL = 0.1;
        if (s_rel_ahead < kTgtFollowDist) { prob_LCL = 0.3; }
        if (cur_car->state_.d_dot < -kLatVelLaneChange) { prob_LCL = 0.8; }
        traj_LCL.probability = prob_LCL;
        
        cur_car->pred_trajs_[kLaneChangeLeft] = traj_LCL;
        cur_car->pred_trajs_.at(kKeepLane).probability -= prob_LCL;
      }
      
      // LaneChangeRight:
      //   Add if lane is open to right and set high prob if car ahead is close
      if (cur_car->lane_ < kNumLanes) {
        v_tgt = cur_car->state_.s_dot;
        d_tgt = tgt_lane2tgt_d(cur_car->lane_ + 1);
        VehTrajectory traj_LCR = GetTrajectory(cur_car->state_, t_tgt, v_tgt,
                                               d_tgt, kMaxA, map_interp_s,
                                               map_interp_x, map_interp_y);
        
        double prob_LCR = 0.1;
        if (s_rel_ahead < kTgtFollowDist) { prob_LCR = 0.3; }
        if (cur_car->state_.d_dot > kLatVelLaneChange) { prob_LCR = 0.8; }
        traj_LCR.probability = prob_LCR;
        
        cur_car->pred_trajs_[kLaneChangeRight] = traj_LCR;
        cur_car->pred_trajs_.at(kKeepLane).probability -= prob_LCR;
      }
    }
  }
  
  // Debug logging
  if (kDBGPrediction != 0) {
    std::cout << "Predicted intents:" << std::endl;
    for (auto it = detected_cars.begin();
         it != detected_cars.end(); ++it) {
      std::cout << "car #" << it->first << " - ";
      auto predictions = it->second.pred_trajs_;
      for (auto it2 = predictions.begin();
           it2 != predictions.end(); ++it2) {
        std::cout << it2->first << " = "
        << it2->second.probability << ", ";
      }
      std::cout << std::endl;
    }
  }
}
