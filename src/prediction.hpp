//
//  prediction.hpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#ifndef prediction_hpp
#define prediction_hpp

#include <stdio.h>

#include <vector>
#include <map>
#include "path_common.hpp"
#include "vehicle.hpp"

void PredictBehavior(std::map<int, DetectedVehicle> &detected_cars,
                     const std::map<int, std::vector<int>> &car_ids_by_lane);

void PredictTrajectory(std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane,
                       double predict_time);

#endif /* prediction_hpp */
