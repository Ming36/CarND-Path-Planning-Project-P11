//
//  prediction.hpp
//  Path_Planning
//
//  Created by Student on 1/9/18.
//

#ifndef prediction_hpp
#define prediction_hpp

#include <stdio.h>
#include "vehicle.hpp"
#include "trajectory.hpp"

void PredictBehavior(const EgoVehicle &ego_car,
                     const std::map<int, std::vector<int>> &car_ids_by_lane,
                     const std::vector<double> &map_interp_s,
                     const std::vector<double> &map_interp_x,
                     const std::vector<double> &map_interp_y,
                     std::map<int, DetectedVehicle> *detected_cars);

#endif /* prediction_hpp */
