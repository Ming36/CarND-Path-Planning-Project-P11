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
#include "vehicle.hpp"

void PredictBehavior(Vehicle &veh, std::vector<Vehicle> &cars_detected);

void PredictTrajectory();

#endif /* prediction_hpp */
