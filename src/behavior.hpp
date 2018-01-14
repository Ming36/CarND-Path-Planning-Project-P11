//
//  behavior.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef behavior_hpp
#define behavior_hpp

#include <stdio.h>

#include <string>
#include "vehicle.hpp"

VehBehavior VehBehaviorFSM(EgoVehicle ego_car,
                           std::vector<DetectedVehicle> veh_preds_lanetoleft,
                           std::vector<DetectedVehicle> veh_preds_curlane,
                           std::vector<DetectedVehicle> veh_preds_lanetoright);

#endif /* behavior_hpp */
