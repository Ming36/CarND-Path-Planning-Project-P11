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
#include <vector>
#include <map>
#include "vehicle.hpp"

void VehBehaviorFSM(EgoVehicle &ego_car,
                    const std::map<int, DetectedVehicle> &detected_cars,
                    const std::map<int, std::vector<int>> &car_ids_by_lane);

#endif /* behavior_hpp */
