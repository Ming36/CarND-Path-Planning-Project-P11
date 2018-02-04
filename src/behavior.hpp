//
//  behavior.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef behavior_hpp
#define behavior_hpp

#include <stdio.h>
#include "vehicle.hpp"

int LaneCostFcn(const EgoVehicle &ego_car,
                const std::map<int, DetectedVehicle> &detected_cars,
                const std::map<int, std::vector<int>> &car_ids_by_lane);

VehIntents BehaviorFSM(const EgoVehicle &ego_car,
                       const std::map<int, DetectedVehicle> &detected_cars,
                       const std::map<int, std::vector<int>> &car_ids_by_lane);

double SetTargetSpeed(const EgoVehicle &ego_car,
                      const std::map<int, DetectedVehicle> &detected_cars,
                      const std::map<int, std::vector<int>> &car_ids_by_lane);

double TargetSpeedKL(double base_tgt_spd, const EgoVehicle &ego_car,
                     const std::map<int, DetectedVehicle> &detected_cars,
                     const std::map<int, std::vector<int>> &car_ids_by_lane);

double TargetSpeedPLC(VehSides sidePLC, double base_tgt_spd,
                      const EgoVehicle &ego_car,
                      const std::map<int, DetectedVehicle> &detected_cars,
                      const std::map<int, std::vector<int>> &car_ids_by_lane);

double TargetSpeedLC(VehSides sideLC, double base_tgt_spd,
                      const EgoVehicle &ego_car,
                      const std::map<int, DetectedVehicle> &detected_cars,
                      const std::map<int, std::vector<int>> &car_ids_by_lane);

#endif /* behavior_hpp */
