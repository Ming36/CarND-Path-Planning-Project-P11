//
//  sensor_fusion.hpp
//  Path_Planning
//
//  Created by Student on 1/30/18.
//

#ifndef sensor_fusion_hpp
#define sensor_fusion_hpp

#include <stdio.h>
#include "vehicle.hpp"

int GetCurrentTrajIndex(const VehTrajectory &prev_ego_traj,
                        int prev_path_size);

VehState ProcessEgoState(double car_x, double car_y, int idx_current_pt,
                         const VehTrajectory &prev_ego_traj,
                         const std::vector<double> &map_interp_s,
                         const std::vector<double> &map_interp_x,
                         const std::vector<double> &map_interp_y);

void ProcessDetectedCars(const EgoVehicle &ego_car,
                         const std::vector<std::vector<double>> &sensor_fusion,
                         const std::vector<double> &map_interp_s,
                         const std::vector<double> &map_interp_x,
                         const std::vector<double> &map_interp_y,
                         const std::vector<double> &map_interp_dx,
                         const std::vector<double> &map_interp_dy,
                         std::map<int, DetectedVehicle> *detected_cars);

std::map<int, std::vector<int>> SortDetectedCarsByLane(
                          const std::map<int, DetectedVehicle> &detected_cars);

#endif /* sensor_fusion_hpp */
