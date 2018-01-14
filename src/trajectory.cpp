//
//  trajectory.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "trajectory.hpp"

VehTrajectory GetTrajectory(double car_s, double car_d, const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_s) {

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
  std::vector<double> next_s_vals;
  std::vector<double> next_d_vals;
  std::vector<double> next_t_vals;
  
  // The car will visit each (x,y) point sequentially every .02 seconds
  double target_speed = mph2mps(49.);
  double dist_inc = mps2pointdist(target_speed);
  double new_t = 0;
  for(int i = 0; i < 50; i++)
  {
    double new_car_s = car_s + (i+1) * dist_inc;
    double new_car_d = car_d;
    std::vector<double> new_car_xy = GetXY(new_car_s, new_car_d,
                                           maps_s, maps_x, maps_y);
    next_x_vals.push_back(new_car_xy[0]);
    next_y_vals.push_back(new_car_xy[1]);
    next_s_vals.push_back(new_car_s);
    next_d_vals.push_back(new_car_d);
    next_t_vals.push_back(new_t*i*0.02);
  }
  
  return {next_x_vals, next_y_vals, next_s_vals, next_d_vals, next_t_vals};
}
