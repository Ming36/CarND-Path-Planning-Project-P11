//
//  path_helper.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef path_helper_hpp
#define path_helper_hpp

#include <stdio.h>

#include <math.h>
#include <vector>

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double mps2mph(double x);
double mph2mps(double x);
double mps2pointdist(double x);

double Distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

std::vector<double> GetFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y);

std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &maps_dx,
                                      const std::vector<double> &maps_dy);

std::vector<double> GetXY(double s, double d,
                          const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);

#endif /* path_helper_hpp */
