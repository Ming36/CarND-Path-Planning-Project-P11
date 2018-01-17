//
//  path_common.hpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#ifndef path_common_hpp
#define path_common_hpp

#include <stdio.h>

#include <math.h>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

/**
 * Constant parameters
 */
constexpr double kSensorRange = 100.; // m
constexpr double kPredictTime = 2.0; // sec
constexpr double kLaneWidth = 4.0; // m
constexpr double kLatVelLaneChange = 5.0; // mph
constexpr double kTargetSpeed = 44.7387; // mph (20 m/s)
constexpr double kSimCycleTime = 0.02; // sec
constexpr double kMaxA = 5.0; // max a for constant accel approximation to keep peak <10m/s2

/**
 * Basic parameter helpers
 */
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double mps2mph(double x) { return x * 2.23694; }
inline double mph2mps(double x) { return x / 2.23694; }
inline double mps2pointdist(double x) { return x * 0.020; }

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

std::vector<double> JMT(std::vector< double> start, std::vector <double> end,
                        double t_end);

double EvalPoly(double x, std::vector<double> coeffs);

#endif /* path_helper_hpp */
