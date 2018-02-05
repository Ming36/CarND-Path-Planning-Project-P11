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
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

/**
 * Constant parameters
 */

// Debug logging
constexpr int kDBGMain = 0; // 1=Basic, 2=Telemetry, 3=Every Sim Loop
constexpr int kDBGVehicle = 0;
constexpr int kDBGSensorFusion = 0;
constexpr int kDBGPrediction = 0;
constexpr int kDBGBehavior = 0;
constexpr int kDBGTrajectory = 0;

// Simulation and Track
constexpr double kSimCycleTime = 0.02; // (sec)
constexpr double kMaxS = 6945.554; // (m) max S before track wraps back to 0
constexpr double kMapInterpInc = 1.; // (m) interpolated increment in Frenet S
constexpr double kLaneWidth = 3.9; // (m) width per lane
constexpr int kNumLanes = 3; // (#) lanes in the road

// Main Path Planner
constexpr int kPathCycleTimeMS = 200; // (ms) path planner cycle time
constexpr double kSensorRange = 100.; // (m)

// Prediction
constexpr double kLatVelLaneChange = (5.) / 2.23694; // (mph)->m/s to judge LC
constexpr double kPredictTime = 1.5; // (sec) time to predict car paths
//constexpr double kPredictTime = 2.; // (sec) time to predict car paths

// Behavior
constexpr double kCostDistAhead = 5.;
constexpr double kCostSpeedAhead = 7.;
constexpr double kCostSpeedBehind = 7.;
constexpr double kCostChangeLanes = 0.8;
constexpr double kCostFreqLaneChange = 1.;
constexpr int kCounterFreqLaneChange = 15; // (#) path cycles
constexpr double kRelSpeedBehind = (10.) / 2.23694; // mph -> m/s
constexpr double kLaneChangeMinGap = 10.; // (m)
//constexpr double kLaneChangeMinGap = 9.; // (m)
constexpr double kTargetSpeed = (49.) / 2.23694; // mph -> m/s, base target
constexpr double kTgtMinSpeed = (0.) / 2.23694; // mph -> m/s, min target speed
constexpr double kTgtStartFollowDist = 40.; // (m)
constexpr double kTgtFollowDist = 14.; // (m)
//constexpr double kTgtFollowDist = 15.; // (m)
constexpr double kTgtMinFollowDist = 11.; // (m)
constexpr double kMinFollowTgtSpeedDec = (10.) / 2.23694; // (mph)->m/s to slow
constexpr double kPlanLCTgtSpeedDec = (20.) / 2.23694; // (mph)->m/s, to find gap
constexpr double kPlanLCCloseDist = 11.; // (m)

// Trajectory
constexpr double kPathBufferTime = 0.5; // (sec) duration of prev path buffer
constexpr double kNewPathTime = 2.5; // (sec) duration of new planned path
constexpr double kMinTrajPntDist = (3.) / 2.23694 * kSimCycleTime; // (mph)->m
constexpr double kMaxA = 8.; // (m/s^2) target max accel to keep peak < 10m/s^2
constexpr double kSpdAdjOffset = (2.) / 2.23694; // (mph)->m/s
constexpr double kAccAdjOffset = 1.; // (m/s^2)
constexpr int kAccelAveSamples = 10; // (#) smoothing ave accel
constexpr double kCollisionSThresh = 8.; // (m) gap S to judge collision risk
constexpr double kCollisionDThresh = 3.; // (m) gap D to judge collision risk
constexpr int kEvalRiskStep = 10; // (#) time step interval to check risk
constexpr int kTrajGenNum = 5; // (#) possible traj's to sample from
constexpr double kRandSpdMean = (5.) / 2.23694; // (mph)->m/s speed adj mean
constexpr double kRandSpdDev = (2.) / 2.23694; // (mph)->m/s speed adj std dev
constexpr double kRandTimeMean = 0.; // (sec) path time adj mean
constexpr double kRandTimeDev = 0.6; // (sec) path time adj std dev
constexpr double kMinTrajTime = 1.5; // sec
constexpr double kTrajCostRisk = 10.; // cost for traj collision risk
constexpr double kTrajCostDeviation = 1.; // cost for traj deviation from base
constexpr double kTrajCostThresh = 20; // cost thresh to judge traj risk
//constexpr double kTrajCostThresh = 10; // cost thresh to judge traj risk
constexpr double kBackupTgtSpeedDec = (10.) / 2.23694; // (mph)->m/s to slow

/**
 * Basic parameter helpers
 */
constexpr double pi() { return M_PI; }
inline double mps2mph(double x) { return x * 2.23694; }
inline double sq(double x) { return x * x; }

inline double tgt_lane2tgt_d(int tgt_lane) {
  return (kLaneWidth / 2) + (tgt_lane - 1) * kLaneWidth;
}

double Distance(double x1, double y1, double x2, double y2);

std::vector<std::vector<double>> InterpolateMap(std::vector<double> &map_s,
                                                std::vector<double> &map_x,
                                                std::vector<double> &map_y,
                                                std::vector<double> &map_dx,
                                                std::vector<double> &map_dy,
                                                double s_dist_inc);

int ClosestWaypoint(double x, double y, const std::vector<double> &map_x,
                    const std::vector<double> &map_y);

std::vector<double> GetHiResXY(double s, double d,
                               const std::vector<double> &map_s,
                               const std::vector<double> &map_x,
                               const std::vector<double> &map_y);

std::vector<double> GetHiResFrenet(double x, double y,
                                   const std::vector<double> &map_s,
                                   const std::vector<double> &map_x,
                                   const std::vector<double> &map_y);

std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &map_dx,
                                      const std::vector<double> &map_dy);

std::vector<double> JMT(std::vector< double> start, std::vector<double> end,
                        double t_end);

double EvalPoly(double x, std::vector<double> coeffs);

std::vector<double> DiffPoly(std::vector<double> coeffs);

double LogCost(double x, double x_saturate);

#endif /* path_helper_hpp */
