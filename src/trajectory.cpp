//
//  trajectory.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "trajectory.hpp"

VehTrajectory GetTrajectory(EgoVehicle &ego_car, double t_tgt,
                            double v_tgt, double d_tgt,
                            const std::vector<double> &map_hires_s,
                            const std::vector<double> &map_hires_x,
                            const std::vector<double> &map_hires_y) {
 
  // Generate S trajectory
  
  double s_est;
  double s_dot_est;
  double s_dot_dot_est;
  
  // Set estimated s, s_dot, s_dot_dot to keep a reasonable JMT with basic kinematics
  const double t_maxa = (v_tgt - ego_car.s_dot_) / kMaxA;
  
  if (t_maxa > t_tgt) {
    // Cut off target v and a to limit t
    s_dot_est = ego_car.s_dot_ + kMaxA * t_tgt;
    s_dot_dot_est = kMaxA;
  }
  else {
    // Can achieve target speed in time
    s_dot_est = v_tgt;
    s_dot_dot_est = (s_dot_est - ego_car.s_dot_) / t_tgt;
  }
  
  s_est = ego_car.s_ + ego_car.s_dot_*t_tgt + 0.5*s_dot_dot_est*t_tgt*t_tgt;
  
  std::vector<double> start_state_s = {ego_car.s_, ego_car.s_dot_, ego_car.s_dot_dot_};
  std::vector<double> end_state_s = {s_est, s_dot_est, s_dot_dot_est};

  ego_car.coeffs_JMT_s_ = JMT(start_state_s, end_state_s, t_tgt);
  ego_car.coeffs_JMT_s_dot_ = DiffPoly(ego_car.coeffs_JMT_s_);
  ego_car.coeffs_JMT_s_dot_dot_ = DiffPoly(ego_car.coeffs_JMT_s_dot_);
  
  // Generate D trajectory
  
  double d_est = d_tgt;
  double d_dot_est = 0;
  double d_dot_dot_est = 0;
  
  std::vector<double> start_state_d = {ego_car.d_, ego_car.d_dot_, ego_car.d_dot_dot_};
  std::vector<double> end_state_d = {d_est, d_dot_est, d_dot_dot_est};
  
  ego_car.coeffs_JMT_d_ = JMT(start_state_d, end_state_d, t_tgt);
  ego_car.coeffs_JMT_d_dot_ = DiffPoly(ego_car.coeffs_JMT_d_);
  ego_car.coeffs_JMT_d_dot_dot_ = DiffPoly(ego_car.coeffs_JMT_d_dot_);
  
  // Look up (s,d) vals for each sim cycle time step and convert to raw (x,y)
  VehTrajectory result_traj;
  /*
  std::vector<double> raw_x_vals;
  std::vector<double> raw_y_vals;
  std::vector<double> raw_s_vals;
  std::vector<double> raw_d_vals;
  std::vector<double> raw_t_vals;
  */
  const int num_pts = t_tgt / kSimCycleTime;

  for (int i = 0; i < num_pts; ++i) { // TODO fix initial index
    double t = i * kSimCycleTime; // idx 0 is 1st point ahead of car
    
    double raw_s = EvalPoly(t, ego_car.coeffs_JMT_s_);
    double raw_d = EvalPoly(t, ego_car.coeffs_JMT_d_);
    
    std::vector<double> raw_xy = GetHiresXY(raw_s, raw_d, map_hires_s,
                                            map_hires_x, map_hires_y);
    
    /*
    raw_x_vals.push_back(raw_xy[0]);
    raw_y_vals.push_back(raw_xy[1]);
    raw_s_vals.push_back(raw_s);
    raw_d_vals.push_back(raw_d);
    raw_t_vals.push_back(t);
    */
    
    result_traj.x.push_back(raw_xy[0]);
    result_traj.y.push_back(raw_xy[1]);
    result_traj.s.push_back(raw_s);
    result_traj.d.push_back(raw_d);
    result_traj.t.push_back(t);
  }
  
  
  /*
  std::vector<double> car_sd = GetHiresFrenet(ego_car.x_, ego_car.y_,
                                              map_hires_s,
                                              map_hires_x,
                                              map_hires_y);
  
  std::vector<double> car_xy = GetHiresXY(car_sd[0], car_sd[1], map_hires_s,
                                          map_hires_x, map_hires_y);
  
  std::cout << "x: " << ego_car.x_ << ", y: " << ego_car.y_
            << ", xy->s: " << ego_car.s_ << ", xy->d: " << ego_car.d_
            << ", sd->x: " << raw_x_vals[0] << ", sd->y: " << raw_y_vals[1];
  
  std::cout << "" << std::endl;
  */
  
  /*
  VehTrajectory result_traj;
  result_traj.x = raw_x_vals;
  result_traj.y = raw_y_vals;
  result_traj.s = raw_s_vals;
  result_traj.d = raw_d_vals;
  result_traj.t = raw_t_vals;
  */
  
  return result_traj;
}

void GetFinalTrajectory(EgoVehicle &ego_car,
                        const std::vector<double> &map_hires_s,
                        const std::vector<double> &map_hires_x,
                        const std::vector<double> &map_hires_y) {
  
  // Set target time and speed
  const double t_tgt = ego_car.behavior_.target_time;
  const double v_tgt = mph2mps(kTargetSpeedMPH); // mph -> m/s
  
  // Set target D based on target lane
  double d_tgt = 1.9 + (ego_car.behavior_.target_lane-1) * 4.0;

  // Calculate initial trajectory
  VehTrajectory traj = GetTrajectory(ego_car, t_tgt, v_tgt, d_tgt,
                                     map_hires_s, map_hires_x, map_hires_y);

  // Check for (x,y) overspeed and recalculate trajectory to compensate
  double v_peak = 0;
  double xy_speed = 0;
  for (int i=0; i < traj.x.size()-1; ++i) {
    xy_speed = (Distance(traj.x[i], traj.y[i], traj.x[i+1], traj.y[i+1])
                       / kSimCycleTime);
    if (xy_speed > v_peak) { v_peak = xy_speed; }
  }
  
  //std::cout << "Before: " << mps2mph(v_peak) << std::endl;
  
  if (v_peak > mph2mps(kTargetSpeedMPH)) {
    double speed_adj_ratio = mph2mps(kTargetSpeedMPH) / v_peak;
    //std::cout << "Adj: " << speed_adj_ratio << std::endl;
    
    traj = GetTrajectory(ego_car, t_tgt,
                         (v_tgt * speed_adj_ratio - mph2mps(kSpdAdjOffsetMPH)),
                         d_tgt, map_hires_s, map_hires_x, map_hires_y);
  }
  
  /*
  v_peak = 0;
  for (int i=0; i < traj.x.size()-1; ++i) {
    xy_speed = (Distance(traj.x[i], traj.y[i], traj.x[i+1], traj.y[i+1])
                / kSimCycleTime);
    if (xy_speed > v_peak) { v_peak = xy_speed; }
  }
  std::cout << "After: " << mps2mph(v_peak) << std::endl << std::endl;
  */
  
  // Store final trajectory
  ego_car.trajectory_.x.clear();
  ego_car.trajectory_.y.clear();
  ego_car.trajectory_.s.clear();
  ego_car.trajectory_.d.clear();
  ego_car.trajectory_.t.clear();
  
  ego_car.trajectory_.x = traj.x;
  ego_car.trajectory_.y = traj.y;
  ego_car.trajectory_.s = traj.s;
  ego_car.trajectory_.d = traj.d;
  ego_car.trajectory_.t = traj.t;
}
