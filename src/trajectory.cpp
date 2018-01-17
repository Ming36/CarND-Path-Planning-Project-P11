//
//  trajectory.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "trajectory.hpp"

void GetTrajectory(EgoVehicle &ego_car, const double t_tgt,
                   const std::vector<double> &maps_x,
                   const std::vector<double> &maps_y,
                   const std::vector<double> &maps_s) {

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;
  std::vector<double> next_s_vals;
  std::vector<double> next_d_vals;
  std::vector<double> next_t_vals;
  
  const double v_tgt = mph2mps(kTargetSpeed); // mph -> m/s
  const double d_tgt = ego_car.d_; // DUMMY Keep lane
  
  // Generate D trajectory
  double s_est;
  double s_dot_est;
  double s_dot_dot_est;
  
  /*
  constexpr double min_t = 1.0; // sec
  double t_est;
  double a_est = (target_speed - ego_car.s_dot_) / min_t;

  if (a_est > max_a) {
    a_est = max_a;
    t_est = (target_speed - ego_car.s_dot_) / max_a;
  }
  else {
    t_est = min_t;
  }
  */
  
  // Set estimated s, v, a to keep a reasonable JMT with basic kinematics
  double t_amax = (v_tgt - ego_car.s_dot_) / kMaxA;
  
  if (t_amax > t_tgt) {
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
  
  ego_car.coeffs_JMT_s_dot_ = {ego_car.coeffs_JMT_s_[1], 2*ego_car.coeffs_JMT_s_[2], 3*ego_car.coeffs_JMT_s_[3], 4*ego_car.coeffs_JMT_s_[4], 5*ego_car.coeffs_JMT_s_[5]};
  
  ego_car.coeffs_JMT_s_dot_dot_ = {2*ego_car.coeffs_JMT_s_[2], 6*ego_car.coeffs_JMT_s_[3], 12*ego_car.coeffs_JMT_s_[4], 20*ego_car.coeffs_JMT_s_[5]};
  
  // Generate D trajectory
  
  double d_est;
  double d_dot_est;
  double d_dot_dot_est;
  
  d_est = d_tgt;
  d_dot_est = 0;
  d_dot_dot_est = 0;
  
  // TODO Set estimated d, v, a to keep a reasonable JMT with basic kinematics
  
  std::vector<double> start_state_d = {ego_car.d_, ego_car.d_dot_, ego_car.d_dot_dot_};
  std::vector<double> end_state_d = {d_est, d_dot_est, d_dot_dot_est};
  
  ego_car.coeffs_JMT_d_ = JMT(start_state_d, end_state_d, t_tgt);
  
  ego_car.coeffs_JMT_d_dot_ = {ego_car.coeffs_JMT_d_[1], 2*ego_car.coeffs_JMT_d_[2], 3*ego_car.coeffs_JMT_d_[3], 4*ego_car.coeffs_JMT_d_[4], 5*ego_car.coeffs_JMT_d_[5]};
  
  ego_car.coeffs_JMT_d_dot_dot_ = {2*ego_car.coeffs_JMT_d_[2], 6*ego_car.coeffs_JMT_d_[3], 12*ego_car.coeffs_JMT_d_[4], 20*ego_car.coeffs_JMT_d_[5]};
  
  for(int i = 0; i < (t_tgt / kSimCycleTime); i++) {
    double t = (i+1) * kSimCycleTime;
    
    double new_car_s = EvalPoly(t, ego_car.coeffs_JMT_s_);
    double new_car_d = EvalPoly(t, ego_car.coeffs_JMT_d_);
    
    std::vector<double> new_car_xy = GetXY(new_car_s, new_car_d,
                                           maps_s, maps_x, maps_y);
    
    next_x_vals.push_back(new_car_xy[0]);
    next_y_vals.push_back(new_car_xy[1]);
    next_s_vals.push_back(new_car_s);
    next_d_vals.push_back(new_car_d);
    next_t_vals.push_back(t);
  }
  
  ego_car.trajectory_.x = next_x_vals;
  ego_car.trajectory_.y = next_y_vals;
  ego_car.trajectory_.s = next_s_vals;
  ego_car.trajectory_.d = next_d_vals;
  ego_car.trajectory_.t = next_t_vals;
}
