//
//  path_common.cpp
//  Path_Planning
//
//  Created by Student on 1/12/18.
//

#include "path_common.hpp"

/**
 * Calculate Cartesian distance between two (x,y) points
 */
double Distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

/**
 * Find closest waypoint ahead or behind
 */
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y) {
  
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  
  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = Distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

/**
 * Find next waypoint ahead
 */
int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y) {
  
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
  
  //std::cout << "closest waypt: " << closestWaypoint;
  
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  
  double heading = atan2((map_y-y), (map_x-x));
  
  double angle = fabs(theta-heading);
  angle = std::min(2*pi()-angle, angle);
  
  if (angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }
  
  //std::cout << ", next waypt: " << closestWaypoint << std::endl;
  
  return closestWaypoint;
}

/**
 * Transform a position from Cartesian x,y coordinates to Frenet s,d coordinates
 */
std::vector<double> GetFrenet(double x, double y, double theta,
                              const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {
  
  // Find previous and next waypoints from map data
  int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);
  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size() - 1;
  }
  
  /*
   // TODO: Improve accuracy
   int close_wp = ClosestWaypoint(x, y, maps_x, maps_y);
   int next_wp = close_wp + 1;
   if (next_wp > maps_x.size() - 1) {
   next_wp  = 0;
   }
   int prev_wp = close_wp - 1;
   if (prev_wp < 0) {
   prev_wp = maps_x.size() - 1;
   }
   */
  
  //std::cout << "prev waypt: " << prev_wp << ", next_wp: " << next_wp << std::endl;
  
  // Waypoint vector x and y components from prev waypoint to next waypoint
  double vx_wp = maps_x[next_wp] - maps_x[prev_wp];
  double vy_wp = maps_y[next_wp] - maps_y[prev_wp];
  
  // Position vector x and y components from prev waypoint to position coord
  double vx_pos = x - maps_x[prev_wp];
  double vy_pos = y - maps_y[prev_wp];
  
  // Find the projection of position vector onto waypoint vector
  double proj_length = ((vx_pos * vx_wp + vy_pos * vy_wp) /
                        (vx_wp * vx_wp + vy_wp * vy_wp));
  
  // Find the endpoint (x,y) of projection onto waypoint vector
  double proj_endpt_x = proj_length * vx_wp;
  double proj_endpt_y = proj_length * vy_wp;
  
  // Frenet d value is the normal distance from the endpoint to the position coord
  double frenet_d = Distance(vx_pos, vy_pos, proj_endpt_x, proj_endpt_y);
  
  // See if d value is positive or negative by comparing it to a center point (1000, 2000)
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = Distance(center_x, center_y, vx_pos, vy_pos);
  double centerToRef = Distance(center_x, center_y, proj_endpt_x, proj_endpt_y);
  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }
  
  // Calculate s value
  double frenet_s = 0;
  // Accumulate s values up to the previous waypoint
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += Distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }
  // Add length of the position coord's projection along waypoint vector
  frenet_s += Distance(0, 0, proj_endpt_x, proj_endpt_y);
  
  return {frenet_s, frenet_d};
}

/**
 * Calculate Frenet velocities s_dot,d_dot from vx,vy and d normal vector
 * based on closest map waypoint's dx,dy
 */
std::vector<double> GetFrenetVelocity(double vx, double vy, int closest_wp,
                                      const std::vector<double> &maps_dx,
                                      const std::vector<double> &maps_dy) {
  
  double dx = maps_dx[closest_wp];
  double dy = maps_dy[closest_wp];
  double frenet_d_dot = (vx * dx + vy * dy);
  double frenet_s_dot = sqrt(vx*vx + vy*vy - frenet_d_dot*frenet_d_dot);
  
  //std::cout << "vx:" << vx << ",vy:" << vy << ",dx:" << dx << ",dy:" << dy << ",d_dot:" << frenet_d_dot << ",s_dot:" << frenet_s_dot << std::endl;
  
  return {frenet_s_dot, frenet_d_dot};
}

/**
 * Transform from Frenet s,d coordinates to Cartesian x,y
 */
std::vector<double> GetXY(double s, double d, const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
  
  int prev_wp = -1;
  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1) % maps_x.size();
  
  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);
  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);
  
  double perp_heading = heading - pi()/2;
  
  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);
  
  return {x, y};
}

std::vector<double> JMT(std::vector<double> start, std::vector <double> end,
                        double t_end) {
  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time t_end.
   
   INPUTS
   
   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]
   
   end - the desired end state for vehicle. Like "start" this is a
   length three array.
   
   t_end - The duration, in seconds, over which this maneuver should occur.
   
   OUTPUT
   
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   
   EXAMPLE
   
   > JMT( [0, 10, 0], [10, 10, 0], 1)
  
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  
  const double t_end2 = t_end * t_end;
  const double t_end3 = t_end2 * t_end;
  const double t_end4 = t_end3 * t_end;
  const double t_end5 = t_end4 * t_end;
  
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
  A << t_end3, t_end4, t_end5,
       3*t_end2, 4*t_end3, 5*t_end4,
       6*t_end, 12*t_end2, 20*t_end3;
  
  Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*t_end+0.5*start[2]*t_end2),
       end[1]-(start[1]+start[2]*t_end),
       end[2]-start[2];
  
  Eigen::MatrixXd Ai = A.inverse();
  Eigen::MatrixXd C = Ai * B;
  
  std::vector <double> result = {start[0], start[1], 0.5*start[2]};
  for(int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }
  
  return result;
}

double EvalPoly(double x, std::vector<double> coeffs){
  double y = 0;
  for (int i=0; i < coeffs.size(); ++i) {
    y += coeffs[i] * pow(x, i);
  }
  return y;
}

