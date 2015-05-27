#ifndef RBMT_MOTION_H_
#define RBMT_MOTION_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
#include <map>
#include <cmath>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include "std_msgs/String.h"

#define P_Factor_speed 25.0
#define I_Factor_speed 0.0
#define D_Factor_speed 0.0

#define P_Factor_omega 0.0
#define I_Factor_omega 0.0
#define D_Factor_omega 0.0

#define MAX_I_TERM 3
#define maxSumError 3
#define MAX_INT 4
#define maxError 3
#define SCALING_FACTOR 1

#define minSpeed 0


namespace rbmt_motion {

class MoveMotion {
 public:
  MoveMotion(ros::NodeHandle nh);
  ~MoveMotion();
  void run(ros::Rate rate);
  void csv_write(const geometry_msgs::Twist& vel,const std::string& filepath); 
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  static void timer_cb(const ros::TimerEvent& e);
  static bool t_inc_flag;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber kinect_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_service_pub_;

  size_t n_axes_;
  size_t n_button_;

  
  bool reachFlag;
  

  int buttonL1_;
  bool testFlag;

  float vel_kinect_x_;
  float vel_kinect_y_;
  float vel_kinect_z_;

  float x_encoder;
  float y_encoder;
  float z_imu;

  float s_encoder;
  float psi;
  float theta_imu;

  float body_alpha;
  float body_omega;
  float body_accel;
  float body_vel;
  float accel_sum;
  float alpha_sum;

  float d_traj;
  float theta_traj;
  float deltaX;
  float deltaY;


  float sumError_speed;
  float lastPosition;
  float lastError;
  float errorPID_Y;

  float sumError_omega;
  float lastTheta;
  float errorPID_omega;
  // int buttons_;

  std::vector<float> axis_mins_;
  std::vector<float> axis_maxs_;
  std::vector<float> axis_normals_;

  std::map<std::string, float> vel_param_;

  //! Map the button or axis names to their number in either buttons_ or axes_
  std::map<std::string, size_t> num_;

/*!
 * axes.at(0) horizontal left analog
 * axes.at(1) vertical left analog
 * axes.at(2) LT
 * axes.at(3) horizontal right analog
 * axes.at(4) vertical right analog
 * axes.at(5) RT 
 * axes.at(6) horizontal left buttons
 * axes.at(7) vertical left buttons
 *
 * buttons.at(0) button A
 * buttons.at(1) button B
 * buttons.at(2) button X
 * buttons.at(3) button Y
 * buttons.at(4) LB
 * buttons.at(5) RB
 * buttons.at(6) button BACK
 * buttons.at(7) button START
 * buttons.at(8) 
 * buttons.at(9) left analog click
 * buttons.at(10) right analog click
 */
  
  void odom_sub_cb(const geometry_msgs::Pose2D::ConstPtr& odom_msg);
  void joy_sub_cb(const std_msgs::Int16MultiArray::ConstPtr& msg);
  void kinect_sub_cb(const geometry_msgs::Twist::ConstPtr& vel_msg);
  float PID_Y(float targetPositionX,float targetPositionY, float currentPositionX, float currentPositionY);
  float PID_Theta(float targetTheta, float currentTheta);
  // float axis_range(const size_t& ith);

  // float axis_range_ratio(const size_t& ith);

  // float vel_range(const std::string type);

  // float reverse(const float& val);

  // size_t send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac);
};

}// namespace rbmt_motion

#endif