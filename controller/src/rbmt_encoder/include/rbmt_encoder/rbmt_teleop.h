#ifndef RBMT_ENCODER_H_
#define RBMT_ENCODER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <map>
#include <cmath>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>


namespace rbmt_encoder {

class EncoderCount {
 public:
  EncoderCount(ros::NodeHandle nh);
  ~EncoderCount();
  void run(ros::Rate rate);
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber encoder_X_axis_sub_;
  ros::Subscriber encoder_Y_axis_sub_;
  ros::Publisher calc_vel_pub_;
  // ros::Publisher calc_odom_pub_;



  Int32 encoder_tick_x_;
  Int32 encoder_tick_y_;
  Int32 last_encoder_tick_x_;
  Int32 last_encoder_tick_y_;
  
  
  void encoder_X_axis_cb_(const std_msgs::Int32::ConstPtr& enc_msg);
  void encoder_Y_axis_cb_(const std_msgs::Int32::ConstPtr& enc_msg);
  // float axis_range(const size_t& ith);

  // float axis_range_ratio(const size_t& ith);

  // float vel_range(const std::string type);

  // float reverse(const float& val);

  // size_t send_goal(const move_base_msgs::MoveBaseGoal& goal, MoveBaseClient& ac);
};

}// namespace rbmt_encoder

#endif