#include <rbmt_encoder/rbmt_encoder.h>

namespace rbmt_encoder {

EncoderCount::EncoderCount(ros::NodeHandle nh): nh_(nh) {

  encoder_X_axis_sub_ = nh_.subscribe<std_msgs::Int32>("/encoderX/encoder_X_axis",1, &EncoderCount::encoder_X_axis_cb_, this);
  encoder_Y_axis_sub_ = nh_.subscribe<std_msgs::Int32>("/encoderY/encoder_Y_axis",1, &EncoderCount::encoder_Y_axis_cb_, this);
  imu_Z_axis_sub_ = nh_.subscribe<geometry_msgs::Twist>("embedded_chat",1, &EncoderCount::imu_Z_axis_cb_, this);
  joy_sub_ = nh_.subscribe<std_msgs::Int16MultiArray>("read_joy",1, &EncoderCount::joy_sub_cb_, this);
  calc_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("calculated_velocity", 100);
  calc_odom_pub_ = nh_.advertise<geometry_msgs::Pose2D>("calculated_odometry", 100);
}

EncoderCount::~EncoderCount() {

}

void EncoderCount::encoder_X_axis_cb_(const std_msgs::Int32::ConstPtr& enc_x_msg) {
  encoder_tick_x_ = enc_x_msg->data;
}

void EncoderCount::encoder_Y_axis_cb_(const std_msgs::Int32::ConstPtr& enc_y_msg) {
  encoder_tick_y_ = enc_y_msg->data;
}

void EncoderCount::imu_Z_axis_cb_(const geometry_msgs::Twist::ConstPtr& imu_z_msg){
  imu_yaw_z_ = imu_z_msg->angular.z;
}

void EncoderCount::joy_sub_cb_(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  buttonSelect_   = msg->data[17];
}

void EncoderCount::run(ros::Rate rate) {
  const bool debug = false;
  const float enc_wheel_radius = 0.048;
  float x_zero_ = 0,y_zero_= 0;
  float theta_zero_;
  int zeroingFlag_ = 0;
  while (ros::ok()) {
    // Set the values based on joy readings
    float speed_body_X_,speed_body_Y_;
    // float speed_body_X_
        
    // Publish
    geometry_msgs::Twist calc_vel;
    geometry_msgs::Pose2D calc_odom;


    if(buttonSelect_ == 1 && zeroingFlag_ == 0) { 
      zeroingFlag_ = 1;
      x_zero_ = encoder_tick_x_;
      y_zero_ = encoder_tick_y_;
      theta_zero_ = imu_yaw_z_;
    }
    else if(buttonSelect_ == 0 && zeroingFlag_ == 1){
      zeroingFlag_ = 0;
    }
    
    speed_body_X_ = enc_wheel_radius * (encoder_tick_x_ - last_encoder_tick_x_)*25/6 * 0.10472;
    speed_body_Y_ = enc_wheel_radius * (encoder_tick_y_ - last_encoder_tick_y_)*25/6 * 0.10472;
    last_encoder_tick_x_ = encoder_tick_x_;
    last_encoder_tick_y_ = encoder_tick_y_;
    
    calc_vel.linear.x = speed_body_X_; 
    calc_vel.linear.y = speed_body_Y_;
    calc_vel_pub_.publish(calc_vel);

    calc_odom.x = -enc_wheel_radius *(encoder_tick_x_- x_zero_)/1440.0 * 3.14159;
    calc_odom.y = enc_wheel_radius * (encoder_tick_y_ - y_zero_)/1440 * 3.14159;
    calc_odom.theta = -(imu_yaw_z_ - theta_zero_);
    calc_odom_pub_.publish(calc_odom);

    ROS_INFO("odom_X_ : %f, odom_Y_ : %f, theta_ : %f, theta_zero_: %f",calc_odom.x,calc_odom.y, calc_odom.theta, x_zero_);
    ros::spinOnce();
    rate.sleep();
  }
}

}// namespace rbmt_encoder