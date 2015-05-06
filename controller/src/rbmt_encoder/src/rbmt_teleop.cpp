#include <rbmt_teleop/rbmt_teleop.h>

namespace rbmt_teleop {

EncoderCount::EncoderCount(ros::NodeHandle nh): nh_(nh) {

  encoder_X_axis_sub_ = nh_.subscribe<std_msgs::Int32>("/encoderX/encoder_X_axis",1, &EncoderCount::encoder_X_axis_cb_, this);
  encoder_Y_axis_sub_ = nh_.subscribe<std_msgs::Int32>("/encoderY/encoder_Y_axis",1, &EncoderCount::encoder_Y_axis_cb_, this);
  calc_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("calculated_velocity", 100);
  // calc_odom_pub_ = nh_.advertise<geometry_msgs::Pose2D>("calculated_odometry", 100);
}

EncoderCount::~EncoderCount() {

}

void EncoderCount::encoder_X_axis_cb_(const std_msgs::Int32::ConstPtr& enc_x_msg) {
  encoder_tick_x_ = enc_x_msg->data;
}

void EncoderCount::encoder_Y_axis_cb_(const std_msgs::Int32::ConstPtr& enc_y_msg) {
  encoder_tick_y_ = enc_y_msg->data;
}

void EncoderCount::run(ros::Rate rate) {
  const bool debug = false;
  const float enc_wheel_radius = 48/1000;

  while (ros::ok()) {
    // Set the values based on joy readings
    float speed_body_X_,speed_body_Y_;
    // float speed_body_X_
        
    // Publish
    geometry_msgs::Twist calc_vel;
    geometry_msgs::Pose2D calc_odom;

   // omega = Tetha / Delta_Time -- Delta_Time = 10ms
   // omega = Tetha / 10ms = ((tickEnc - last_tickEnc)/1440) * 2 * PI * 1000/10 rad/s
   // omega = (tickEnc - last_tickEnc) * 200/1440 * PI rad/s
   // omega = (tickEnc - last_tickEnc) * 200/1440 * PI * (1/(2PI)) rotation/rad rad/s
   // omega = (tickEnc - last_tickEnc) * 100/1440 rotation/s
   // omega = (tickEnc - last_tickEnc) * 100/1440 rotation/(1/60) minute
   // omega = (tickEnc - last_tickEnc) * 100/1440 * 60 rotation/minute
    //omega = (tickEnc - last_tickEnc) * 6000/1440 RPM
    speed_body_X_ = enc_wheel_radius * float(encoder_tick_x_ - last_encoder_tick_x_)*25/6 * 0.10472;
    speed_body_Y_ = enc_wheel_radius * float(encoder_tick_y_ - last_encoder_tick_y_)*25/6 * 0.10472; 
    
    calc_vel.linear.x = speed_body_X_; 
    calc_vel.linear.y = speed_body_Y_;
    calc_vel_pub_.publish(calc_vel);

    // calc_odom.x = ;
    // calc_odom.y = ;
    // calc_odom.theta = ;
    // calc_odom_pub_.publish(calc_odom);

    ros::spinOnce();
    rate.sleep();
  }
}

}// namespace rbmt_teleop
