#include <rbmt_motion/rbmt_motion.h>



namespace rbmt_motion {

bool MoveMotion::t_inc_flag= 0;

MoveMotion::MoveMotion(ros::NodeHandle nh): nh_(nh) {

  
  joy_sub_ = nh_.subscribe<std_msgs::Int16MultiArray>("read_joy",1, &MoveMotion::joy_sub_cb, this);//get controller command from /read_joy topic
  odom_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("calculated_odometry", 1, &MoveMotion::odom_sub_cb, this);
  kinect_sub_ = nh_.subscribe<geometry_msgs::Twist>("kinect_velocity",1, &MoveMotion::kinect_sub_cb, this);//get controller command from /read_joy topic
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("read_velocity", 100);//("traj_motion", 100);
  // cmd_service_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cmd_service", 100);
}

MoveMotion::~MoveMotion() {

}

void MoveMotion::timer_cb(const ros::TimerEvent& e){
  // ROS_INFO("Timer triggered");
  t_inc_flag = 1;
}

void MoveMotion::odom_sub_cb(const geometry_msgs::Pose2D::ConstPtr& odom_msg){
  x_encoder = odom_msg->x;
  y_encoder = odom_msg->y;
  // ROS_INFO("%f",y_encoder);
}

void MoveMotion::kinect_sub_cb(const geometry_msgs::Twist::ConstPtr& vel_msg) {
  vel_kinect_x_ = vel_msg->linear.x;
  vel_kinect_y_ = vel_msg->linear.y;
  vel_kinect_z_ = vel_msg->angular.z;
}

void MoveMotion::joy_sub_cb(const std_msgs::Int16MultiArray::ConstPtr& msg) {

  buttonL1_       = msg->data[12];
  if(buttonL1_ == 1){
    testFlag = 1;
  }
  else if(buttonL1_ == 0){
    //Reset encoders
  }

  
  // buttonsR1_
    // joy.data[0]  = ps2x.Analog(PSS_LX);
    // joy.data[1]  = ps2x.Analog(PSS_LY);
    // joy.data[2]  = ps2x.Analog(PSS_RX);
    // joy.data[3]  = ps2x.Analog(PSS_RY);
    // joy.data[4]  = ps2x.Button(PSB_TRIANGLE);
    // joy.data[5]  = ps2x.Button(PSB_CROSS);
    // joy.data[6]  = ps2x.Button(PSB_SQUARE);
    // joy.data[7]  = ps2x.Button(PSB_CIRCLE);
    // joy.data[8]  = ps2x.Button(PSB_PAD_DOWN);
    // joy.data[9]  = ps2x.Button(PSB_PAD_LEFT);
    // joy.data[10] = ps2x.Button(PSB_PAD_UP);
    // joy.data[11] = ps2x.Button(PSB_PAD_RIGHT);
    // joy.data[12] = ps2x.Button(PSB_L1);
    // joy.data[13] = ps2x.Button(PSB_L2);
    // joy.data[14] = ps2x.Button(PSB_R1);
    // joy.data[15] = ps2x.Button(PSB_R2);
    // joy.data[16] = ps2x.Button(PSB_START);
    // joy.data[17] = ps2x.Button(PSB_SELECT);
    // joy.data[18] = ps2x.Button(PSB_L3);
    // joy.data[19] = ps2x.Button(PSB_R3);
}
 
void MoveMotion::csv_write(const geometry_msgs::Twist& vel, const std::string& filepath) {
  using namespace std;
  using namespace boost;

  std::ofstream csv;
  csv.open(filepath.c_str(),ios::app);
  if ( csv.is_open() ) {
    // sPose.header.stamp = ros::Time::now();
    csv << lexical_cast<string>(vel.linear.x); csv << ",";
    csv << lexical_cast<string>(vel.linear.y); csv << ",";
    csv << lexical_cast<string>(vel.linear.z); csv << ",";
    csv << lexical_cast<string>(vel.angular.x); csv << ",";
    csv << lexical_cast<string>(vel.angular.y); csv << ",";
    csv << lexical_cast<string>(vel.angular.z); csv << ",";

    // if (new_sample)
    csv << "\n";
  }
  else {
    assert(false && "csv.open(filepath.c_str()): FALSE");
  }
  
  csv.close();
} 

float MoveMotion::PID_Y(float targetPosition, float currentPosition){
  float outputPID_speed;
  float temp;
  float p_term, i_term, d_term;

  errorPID_Y = targetPosition - currentPosition;

  // Calculate Pterm and limit error overflow
  // if (errorPID_Y > maxError){
    // p_term = MAX_INT;
  // }
  // else if (errorPID_Y < maxError){
    // p_term = -MAX_INT;
  // }
  // else{
    p_term = errorPID_Y;//P_Factor * errorPID_Y;
  // }

  // Calculate Iterm and limit integral runaway
  // if(I_Factor != 0){
    temp = sumError + errorPID_Y;
  //   if(temp > maxSumError){
  //     i_term = MAX_I_TERM;
  //     sumError = maxSumError;
  //   }
  //   else if(temp < -maxSumError){
  //     i_term = -MAX_I_TERM;
  //     sumError = -maxSumError;
  //   }
  //   else{
      sumError = temp;
      i_term = I_Factor * sumError;
    // }
  // }
  // else i_term =0;

  // Calculate Dterm
  d_term = D_Factor * (lastPosition - currentPosition);

  lastPosition = currentPosition;

  outputPID_speed = (p_term + i_term + d_term) / SCALING_FACTOR;
  // if(outputPID_speed > MAX_INT){
  //   outputPID_speed = MAX_INT;
  // }
  // else if(outputPID_speed < -MAX_INT){
  //  outputPID_speed = -MAX_INT;
  // }

  return outputPID_speed;
}

void MoveMotion::run(ros::Rate rate) {
  const bool debug = false;

  // Set the values based on joy readings
  float speedX_,speedY_,speedW_;
  float q_X, q_Y, t;
  float deltaX = 0;
  float deltaY = 3;
  float T = 10;
  float q_double_dot_X,q_double_dot_Y;
  float x_zero,y_zero;
  // Publish
  geometry_msgs::Twist cmd_vel;
  // ros::Timer timer_sub_ = nh_.createTimer(ros::Duration(0.01), &MoveMotion::timer_cb);
  ROS_INFO("RBMT_MOTION is a go");
  sumError = 0;
  errorPID_Y = 0;
  x_zero = x_encoder;
  y_zero = y_encoder;
  while (ros::ok()) {

    //With a parabolic trajectory, acceleration is a constant
    q_double_dot_X = (2/(T*T))*deltaX; //acceleration along the x axis
    q_double_dot_Y = (2/(T*T))*deltaY; //acceleration along the y axis
    testFlag == 1;
    //Trajectory Generation
    
    if(t > T){
      speedX_ = 0;
      speedY_ = 0;
      speedW_ = 0;
      x_zero = x_encoder;
      y_zero = y_encoder;
      // t = 0;
    }
    
    else {
      if(t_inc_flag == 1  && testFlag == 1 && t <= T){
        t_inc_flag = 0;
        t = t + 0.01;
        q_X = (1/(T*T))*deltaX*(t*t);
        q_Y = (1/(T*T))*deltaY*(t*t);
        //csvWrite v_now dan x_encoder

        //For Open Loop Control
        speedW_ = 0;
        speedX_ = 0;//(q_X - (x_encoder - x_zero))/ 0.01;
        if(q_double_dot_Y * t < minSpeed) speedY_ = minSpeed;
          else 
          speedY_ = q_double_dot_Y * t;

        // For Closed Loop Control
        // speedY_ = PID_Y(q_Y, y_encoder);

        ROS_INFO("T = %f, t = %f, q_Y = %f, y_encoder = %f, speedY_ = %f, errorPID_Y = %f",T,t,q_Y,y_encoder, speedY_, errorPID_Y);
        

      }
    }

    //if target reached then stop
    

    // }
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.x= "<< cmd_vel.linear.x);
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.linear.y= "<< cmd_vel.linear.y);
    //ROS_DEBUG_STREAM_COND(debug, "cmd_vel.angular.z= "<< cmd_vel.angular.z);
    cmd_vel.linear.x = speedX_; //x_vel;
    cmd_vel.linear.y = speedY_;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;//x_encoder;
    cmd_vel.angular.z = speedW_;//y_encoder;//speedW_;//theta_vel;
    cmd_vel_pub_.publish(cmd_vel);  
    std::string csv_filepath = "/home/lintang-sutawika/krai/trui-bot-prj/controller/src/rbmt_motion/test.csv";
    csv_write(cmd_vel,csv_filepath);
    

    // ros::spin();
    ros::spinOnce();
    rate.sleep();
  }
}


}// namespace rbmt_motion
