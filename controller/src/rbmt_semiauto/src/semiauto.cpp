#include <ros/ros.h>
#include <ros/console.h>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>


ros::Publisher move_pub;
geometry_msgs::Twist move;


float v_x,v_y, v_z;
float x_last = 0;
float y_last = 0;
float z_last = 0;
uint64_t time_last = 0;
float x_eta,y_eta,eta;
int buttonL1,buttonR1;
int count = 0;


void 
trigger (const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  buttonL1       = msg->data[12];
	buttonR1       = msg->data[14];
}

void 
semiauto (const geometry_msgs::PoseStamped& sPose)
{
	count++;
	
  uint64_t time_temp = sPose.header.stamp.toNSec();
	float x_temp = -(sPose.pose.position.y);
  float y_temp = sPose.pose.position.x;
  float z_temp = sPose.pose.position.z;
 	
 	uint64_t time_out = time_temp - time_last;
  float x_out = x_temp - x_last;
  float y_out = y_temp - y_last;
  float z_out = z_temp - z_last;
  ROS_INFO("time_last = %d, time_temp = %d, time_out = %f", time_last, time_temp, time_out);
  
  time_last = time_temp;
  x_last = x_temp;
  y_last = y_temp;
  z_last = z_temp;
  
	v_x = x_out / ((float)time_out / 1000000000);
	v_y = y_out / ((float)time_out / 1000000000);
	v_z = z_out / ((float)time_out / 1000000000);
  eta = z_out / v_z;
  ROS_INFO("Vx : %f, Vy : %f, Vz : %f, ", v_x, v_y, v_z);
	
  if(count == 2){
  x_eta = x_temp + (v_x * eta);
	y_eta = y_temp + (v_y * eta);
  ROS_INFO("x_eta : %f, y_eta : %f, ", x_eta, y_eta);
  }

}

void run(ros::Rate rate){

    while(ros::ok()) {
   
      move.angular.x = buttonR1;
      // ROS_INFO("hit is %f", move.angular.x);
      // if (count >= 2) {
    if (buttonL1 == 1) {     
        move.linear.x = x_eta / eta;
        move.linear.y = y_eta / eta;
      }
  //  }
    
    else {
    //  count = 0;
    // // move.angular.x = buttonR1;
      move.linear.x = 0;
      move.linear.y = 0;
      x_eta = 0;
      y_eta = 0;
      x_temp = 0;
      y_temp = 0; 
    }
      // move.linear.y = v_y;
      move_pub.publish(move);
  
    //}// Spin
    ros::spinOnce();
    rate.sleep();
  }
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "semiauto");
  ros::NodeHandle nh;

  // Create a ROS subscriber for raw cock '-> this really gives some negative inuendo' pose
  ros::Subscriber joy_sub = nh.subscribe ("read_joy",1, trigger);
  move_pub = nh.advertise<geometry_msgs::Twist>("read_velocity", 100);
  ros::Subscriber sub = nh.subscribe ("transformed_pose", 1, semiauto);	
  	
  run(ros::Rate(100));
}