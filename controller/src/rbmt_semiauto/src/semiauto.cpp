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

float x_last = 0;
float y_last = 0;
float z_last = 0;
float time_last = 0;
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

	float time_temp = sPose.header.stamp.toSec();
	float x_temp = -(sPose.pose.position.y);
  float y_temp = sPose.pose.position.x;
  float z_temp = sPose.pose.position.z;
 	
 	float time_out = time_temp - time_last;
  float x_out = x_temp - x_last;
  float y_out = y_temp - y_last;
  float z_out = z_temp - z_last;

  time_last = time_temp;
  x_last = x_temp;
  y_last = y_temp;
  z_last = z_temp;
  
	float v_x = x_out / time_out;
	float v_y = y_out / time_out;
	float v_z = z_out / time_out;

	eta = z_out / v_z;

	x_eta = v_x * eta;
	y_eta = v_y * eta;


}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "semiauto");
  ros::NodeHandle nh;

  // Create a ROS subscriber for raw cock '-> this really gives some negative inuendo' pose
  ros::Subscriber joy_sub = nh.subscribe ("read_joy",1, trigger);
  move_pub = nh.advertise<geometry_msgs::Twist>("read_velocity", 1,false);
  ros::Subscriber sub = nh.subscribe ("transformed_pose", 1, semiauto);	
  	
  while(ros::ok()) {
  	if (buttonL1 == 1) {
  		move.angular.x = buttonR1;
  		// ROS_INFO("hit is %f", move.angular.x);
  		if (count == 2) {
		  	
		  	move.linear.x = x_eta / eta;
		  	move.linear.y = y_eta / eta;
		  }
	 	}
  	
  	else {
  		count = 0;
  		move.angular.x = buttonR1;
			move.linear.x = 0;
			move.linear.y = 0; 
  	}

 	move_pub.publish(move);
  
  	//}// Spin
  	ros::spinOnce();
  }
}