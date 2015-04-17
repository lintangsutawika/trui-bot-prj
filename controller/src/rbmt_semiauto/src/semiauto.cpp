#include <ros/ros.h>
#include <ros/console.h>

// ROS includes
#include <geometry_msgs/PoseStamped.h>

void 
semiauto (const geometry_msgs::PoseStamped& sPose)
{

 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "semiauto");
  ros::NodeHandle nh;

  // Create a ROS subscriber for raw cock pose
  ros::Subscriber sub = nh.subscribe ("transformed_pose", 1, semiauto);

  // Spin
  ros::spin();
}