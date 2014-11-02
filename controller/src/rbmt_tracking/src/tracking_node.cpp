#include <rbmt_tracking/tracker.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  
  rbmt_tracking::Tracker tracker(nh);
  // tracker.run(ros::Rate(10));

  ROS_INFO("Waiting for nav stuff to be ready");
  ros::Duration(5.0).sleep();

  tracker.run_dummy(ros::Rate(0.2));

  return(0);
}