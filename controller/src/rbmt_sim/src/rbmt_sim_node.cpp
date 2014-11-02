#include <rbmt_sim/rbmt_sim.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "sim");
  ros::NodeHandle nh;

  rbmt_sim::Simulator sim(nh);
  sim.run(ros::Rate(10));

  return(0);
}