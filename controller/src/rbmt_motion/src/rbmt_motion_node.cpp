#include <rbmt_motion/rbmt_motion.h>
#include <log4cxx/logger.h>


int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "motion_move");
  ros::NodeHandle nh;

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_motion::MoveMotion motion_move(nh);
  ros::Timer timer_sub_ = nh.createTimer(ros::Duration(0.01), rbmt_motion::MoveMotion::timer_cb);
  motion_move.run(ros::Rate(100));
  
  return(0);
}