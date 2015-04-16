#include <rbmt_mobility_test/rbmt_mobility_test.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "mobility_test");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_mobility_test::MobilityTest mobility_test(nh);
  mobility_test.run(ros::Rate(100));
  
  return(0);
}