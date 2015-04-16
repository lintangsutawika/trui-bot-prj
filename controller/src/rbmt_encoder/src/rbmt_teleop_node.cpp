#include <rbmt_encoder/rbmt_encoder.h>
#include <log4cxx/logger.h>

int main(int argc, char** argv){
  using namespace std;

  ros::init(argc, argv, "encoder_calc");
  ros::NodeHandle nh;
  
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  rbmt_encoder::EncoderCount encoder_calc(nh);
  encoder_calc.run(ros::Rate(100));
  
  return(0);
}