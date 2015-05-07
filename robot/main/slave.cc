#include <arduino/Arduino.h>
#include <actuator/sc.h>
#include <sensor/two_phase_incremental_encoder.hpp>
#include <ros_lib/ros.h>
#include <ros_lib/std_msgs/Int32.h>

const int pwm_pin = 11;
const int dir_pin1 = 12;
const int dir_pin2 = 13;

const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called

const int outmax = 255.0;
const int outmin = -255.0;

bool sendFlag=0;
long tempRead_Encoder = 0;

ros::NodeHandle nh;
std_msgs::Int32 enc_msg;

ros::Publisher chatter("encoder_X_axis", &enc_msg);
// ros::Publisher chatter("encoder_Y_axis", &enc_msg);

trui::Sc sc(pwm_pin, dir_pin1,dir_pin2, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution,outmax, outmin);

void setup()
{
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 156;//780;
                    // target time = timer resolution * (timer counts + 1)
                   // (timer counts + 1) = target time / timer resolution
                   // (timer counts + 1) = 5*10^-3 / (1 / (16.10^6 / 1024))
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect){

  tempRead_Encoder = sc.get_encoder();
  sendFlag =1;
}

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  setup();
  nh.initNode();
  nh.advertise(chatter);
  while(true){
    enc_msg.data = tempRead_Encoder;
    if(sendFlag == 1){
      sendFlag=0;
      chatter.publish( &enc_msg );
    }
    // delay(1);
    nh.spinOnce();
  }
  return 0;
}
