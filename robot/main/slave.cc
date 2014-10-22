#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>
#include <actuator/motor.h>

int timer1_counter;
const size_t pwm_pin = 11;
const size_t dir_pin = 12; 
const float outmax = 100.0; 
const float outmin = -100.0; 

const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called

  
trui::Motor g_motor(pwm_pin, dir_pin, encoder_out_a_pin, encoder_out_b_pin, encoder_resolution, outmax, outmin);
static float g_cmd_speed = 0;

void setup_timer()
{
  
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 780;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{
  //interrupts();
  g_motor.set_speed(g_cmd_speed);
}

int main() {
  long timeNow = 0, timeOld = 0;
  init();// this needs to be called before setup() or some functions won't work there
  setup_timer();
  pinMode(13,OUTPUT);
  Serial.begin(57600);

  g_motor.setup();

  bool led_val = 0;
  while (true) {
    // //1. Wait msg from master
    const uint8_t channel = MAVLINK_COMM_0;
    const uint8_t set_speed_msgid = MAVLINK_MSG_ID_ATTITUDE;//MAVLINK_MSG_ID_MANUAL_SETPOINT;
    const uint8_t actual_speed_query_msgid = MAVLINK_MSG_ID_COMMAND_INT;
    
    mavlink_message_t rx_msg;
    mavlink_status_t rx_status;

    bool msg_found = false;
    while (!msg_found) {
      if (Serial.available() > 0) {
        if (mavlink_parse_char(channel, Serial.read(), &rx_msg, &rx_status)) {
          if ( (rx_msg.msgid==set_speed_msgid) ) {

            

            // If msg.roll is _not_ harcoded, undefined behavior happens :(
            // msg.roll = 50;// TODO remove me!

            break;
          } 
          else if ( (rx_msg.msgid==actual_speed_query_msgid) ) {
            break;
          }
        }
      }
    }

    // //2. Identify msg
    // //2A. If msgid = manual_setpoint, set speed control to cmd_speed
     if ( (rx_msg.msgid==set_speed_msgid) ) {
       mavlink_attitude_t msg;
       mavlink_msg_attitude_decode(&rx_msg, &msg);

    // //   // msg.roll = 50;// TODO remove me!
      // if (msg.roll == 50) {
      //        // msg.roll = 50;
      //         if (led_val==0) led_val = 1; else led_val = 0;

      //         g_cmd_speed = 50;
      //       }

      msg.roll = 50.0;
      if (msg.roll == (float)50) {
        // msg.roll = 50.0;// this hardcode here does not make the motor work
        g_cmd_speed = (float)msg.roll;
        if (led_val==0) led_val = 1; else led_val = 0;
      }      
    } 


    // //2B. If msgid = asking for actual speed, send back actual speed to master
    // else if ( (rx_msg.msgid==actual_speed_query_msgid) ) {
    //   float actual_speed;
    //   mavlink_message_t msg_to_master;
    //   uint8_t system_id= MAV_TYPE_RBMT;
    //   uint8_t component_id= MAV_COMP_ID_ARDUINO_SLAVE1;

    //   uint32_t time_boot_ms= millis(); 
    //   mavlink_msg_manual_setpoint_pack(system_id, component_id, &msg_to_master, time_boot_ms, actual_speed, 0., 0., 0., 0, 0);

    //   uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    //   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_master);

    //   Serial.write(buf, len);
    // }

    digitalWrite(13,led_val);
    delay(1000);

    // timeNow = millis();
    // if(timeNow - timeOld > 50){
    //   timeOld = timeNow;
    //   g_motor.set_speed(g_cmd_speed);    
    // }  
  }// while (true)

  return 0;
}
