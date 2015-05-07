#include <arduino/Arduino.h>
#include <comm/arduino_mavlink_packet_handler.hpp>
#include <comm/custom.h>
#include <actuator/sc.h>
#include <sensor/two_phase_incremental_encoder.hpp>

#define definedSpeed 100 //RPM

int timer1_counter;
const int pwm_pin = 11;
const int dir_pin1 = 12;
const int dir_pin2 = 13;

const int encoder_out_a_pin = 2;
const int encoder_out_b_pin = 3;
const int encoder_resolution = 360; //Only needed for Initialization, not used unless .rot() is called

const int outmax = 255.0;
const int outmin = -255.0;


const uint8_t header= 0xCE;
const uint8_t footer= 0xEE;

const float Kp = 1.5;//0.58;
const float Ki = 0.5;//0.3;
const float Kd = 0.005;

float speed=0;
float bufferSpeed=0;

float e_k_=0;
float e_k_1_=0;
float e_k_2_=0;
float U_t_=0;
float U_t_1_=0;
float delta_T=0.05;
bool doFlag = 0;
bool resetFlag=0;
long error=0,thetaDot=0;
float u_k;
byte buffer[7] = {0,0,0,0,0,0,0};
int theta[100];
int theta_dot[100];

//Array to place measured data
byte dataFrom_Encoder[3] = {0xCE,0,0};
int tempRead_Encoder = 0;
bool sendFlag=0;
int i;


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
  speed = 0;

}

ISR(TIMER1_COMPA_vect)        // interrupt service routine 
{
  
  theta_dot[i] = sc.set_speed(speed);
  theta[i] = sc.get_pos();
  if(i < 100){
    i++; 
  }
  else {
    sendFlag =1; i = 101;
  }
  
  // if(requestIs_Valid == 0){
    
  //   arrayIndex+=2;
  //   if(arrayIndex == 1000) {requestIs_Valid=1; sendRequest = 1;}
  //   }

  
// sc.set_speed(30);
  
  //thetaDot = sc.read_encoder();
  // if(buffer[3] == 0xCC) speed = -bufferSpeed;
  //   else if(buffer[3] == 0x0C) speed = bufferSpeed;
   
  // error = speed - thetaDot;//cmdSetpoint - theta;
  // speed = 300;
  // sc.PIDvelocity_algorithm(speed,Kp,Ki,Kd,delta_T);


}

int main() {
  init();// this needs to be called before setup() or some functions won't work there
  setup();

  sc.setup();
  // sc.reset();
  Serial.begin(9600); //For normal operation
  // Serial.begin(115200); //Debugging use only

  long timeNow = 0, timeOld = 0;
  int incoming_byte = 0;
  
  int flag=0;
  int j = 0;
  while (true) {

    if(sendFlag == 0) speed = definedSpeed;
    else {
      speed = 0;
      sendFlag=0;
      for(j = 0; j<100; j++){
        Serial.print(theta[j]);
        Serial.print(",");
        Serial.println(theta_dot[j]);
      }
    break; //Exit loop;
    }    

  }

  return 0;
}
