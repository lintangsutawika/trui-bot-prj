//Rosserial Node for the RBMT01
//@author: lintang
//
//
#include <arduino/Arduino.h>
#include <Wire.h>
#include <ros_lib/ros.h>
#include <ros_lib/geometry_msgs/Twist.h>
#include <sensor/oc.h>

//Robot Kinematic Properties
#define L 0.45 //in meters, distance from wheel to center
#define R 0.1 //in meters, radius of wheel

//Speed Limiting Factor
#define rot_speedFactor 1//0.1
#define trans_speedXFactor 1//0.3
#define trans_speedYFactor 1//2.2


#define debugPin 13
#define hitPin 24
#define risePin 25
#define miscPneu1 26
#define miscPneu2 27
#define reset1 41
#define reset2 42
#define reset3 43

#define initialSpeed 0

#define DONE 1
#define OUTPUT_READABLE_YAWPITCHROLL
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;
ros::Publisher chatter("embedded_chat", &twist_msg);

trui::Oc* g_oc;
  

int sendspeed1,sendspeed2,sendspeed3,hitBuffer;
float speedX_fromTwist,speedY_fromTwist,speedW_fromTwist;//omega;
float speedX,speedY,speedW,omega, readYaw;
float setYaw = 0;
int theta, dTheta;
int timerCounterX,timerCounterY;
bool do_accelerationX = 0;
bool do_accelerationY = 0;
bool do_decceleration = 0;
bool accelerationAction = 0;
bool deccelerationAction = 0;

bool toggleFlag = 0;
// int accel_timerIncY;

void toggleReset(){

// digitalWrite(reset1,HIGH);
// digitalWrite(reset2,HIGH);
// digitalWrite(reset3,HIGH);
delay(100);
digitalWrite(reset1,LOW);
digitalWrite(reset2,LOW);
digitalWrite(reset3,LOW);
digitalWrite(13,LOW);
delay(500);
digitalWrite(reset1,HIGH);
digitalWrite(reset2,HIGH);
digitalWrite(reset3,HIGH);
digitalWrite(13,HIGH);
}

void assignSpeed( const geometry_msgs::Twist& rbmt_vel){
float v1,v2,v3;
int rbmt_command;

//      Vy
//      ^
//      |
//      |
//      |-------> Vx

// sqrt(3.0)/2.0, -1.0/2.0, -1.0,
//   0.0        ,  1.0    , -1.0
//-sqrt(3.0)/2.0, -1.0/2.0, -1.0;

speedX_fromTwist = rbmt_vel.linear.x;
speedY_fromTwist = rbmt_vel.linear.y;
speedW_fromTwist = rbmt_vel.angular.z;
rbmt_command     = (int)rbmt_vel.angular.y;
if((rbmt_command & B00000001) == 1) {} else if((rbmt_command & B00000100) == 0) {}
if((rbmt_command & B00000010)>>1 == 1) {} else if((rbmt_command & B00000100)>>1 == 0) {}
if((rbmt_command & B00000100)>>2 == 1) {} else if((rbmt_command & B00000100)>>2 == 0){}
if((rbmt_command & B00001000)>>3 == 1) {toggleFlag = 1;} else if((rbmt_command & B00000100)>>2 == 0) {toggleFlag = 0;}
if(rbmt_vel.angular.x == 1) digitalWrite(hitPin,HIGH); else digitalWrite(hitPin,LOW);

// speedX = -trans_speedXFactor*speedX;
// speedY = trans_speedYFactor*speedY;
speedW = speedW_fromTwist;
//Acceleration & Decceleration profile
//determine the direction of the acceleration;
//
float max_Acceleration = 1;

float max_AccelerationX = max_Acceleration * cos(atan2(speedY_fromTwist, speedX_fromTwist));//all in m/s^2
float max_DecelerationX = 0;
float max_AccelerationY = max_Acceleration * sin(atan2(speedY_fromTwist, speedX_fromTwist));//all in m/s^2
float max_DecelerationY = 0;
max_AccelerationX = abs(max_AccelerationX);
max_AccelerationY = abs(max_AccelerationY);

//1) see if the target velocity is either positive of negative
//2) is the speed accelerating (target speed > current) or deccelerating (target speed < current)
if( speedX_fromTwist > 0){ //Clockwise direction
  if(speedX_fromTwist > speedX){ //accelerating
    if(do_accelerationX == 1){
      do_accelerationX = 0;
      speedX = speedX + max_AccelerationX*0.01;//max_Acceleration*(10/1000);
    }
  }
  else if (speedX_fromTwist < speedX){ //deccelerating
    if(do_accelerationX == 1){
      do_accelerationX = 0;
      speedX = speedX - max_DecelerationX*0.01;//max_Acceleration*(10/1000);
    }
  }
  else speedX = speedX_fromTwist;
}

else if(speedX_fromTwist < 0){ //Counter clockwise direction
  if(speedX_fromTwist > speedX){ //deceleration
    if(do_accelerationX == 1){
      do_accelerationX = 0;
      speedX = speedX + max_DecelerationX*0.01;//max_Acceleration*(10/1000);
    }
  }
  else if (speedX_fromTwist < speedX){ //acceleration
    if(do_accelerationX == 1){
      do_accelerationX = 0;
      speedX = speedX - max_AccelerationX*0.01;//max_Acceleration*(10/1000);
    }
  }
  else speedX = speedX_fromTwist;
}

else {
  speedX = 0;
}

if( speedY_fromTwist > 0){ //Clockwise direction
  if(speedY_fromTwist > speedY){ //accelerating
    if(do_accelerationY == 1){
      do_accelerationY = 0;
      speedY = speedY + max_AccelerationY*0.01;//max_Acceleration*(10/1000);
    }
  }
  else if (speedY_fromTwist < speedY){ //deccelerating
    if(do_accelerationY == 1){
      do_accelerationY = 0;
      speedY = speedY - max_DecelerationY*0.01;//max_Acceleration*(10/1000);
    }
  }
  else speedY = speedY_fromTwist;
}

else if(speedY_fromTwist < 0){ //Counter clockwise direction
  if(speedY_fromTwist > speedY){ //deceleration
    if(do_accelerationY == 1){
      do_accelerationY = 0;
      speedY = speedY + max_DecelerationY*0.01;//max_Acceleration*(10/1000);
    }
  }
  else if (speedY_fromTwist < speedY){ //acceleration
    if(do_accelerationY == 1){
      do_accelerationY = 0;
      speedY = speedY - max_AccelerationY*0.01;//max_Acceleration*(10/1000);
    }
  }
  else speedY = speedY_fromTwist;
}

else {
  speedY = 0;
}

// dTheta = speedW * 1/100;
// theta = theta + dTheta;
// omega = PID(theta);

v1 = (-sqrt(3.0)/2.0)*speedY - speedX/2 + rot_speedFactor*L*omega; //rot_speedFactor*L*omega;
v2 = +speedX + rot_speedFactor*L*omega; //rot_speedFactor*L*omega;
v3 = (sqrt(3.0)/2.0)*speedY - speedX/2  + rot_speedFactor*L*omega; //rot_speedFactor*L*omega;


sendspeed1 = v1/R*9.55; //Covert rad/s to RPM 
sendspeed2 = v2/R*9.55; //Covert rad/s to RPM
sendspeed3 = v3/R*9.55; //Covert rad/s to RPM 
}

ISR(TIMER1_COMPA_vect){
  if(do_accelerationX == 0){do_accelerationX = 1;}
  if(do_accelerationY == 0){do_accelerationY = 1;}
  //PID for orientation control
  if (g_oc != NULL) {
    omega = g_oc->set_orientation(0, readYaw);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("read_velocity", &assignSpeed );

void intteruptSetup(){
    // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;      // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
 
  // set compare match register to desired timer count:
  OCR1A = 155;//780;
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





int main() {

  //Timer0  
  //Setting   Divisor     Frequency
  //0x01        1           62500
  //0x02        8           7812.5
  //0x03        64          976.5625   <--DEFAULT
  //0x04        256         244.140625
  //0x05        1024        61.03515625

  //TCCR0B = TCCR0B & 0b11111000 | <setting>;
  const uint8_t header= 0xCE;
  const uint8_t footer= 0xEE;

  const uint8_t hit_mode= 0x0A;
  const uint8_t position_mode= 0x0B;
  const uint8_t hit_flag = 0x0F;
  const uint8_t rise_flag =0xF0;

  bool toggleCount = 0;
  
  int fromSerial_1[2];
  int fromSerial_2[2];
  int fromSerial_3[2];

  init(); //Mandatory arduino setups, hardware registers etc
  intteruptSetup();
  
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  g_oc = new trui::Oc();
  // Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);    
  Serial3.begin(9600);
  pinMode(hitPin,OUTPUT);
  pinMode(risePin,OUTPUT);
  pinMode(miscPneu1,OUTPUT);
  pinMode(miscPneu2,OUTPUT);

  pinMode(reset1,OUTPUT);
  pinMode(reset2,OUTPUT);
  pinMode(reset3,OUTPUT);
  digitalWrite(hitPin,LOW);
  digitalWrite(risePin,LOW);
  digitalWrite(miscPneu1,LOW);
  digitalWrite(miscPneu2,LOW);

  delay(1000);
  toggleReset();
  delay(500);

  speedX = 0;
  speedY = 0;
  speedW = 0;
  while (true){

   if(toggleFlag == 1 && toggleCount == 0){toggleCount = 1;toggleReset();}
   else if(toggleFlag == 0 && toggleCount == 1){toggleCount = 0;}

    readYaw = g_oc->read_ypr();
  // if(nh.connected()){
    if(!nh.connected()){
    //   while(true){
        sendspeed1 = 0;
        sendspeed2 = 0;
        sendspeed3 = 0;
    //   }
    }
  // } 

    // if(Serial1.available()){//&& Serial1.read() == 0xCE){
    //   fromSerial_1[0] = Serial1.read();
    //   // fromSerial_1[1] = Serial1.read();
    //   twist_msg.linear.x = (fromSerial_1[0]);// | (fromSerial_1[1]<<8));
    // }

    // if(Serial2.available()){// && Serial2.read() == 0xCE){
    //   fromSerial_2[0] = Serial2.read();
    //   // fromSerial_2[1] = Serial2.read();
    //   twist_msg.linear.y = (fromSerial_2[0]);// | (fromSerial_2[1]<<8));
    // }

    // if(Serial3.available()){// && Serial3.read() == 0xCE){
    //   fromSerial_3[0] = Serial3.read();
    //   // fromSerial_3[1] = Serial3.read();
    //   twist_msg.linear.z = (fromSerial_3[0]);// | (fromSerial_3[1]<<8));
    // }
    twist_msg.linear.x  = sendspeed1;//speedY;
    twist_msg.linear.y  = sendspeed2;//speedX;
    twist_msg.linear.z  = sendspeed3;
    twist_msg.angular.x = 0;//ypr[0] * (180/PI);
    twist_msg.angular.y = omega;//ypr[1] * (180/PI);
    twist_msg.angular.z = readYaw;//ypr[2] * (180/PI);

    chatter.publish( &twist_msg );
    
    uint8_t buffer1[7];
    uint8_t buffer2[7];
    uint8_t buffer3[7];


    buffer1[0]= header;
    buffer1[6]= footer;
    buffer1[1]= (sendspeed1 & 0x00FF);
    buffer1[2]= ((sendspeed1 & 0xFF00)>>8);
    buffer1[3]= 0;
    uint16_t checksum1= buffer1[0] + buffer1[1] + buffer1[2] + buffer1[6];
    buffer1[4]= checksum1 & 0x00FF;
    buffer1[5]= (checksum1 & 0xFF00) >> 8;


    analogWrite(13,buffer1[1]);

    buffer2[0]= header;
    buffer2[6]= footer;
    buffer2[1]= (sendspeed2 & 0x00FF);
    buffer2[2]= ((sendspeed2 & 0xFF00)>>8);
    buffer2[3]= 0;
    uint16_t checksum2= buffer2[0] + buffer2[1] + buffer2[2] + buffer2[6];
    buffer2[4]= checksum2 & 0x00FF;
    buffer2[5]= (checksum2 & 0xFF00) >> 8;


    buffer3[0]= header;
    buffer3[6]= footer;
    buffer3[1]= (sendspeed3 & 0x00FF);
    buffer3[2]= ((sendspeed3 & 0xFF00)>>8);
    buffer3[3]= 0;
    uint16_t checksum3= buffer3[0] + buffer3[1] + buffer3[2] + buffer3[6];
    buffer3[4]= checksum3 & 0x00FF;
    buffer3[5]= (checksum3 & 0xFF00) >> 8;

    //For normal operation
    Serial1.write(buffer1, 7);
    Serial2.write(buffer2, 7);
    Serial3.write(buffer3, 7);
    // delay(1);
  nh.spinOnce();
  
  }

  delete g_oc;
  return 0;
}