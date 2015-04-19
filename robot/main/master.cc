//Rosserial Node for the RBMT01
//@author: lintang
//
//
#include <arduino/Arduino.h>
#include <Wire.h>
#include <ros_lib/ros.h>
#include <ros_lib/geometry_msgs/Twist.h>
#include <MPU6050/I2Cdev.h>
#include <MPU6050/MPU6050_6Axis_MotionApps20.h>

//Robot Kinematic Properties
#define L 0.45 //in meters, distance from wheel to center
#define R 0.1 //in meters, radius of wheel

//Speed Limiting Factor
#define rot_speedFactor 1//0.1
#define trans_speedXFactor 1//0.3
#define trans_speedYFactor 1//2.2


#define debugPin 13
#define hitPin 20
#define risePin 21
#define miscPneu1 22
#define miscPneu2 23
#define reset1 41
#define reset2 42
#define reset3 43

#define initialSpeed 0

#define DONE 1
#define OUTPUT_READABLE_YAWPITCHROLL
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;

ros::Publisher chatter("embedded_chat", &twist_msg);


int sendspeed1,sendspeed2,sendspeed3,hitBuffer;
float speedX_fromTwist,speedY_fromTwist,speedW_fromTwist;//omega;
float speedX,speedY,speedW,omega;
int theta, dTheta;
int timerCounterX,timerCounterY;
bool do_accelerationX = 0;
bool do_accelerationY = 0;
bool do_decceleration = 0;
bool accelerationAction = 0;
bool deccelerationAction = 0;
// int accel_timerIncY;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

MPU6050 mpu;


void assignSpeed( const geometry_msgs::Twist& rbmt_vel){
float v1,v2,v3;

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

if(rbmt_vel.angular.x == 1) digitalWrite(hitPin,HIGH); else digitalWrite(hitPin,LOW);

// speedX = -trans_speedXFactor*speedX;
// speedY = trans_speedYFactor*speedY;

//Acceleration & Decceleration profile
float max_AccelerationX = 1;
float max_DecelerationX = 1;
float max_AccelerationY = 3;
float max_DecelerationY = 3;
// accelration -> 0 to 1 or 0 to -1, decceleration 1 to 0 or -1 to 0;
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

v1 = (-sqrt(3.0)/2.0)*speedY - speedX/2 + rot_speedFactor*L*speedW; //rot_speedFactor*L*omega;
v2 = +speedX + rot_speedFactor*L*speedW; //rot_speedFactor*L*omega;
v3 = (sqrt(3.0)/2.0)*speedY - speedX/2  + rot_speedFactor*L*speedW; //rot_speedFactor*L*omega;


sendspeed1 = v1/R*9.55; //Covert rad/s to RPM 
sendspeed2 = v2/R*9.55; //Covert rad/s to RPM
sendspeed3 = v3/R*9.55; //Covert rad/s to RPM 
}

ISR(TIMER1_COMPA_vect){
  if(do_accelerationX == 0){do_accelerationX = 1;}
  if(do_accelerationY == 0){do_accelerationY = 1;}
  //PID for orientation control
}

ros::Subscriber<geometry_msgs::Twist> sub("read_velocity", &assignSpeed );

void toggleReset(){

// digitalWrite(reset1,HIGH);
// digitalWrite(reset2,HIGH);
// digitalWrite(reset3,HIGH);
// delay(500);
digitalWrite(reset1,LOW);
digitalWrite(reset2,LOW);
digitalWrite(reset3,LOW);
delay(500);
digitalWrite(reset1,HIGH);
digitalWrite(reset2,HIGH);
digitalWrite(reset3,HIGH);
}

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

void initIMU(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    
    mpu.initialize();
    mpu.testConnection();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        mpuInterrupt = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {}

}

void readYPR(){


    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * (180/PI));
            // Serial.print("\t");
            // Serial.print(ypr[1] * (180/PI));
            // Serial.print("\t");
            // Serial.println(ypr[2] * (180/PI));
        #endif
    }
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
  
  int fromSerial_1[2];
  int fromSerial_2[2];
  int fromSerial_3[2];

  init(); //Mandatory arduino setups, hardware registers etc
  intteruptSetup();
  initIMU();
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

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
    readYPR();
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
    twist_msg.linear.x  = speedY;
    twist_msg.linear.y  = speedX;
    twist_msg.linear.z  = 0;
    twist_msg.angular.x = ypr[0] * (180/PI);
    twist_msg.angular.y = ypr[1] * (180/PI);
    twist_msg.angular.z = ypr[2] * (180/PI);

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
  return 0;
}