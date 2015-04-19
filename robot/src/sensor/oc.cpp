#include "oc.h"

using namespace trui;

Oc::Oc() {
  // Motor::Motor (size_t pwm_pin_, size_t dir_pin1_, size_t dir_pin2_,/* size_t en_1_pin, size_t en_2_pin, size_t xy_pin,*/ size_t oe_pin, size_t sel_1_pin, size_t sel_2_pin, size_t reset_x_pin,/* size_t reset_y_pin,*/ size_t bit0, size_t bit1,size_t bit2, size_t bit3, size_t bit4, size_t bit5, size_t bit6, size_t bit7, float outmax_, float outmin_) : pwm_pin_(pwm_pin_), dir_pin1_(dir_pin1_), dir_pin2_(dir_pin2_), /*en_1_pin(en_1_pin), en_2_pin(en_2_pin), xy_pin(xy_pin),*/ oe_pin(oe_pin), sel_1_pin(sel_1_pin), sel_2_pin(sel_2_pin), reset_x_pin(reset_x_pin), /*reset_y_pin(reset_y_pin),*/ bit0(bit0) , bit1(bit1) , bit2(bit2) , bit3(bit3), bit4(bit4) , bit5(bit5) , bit6(bit6) , bit7(bit7), outmax_(outmax_), outmin_(outmin_){
  theta_read_=0, theta_input_=0, cmd_theta = 0;
  theta_read_k_1_=0;
  theta_read_k_2_ =0;
  theta_read_k_3_ =0;
  theta_read_k_4_ =0;
  theta_read_refined = 0;
  iTerm_= 0;    
  delta_=0, error_=0, last_error_=0; 
  kp_= 0.2, ki_= 0.05, kd_= 0;//0.01;
  outmin_ = 30;
  outmax_ = 30;
  // kp_= 0.5, ki_= 0.0528, kd_= 0;//for 50ms sampling time

  mpu = new MPU6050();
  setup();
}

Oc::~Oc() {
  delete mpu;
}


void Oc::setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  
  mpu->initialize();
  mpu->testConnection();
  devStatus = mpu->dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu->setXGyroOffset(220);
  mpu->setYGyroOffset(76);
  mpu->setZGyroOffset(-85);
  mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      mpu->setDMPEnabled(true);

      mpuIntStatus = mpu->getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      mpuInterrupt = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu->dmpGetFIFOPacketSize();
  } else {}
}


void Oc::reset(){
  
}

float Oc::read_ypr(){
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu->getIntStatus();

  // get current FIFO count
  fifoCount = mpu->getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu->resetFIFO();
//        Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu->getFIFOCount();

      // read a packet from FIFO
      mpu->getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu->dmpGetQuaternion(&q, fifoBuffer);
          mpu->dmpGetGravity(&gravity, &q);
          mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
          // Serial.print("ypr\t");
          // Serial.print(ypr[0] * (180/PI));
          // Serial.print("\t");
          // Serial.print(ypr[1] * (180/PI));
          // Serial.print("\t");
          // Serial.println(ypr[2] * (180/PI));
      #endif
  }

  return ypr[0]*(180/PI);
}

void Oc::testing_imu(){
  
}


float Oc::set_orientation(float cmd_theta,float current_theta) {
  theta_input_ = cmd_theta; //setpoint

  theta_read_k_ = current_theta;
  theta_read_refined = (theta_read_k_+ theta_read_k_1_ + theta_read_k_2_ + theta_read_k_3_ + theta_read_k_4_)/5;
  error_ = theta_read_refined - theta_input_;
  iTerm_ = iTerm_ + (float)error_*ki_;       //Integral Term of PID Control 
  
  // if(iTerm_ > outmax_) iTerm_ = outmax_;
  // else if(iTerm_ < outmin_) iTerm_ = outmin_;
                
  deriv_comp_ = (tick_enc_ - 2*last_tick_enc_ + last2_tick_enc_);//*numerator_/denominator_;
  
  if(error_ < 1 && error_ > -1) error_ = 0;              
  omega_ =  (float)error_*kp_;// + iTerm_ - deriv_comp_*kd_;
  


  // if(omega_ > outmax_) omega_ = outmax_;
  // else if(omega_ < outmin_) omega_ = outmin_;
                
  theta_read_k_4_ = theta_read_k_3_;
  theta_read_k_3_ = theta_read_k_2_;
  theta_read_k_2_ = theta_read_k_;
  last_error_ = error_;
  return omega_;
  // last2_tick_enc_ = last_tick_enc_; 
  // last_tick_enc_ = tick_enc_;    
}
