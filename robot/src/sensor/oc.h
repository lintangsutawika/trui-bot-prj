#ifndef OC_H_
#define OC_H_

#include <stdint.h>
#include <arduino/Arduino.h>
#include <MPU6050/I2Cdev.h>
#include <MPU6050/MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars



namespace trui {

class Oc {
   public:
    Oc();
    
    ~Oc();


    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


    float set_orientation(float cmd_speed);
    void setup();
    void reset();
    void outSignal(float pwm);
    void PIDvelocity_algorithm(float speed,float Kp,float Ki,float Kd,float delta_T);
    int64_t read_ypr();
    void testing_imu();

   private:
    
    // orientation/motion vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    size_t pwm_pin_;
    size_t dir_pin1_;
    size_t dir_pin2_;
    int64_t tick_;
    int64_t tick_enc_;
    int64_t last_tick_enc_;
    int64_t last2_tick_enc_;
    int64_t deriv_comp_;
    float omega_;
    float omega_read_;
    float omega_read_k_;
    float omega_read_k_1_;
    float omega_read_k_2_;
    float omega_read_k_3_;
    float omega_read_k_4_;
    float omega_read_refined;
    float omega_input_;
    float last_omega_;
    float mv_;
    float iTerm_;
    float delta_;
    float error_;
    float last_error_;
    float error_over_time_;
    float kp_;
    float ki_;
    float kd_;
    float outmax_;
    float outmin_;
    uint8_t data_;
    float ocr_;
    float e_k_;
    float e_k_1_;
    float e_k_2_;
    float U_t_;
    float U_t_1_;

  };

}// namespace trui

#endif