//Rosserial Node for the RBMT01
//@author: lintang
//
//
#include <arduino/Arduino.h>
#include <Wire.h>
#include <ros_lib/ros.h>
#include <ros_lib/geometry_msgs/Twist.h>

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

//ros::NodeHandle nh;
//geo



  
  }
  return 0;
}