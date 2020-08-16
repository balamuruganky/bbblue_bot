//
// ROS headers
//
#include "ros/ros.h"
#include <std_msgs/Float32.h>

#include <common.h>
#include <encoder.h>
#include <encoder_25ga20e260.h>
#include <dc_motor.h>

//
// librobotcontrol headers
//
//#include <rc/encoder.h>

int gUpdateRate = 10; // Update rate
int gLeftMotorCh = 1;
int gRightMotorCh = 2;

int main(int argc, char** argv) {
  // ROS and node initialize
  ros::init(argc, argv, "bbblue_wheel_odom");

  ros::NodeHandle nh;
 
  ROS_INFO("Initializing node %s in namespace: %s", ros::this_node::getName().c_str(),
           ros::this_node::getNamespace().c_str());

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  if (gLeftMotorCh < 1 or gLeftMotorCh > 2 or gRightMotorCh < 1 or gRightMotorCh > 2)   {
    ROS_ERROR("ERROR: Wrong parameter: left_motor/right_motor channel must be between 1-2");
    return -1;
  }

  ros::param::set("~update_rate", gUpdateRate);
  ros::param::set("~left_motor_channel", gLeftMotorCh);
  ros::param::set("~right_motor_channel", gRightMotorCh);

  DCMotor *pMotorLeft = new DCMotor(nh, gLeftMotorCh, LEFT);
  DCMotor *pMotorRight = new DCMotor(nh, gRightMotorCh, RIGHT);

  ros::Rate r(gUpdateRate);

  ROS_INFO("bbblue_wheel_odom node is up, the update rate is %d", gUpdateRate);

  while (ros::ok())  {
    ros::spinOnce();

    current_time = ros::Time::now();

    // publish the left and right encoder angle
    pMotorLeft->publishAngle();
    pMotorRight->publishAngle();

    last_time = current_time;

    r.sleep();
  }

  delete pMotorLeft;
  delete pMotorRight;

  ROS_INFO("bbblue_wheel_odom node is halted");
  return 0;
}
// %EndTag(FULLTEXT)%