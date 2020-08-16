#ifndef _DCMOTOR_H__
#define _DCMOTOR_H__

#include "ros/ros.h"
//#include <rc/motor.h>

#include <encoder.h>
#include <encoder_25ga20e260.h>
#include <common.h>

class DCMotor {
  ros::Subscriber sub;

  const double freq_hz;
  //const double duty_cycle;
  const Position position;
  const uint8_t  channel;

  void motorCb(const std_msgs::Float32&);
  IEncoder *m_pEnc;

public:

  DCMotor(ros::NodeHandle nh, uint8_t channel, Position position);
  ~DCMotor();

  void publishAngle();
  int  init();
  void deInit();
  void stop();
  void move(float duty_cycle);
};

DCMotor::DCMotor(ros::NodeHandle nh, uint8_t channel, Position position) :  
     position(position)
    ,channel(channel)
    ,freq_hz(/*RC_MOTOR_DEFAULT_PWM_FREQ*/ 0)
    ,m_pEnc(NULL) {

  init();

  int update_rate = 5;
  ros::param::get("~update_rate", update_rate);
  if (position == LEFT) {
    sub = nh.subscribe("bbblue_bot/left_wheel_vel", update_rate, &DCMotor::motorCb, this);
  } else {
    sub = nh.subscribe("bbblue_bot/right_wheel_vel", update_rate, &DCMotor::motorCb, this);
  }
  ROS_INFO("wheel_vel is subscribed with the update rate %d", update_rate);

  stop();
  //
  // Encoder
  //
  m_pEnc = new Encoder25GA20E260(nh, channel, position);
  m_pEnc->init();
}

DCMotor::~DCMotor() {
  deInit();
  m_pEnc->deInit();
  delete m_pEnc;
}

void DCMotor::publishAngle() {
  return (m_pEnc->publishAngle());
}

int DCMotor::init() {
  //return (rc_motor_init_freq(freq_hz) ? -1 : 0);
  return 1;
}

void DCMotor::deInit() {
  stop();
  //rc_motor_cleanup();
}

void DCMotor::stop() {
  //rc_motor_brake(channel);
}

void DCMotor::move(float duty_cycle) {
  //rc_motor_set(channel, duty_cycle);
}

void DCMotor::motorCb(const std_msgs::Float32& msg) {
  move (msg.data);
}

#endif