#ifndef _IENCODER_H__
#define _IENCODER_H__

#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <math.h>

#include <common.h>

class IEncoder {
	float read2angle(uint16_t);
	float normalize(float);
    float GetAngle();

	bool angleMode;
	bool position;
	float initial_angle;
	uint16_t encoder_ppp;
	
    std_msgs::Float32 msg;
    ros::Publisher pub;

public:
	IEncoder(ros::NodeHandle nh, uint16_t ppp, Position position);
	IEncoder(ros::NodeHandle nh, uint16_t ppp, Position position, EncoderMode mode);
    void publishAngle();

	virtual 		~IEncoder();
    virtual int 	getRawRotation() {};
    virtual int8_t	init() {};
    virtual void	deInit() {};
};

IEncoder::IEncoder(ros::NodeHandle nh, uint16_t ppp, Position position)
	: IEncoder::IEncoder(nh, ppp, position, PLUS_MINUS_PI) {}

IEncoder::IEncoder(ros::NodeHandle nh, uint16_t ppp, Position position, EncoderMode mode)
	: encoder_ppp(ppp)
    , angleMode(mode) {

    init();

	int update_rate = 5;
	ros::param::get("~update_rate", update_rate);
    this->initial_angle = read2angle( getRawRotation() );
    if (position == LEFT) {
    	pub = nh.advertise<std_msgs::Float32>("bbblue_bot/left_wheel_angle", update_rate, false);
    } else {
    	pub = nh.advertise<std_msgs::Float32>("bbblue_bot/right_wheel_angle", update_rate, false);
    }
    ROS_INFO("wheel_angle is advertised with the update rate %d", update_rate);
}

IEncoder::~IEncoder() {
	deInit();
}

float IEncoder::GetAngle() {
	// Change sign of the sensor reading
	int8_t k;
	this->position == LEFT? k = -1 : k = 1;

	float current_angle = k * read2angle( getRawRotation() );

	return normalize(current_angle - this->initial_angle);
}

float IEncoder::read2angle(uint16_t register_output) {
    // M_PI defined in math.h for newer versions
	return (register_output * ((float)2*M_PI / encoder_ppp));
}

float IEncoder::normalize(float angle) {
	if (this->angleMode == PLUS_MINUS_PI) angle += M_PI;
	angle = fmod(angle, 2*M_PI);
	if (angle < 0) angle += 2*M_PI;
	if (this->angleMode == PLUS_MINUS_PI) angle -= M_PI;

	return angle;
}

void IEncoder::publishAngle() {
    msg.data = GetAngle();
    pub.publish(msg);
}

#endif
