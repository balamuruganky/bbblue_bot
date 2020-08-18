#ifndef _ENCODER25GA20E260_H__
#define _ENCODER25GA20E260_H__

#include <common.h>
#include <encoder.h>
//#include <rc/encoder.h>

#define PPR25GA20E260 1040

class Encoder25GA20E260 : public IEncoder {
	public:
		Encoder25GA20E260(ros::NodeHandle nh, uint8_t channel, Position position) :
					IEncoder(nh, PPR25GA20E260, position), 
					channel(channel) {}
		Encoder25GA20E260(ros::NodeHandle nh, uint8_t channel, Position position, EncoderMode mode) : 
					IEncoder(nh, PPR25GA20E260, position, mode), 
					channel(channel) {}

		~Encoder25GA20E260() {}

	    /* virtual */ int 		getRawRotation();
	    /* virtual */ int8_t	init();
	    /* virtual */ void		deInit();
	private:
		uint8_t channel;
		int64_t rotation;
};

int Encoder25GA20E260::getRawRotation() {
	/*
	if (rotation = rc_encoder_read(channel)) { 
	    ROS_ERROR("Read encoder 25GA20E260, channel %d is failed", channel);
	    return -1;
	}
	*/

	return (rotation % PPR25GA20E260);
}

int8_t Encoder25GA20E260::init() {
	/*
	if (rc_encoder_init()) {
		ROS_ERROR("Init encoder 25GA20E260, channel %d is failed", channel);
		return -1;
	}
	*/
	return 0;
}

void Encoder25GA20E260::deInit() {
	//rc_encoder_cleanup();
}

#endif