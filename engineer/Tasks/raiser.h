#ifndef RAISER_H
#define RAISER_H

struct raiser {
	float rest_angle;
	float rise_angle;
	float left_ecd_angle;
	float right_ecd_angle;
	float left_ecd_velocity;
	float right_ecd_velocity;
	int init;
	float current_target_angle;
	float RAISER_STATE;
};

#endif
