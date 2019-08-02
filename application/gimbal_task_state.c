#include "gimbal_task_state.h"
#include <stdlib.h>


void set_gimbal_state(gimbal_state_t * state, gimbal_state_name_t dest_state) {
  state->state_name = dest_state;
   switch (dest_state) {
    case PATROL_STATE:
      state->auto_aiming_flag = 0;
			state->attack_flag = 0;
      break;
		case AIM_STATE:
			state->auto_aiming_flag = 1;
			state->attack_flag = 0;
			break;
    case ATTACK_STATE:
      state->auto_aiming_flag = 1;
			state->attack_flag = 0;
      break;
	}
}

gimbal_state_name_t get_gimbal_state(const gimbal_state_t * state) {
  return state->state_name;
}

uint8_t get_auto_aiming_flag(const gimbal_state_t * state) {
  return state->auto_aiming_flag;
}

uint8_t get_auto_attack_flag(const gimbal_state_t * state) {
	return state->attack_flag;
}