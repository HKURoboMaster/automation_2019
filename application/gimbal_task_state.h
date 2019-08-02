#ifndef __GIMBAL_TASK_STATE_H__
#define __GIMBAL_TASK_STATE_H__

#ifdef GIMBAL_TASK_H_GLOBAL
  #define GIMBAL_TASK_H_EXTERN 
#else
  #define GIMBAL_TASK_H_EXTERN extern
#endif

#include "stdint.h"
#include "gimbal_task.h"


typedef enum gimbal_state_name {
  PATROL_STATE, AIM_STATE ,ATTACK_STATE
} gimbal_state_name_t;

typedef struct {
  gimbal_state_name_t state_name;
  uint8_t auto_aiming_flag;
	uint8_t attack_flag;
} gimbal_state_t;


void set_gimbal_state(gimbal_state_t * state, gimbal_state_name_t dest_state);
gimbal_state_name_t get_gimbal_state(const gimbal_state_t * state);
uint8_t get_auto_aiming_flag(const gimbal_state_t * state);
uint8_t get_auto_attack_flag(const gimbal_state_t * state);
#endif
