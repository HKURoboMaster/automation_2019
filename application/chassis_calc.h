#ifndef __CHASSIS_CALC_H__
#define __CHASSIS_CALC_H__

#ifdef CHASSIS_CALC_H_GLOBAL
  #define CHASSIS_CALC_H_EXTERN 
#else
  #define CHASSIS_CALC_H_EXTERN extern
#endif

#include "chassis_task.h"

#define IDLE_CONSTANT_SPEED 500  //speed of idle state
#define NORMAL_CONSTANT_SPEED 1000 //speed of normal state
#define BOOST_CONSTANT_SPEED 3000  //speed of boost state

typedef enum chassis_state_name {
  IDLE_STATE, NORMAL_STATE, BOOST_STATE
} chassis_state_name_t;

//enemy static state detection from tx2
typedef enum cv_static_event {
  ENEMY_NOT_DETECTED,  //no enemy detected
  ENEMY_DETECTED_LEFT,  //enemy detected on the left
  ENEMY_DETECTED_RIGHT   //enemy detected on the right
} cv_static_event_t;

//enemy dynamic state detection from tx2
typedef enum cv_dynamic_event {
  ENEMY_STAY_STILL,  //enemy has no tendency to move
  ENEMY_LEFT_TO_RIGHT,  //enemy moves from left to right
  ENEMY_RIGHT_TO_LEFT  //enemy moves from right to left
} cv_dynamic_event_t;

//power information from pc
typedef enum power_event {
  POWER_NORMAL, //no or little use of buffer power
  POWER_IN_DEBT,  //noticeable use of buffer power
  POWER_IN_SERIOUS_DEBT //considerable use of buffer power
} power_event_t;

//armor information from pc
typedef enum armor_event {
  NO_HIT_FOR_X_SEC, //armor has not been hit for at least x seconds
  HIT_WITHIN_X_SEC //armor has been hit within x seconds
} armor_event_t;

typedef struct chassis_state_t {
  chassis_state_name_t state_name;
  float constant_spd;
} chassis_state_t;

typedef struct chassis_movement {
  int spd_ind; // speed indicator, 1 or -1
  int duration; // movement duration (ms)
  uint32_t start_time; // movement start time (ticks)
} chassis_movement_t;

typedef struct duration_settings {
  int floor; // movement duration floor, default 500ms
  int ceiling; // movement duration ceiling, default 2500ms
} duration_settings_t;

typedef struct bounded_movement_settings {
  int activated; // if this is true, random movement will fall in a certain range
  float left_position;
  float right_position;
} bounded_movement_settings_t;

typedef struct middle_dodge_settings {
  int activated;
  int loc_register;
  uint32_t start_time;
} middle_dodge_settings_t;

float chassis_random_movement(chassis_t pchassis, float speed);
void generate_movement(void);
void forced_movement(int spd_ind, int duration);
void set_duration(int new_duration_floor, int new_duration_ceiling);
void activate_bounded_movement(int range);
void deactivate_bounded_movement(void);
void adjust_accumulated_distance(float adjust_vy);
void move_to_middle(void);

void state_calc(chassis_state_t *state, cv_static_event_t static_eve, power_event_t power_eve, armor_event_t armor_eve);
void update_events(cv_dynamic_event_t *dynamic_eve,cv_static_event_t *static_eve, power_event_t *power_eve, armor_event_t* armor_eve);

void set_state(chassis_state_t * state, chassis_state_name_t dest_state);
chassis_state_name_t get_state(const chassis_state_t * state);
float get_spd(const chassis_state_t * state);

#endif
