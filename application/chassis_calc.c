#include "chassis_calc.h"
#include <stdlib.h>

chassis_movement_t movement = {1, 0, 0}; // current movement of chassis
duration_settings_t sduration = {500, 2500}; // movement duration settings
bounded_movement_settings_t bmove = {0, 0}; // bounded movement settings
middle_dodge_settings_t mdodge = {0, 0, 0};
int chassis_loc = 0; // [for rail: __/--] 0: __ 1: / 2: --
int curr_loc_delta = 0, last_loc_delta = 0;
float accumulated_distance = 0;
int acc_dis_js = 0;

/**
 * Jerry 14 Jul
 * @brief Calculate and output chassis random movement speed.
 * @param speed: constant speed of the state
 * @retval chassis output
 */
float chassis_random_movement(float speed) {
    uint32_t now = HAL_GetTick();
    if (now - movement.start_time > movement.duration) {
        generate_movement(); // current movement expires, generate a new one
        if (mdodge.activated) {
            activate_bounded_movement(2000);
            mdodge.activated = 0;
        }
    }
    float output = movement.spd_ind * speed;
    if (bmove.activated) {
        if (accumulated_distance > bmove.range || accumulated_distance < -bmove.range) {
            generate_movement();
            output = movement.spd_ind * speed;
        }
        accumulated_distance += output / 100;
    }
    if (mdodge.activated) {
        last_loc_delta = curr_loc_delta;
        curr_loc_delta = chassis_loc - mdodge.loc_register;
        if (last_loc_delta == 0 && (curr_loc_delta == 1 || curr_loc_delta == -1)) {
            if (mdodge.loc_register == 1) move_to_middle();
            mdodge.start_time = now;
        } else if ((curr_loc_delta == 2 || curr_loc_delta == -2) && (last_loc_delta == 1 || last_loc_delta == -1)) {
            forced_movement(-movement.spd_ind, (now - mdodge.start_time) / 2);
        }
    }
    acc_dis_js = accumulated_distance;
    return output;
}

/**
 * Jerry 14 Jul
 * @brief Generate new random movement object.
 * Randomly choose duration between floor and ceiling,
 * reverse speed direction.
 */
void generate_movement(void) {
    movement.start_time = HAL_GetTick();
    srand(movement.start_time);
    movement.duration = sduration.floor + rand() % (sduration.ceiling - sduration.floor);
    movement.spd_ind = (movement.spd_ind == 1 ? -1 : 1);
}

/**
 * Jerry 14 Jul
 * @brief A forced movement to force sentry to go for a certain distance
 * in a certain direction.
 * @param spd_ind: speed indicator
 * @param duration: movement duration
 */
void forced_movement(int spd_ind, int duration) {
    movement.start_time = HAL_GetTick();
    movement.spd_ind = spd_ind;
    movement.duration = duration;
}

void move_to_middle(void) {
    deactivate_bounded_movement();
    curr_loc_delta = 0;
    last_loc_delta = 0;
    if (chassis_loc == 1) {
        forced_movement(1, 100000);
    } else if (chassis_loc == 0) {
        forced_movement(1, 100000);
    } else if (chassis_loc == 2) {
        forced_movement(-1, 100000);
    }
    mdodge.loc_register = chassis_loc;
    mdodge.activated = 1;
}

/**
 * Jerry 15 Jul
 * @brief Calculate chassis state.
 */
void state_calc(chassis_state_t *state, cv_static_event_t static_eve, power_event_t power_eve, armor_event_t armor_eve) {
    if (armor_eve == HIT_WITHIN_X_SEC) {
        if (power_eve == POWER_IN_SERIOUS_DEBT) {
            set_state(state, NORMAL_STATE);
        } else {
            set_state(state, BOOST_STATE);
        }
    } else {
        if (static_eve == ENEMY_NOT_DETECTED) {
            set_state(state, IDLE_STATE);
        } else {
            if (power_eve != POWER_NORMAL) {
                set_state(state, IDLE_STATE);
            } else {
                set_state(state, NORMAL_STATE);
            }
        }
    }
}

/**
 * Jerry 15 Jul
 * @brief Update states.
 */
void update_events(cv_dynamic_event_t *dynamic_eve,cv_static_event_t *static_eve, power_event_t *power_eve, armor_event_t* armor_eve) {
    // TODO: Update states
}

/**
 * Jerry 14 Jul
 * @brief Shorthand solution for Set duration.
 * @param if you do not wish to change a value, set it to -1
 */
void set_duration(int new_duration_floor, int new_duration_ceiling) {
    sduration.ceiling = new_duration_ceiling > 0 ? new_duration_ceiling : sduration.ceiling;
    sduration.floor = new_duration_floor > 0 ? new_duration_floor : sduration.floor;
}

/**
 * Jerry 15 Jul
 * @brief Activate bounded movement within the given range.
 * @param range: make a range of [-range, range]
 */
void activate_bounded_movement(int range) {
    bmove.activated = 1;
    bmove.range = range;
    accumulated_distance = 0;
}

/**
 * Jerry 15 Jul
 * @brief Deactivate bounded movement, let sentry move freely.
 */
void deactivate_bounded_movement(void) {
    bmove.activated = 0;
    bmove.range = 0;
    accumulated_distance = 0;
}

/**
 * Jerry 15 Jul
 * @brief Adjust accumulated distance to make it more accurate.
 */
void adjust_accumulated_distance(float adjust_vy) {
    if (bmove.activated) accumulated_distance -= (adjust_vy / 100);
}

/**
 * Jerry 15 Jul
 * @brief Set the chassis state to be one of the three states.
 * Example: set_state(&state, IDLE_STATE);
 */
void set_state(chassis_state_t * state, chassis_state_name_t dest_state) {
  state->state_name = dest_state;
  switch (dest_state) {
    case IDLE_STATE:
      state->constant_spd = IDLE_CONSTANT_SPEED;
      set_duration(500, 2500);
      break;
    case NORMAL_STATE:
      state->constant_spd = NORMAL_CONSTANT_SPEED;
      set_duration(400, 1500);
      break;
    case BOOST_STATE:
      state->constant_spd = BOOST_CONSTANT_SPEED;
      set_duration(300, 1000);
  }
}

/**
 * Jerry 10 Jul
 * @brief Get the chassis state name.
 */
chassis_state_name_t get_state(const chassis_state_t * state) {
  return state->state_name;
}

/**
 * Jerry 10 Jul
 * @brief Get the constant speed under current state.
 */
float get_spd(const chassis_state_t * state) {
  return state->constant_spd;
}
