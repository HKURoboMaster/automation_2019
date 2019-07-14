#include "chassis_calc.h"
#include <stdlib.h>

chassis_movement_t movement = {1, 0, 0}; // current movement of chassis
int duration_ceiling = 2500; // movement duration ceiling, default 2500ms
int duration_floor = 500; // movement duration floor, default 500ms

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
    }
    return movement.spd_ind * speed;
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
    movement.duration = duration_floor + rand() % (duration_ceiling - duration_floor);
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

/**
 * Jerry 14 Jul
 * @brief Set duration ceiling.
 */
void set_duration_ceiling(int new_duration_ceiling) {
    duration_ceiling = new_duration_ceiling;
}

/**
 * Jerry 14 Jul
 * @brief Set duration floor.
 */
void set_duration_floor(int new_duration_floor) {
    duration_floor = new_duration_floor;
}
