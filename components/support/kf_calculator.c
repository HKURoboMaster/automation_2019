/****************************************************************************
 *  Copyright (C) HKU Robomaster 2018-2019 Herkules.
 *  Author: Y.H. Liu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "kf_calculator.h"
#define RM_OK 0

//getter
static input_t get_delta_input(void *); // delta_input
static kf_time_t get_delta_time(void *); // delta_time

//setter
static void set_current_time(void *, uint32_t); //update_current_time
static void set_speed_time(void *, int32_t); //update_speed_time
static void set_raw_speed(void *, float); // update_raw_speed
static void set_raw_angle(void *, float); // update_raw_angle

//Initializer
static int32_t initialize_kf_calculator(void *, kalman_filter_init_t *); //kf_calculator_init
static int32_t initialize_speed_calculator(void *, speed_calc_data_t *); // speed_calc_data_init

//Calculator
static float speed_calculator(void *); //speed_calculator
static float filtered_value_calculator(void *); //filter

//Return caculated value
static float ret_value(void *); //get_value

/** The function clear the data field and bond the function with the pointers
 * 
 * @author: Y.H. Liu
 * @param: obj - pointer to an newly declared instance of the kf_calculator_t
 * @return: RM_OK(0) if success; -1 otherwise
 */
int32_t kf_calculator_register(kf_calculator_t * obj)
{
  if(obj==NULL)
    return -1;
  memset(&(obj->_kf_body),0,sizeof(kalman_filter_t));
  memset(&(obj->_kf_last_tim),0,sizeof(kf_time_t));
  memset(&(obj->_kf_tim),0,sizeof(kf_time_t));
  memset(&(obj->_last_raw_data),0,sizeof(input_t));
  memset(&(obj->_raw_data),0,sizeof(input_t));
  memset(&(obj->_speed_struct),0,sizeof(speed_calc_data_t));

  obj->filtered_value = 0;

  obj->_delta_input = get_delta_input;
  obj->_delta_time = get_delta_time;

  obj->update_current_time = set_current_time;
  obj->update_speed_time = set_speed_time;
  obj->update_raw_speed = set_raw_speed;
  obj->update_raw_angle = set_raw_angle;

  obj->kf_calculator_init = initialize_kf_calculator;
  obj->speed_calc_data_init = initialize_speed_calculator;

  obj->speed_calculator = speed_calculator;
  obj->filter = filtered_value_calculator;
  obj->get_value = ret_value;

  return RM_OK;
}

/** The function retireves the delta raw values. 
 *  To be invoked by the kf_calculator_t::_delta_input(&this_obj). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 * @return: a struct consisting of delta angle and delta speed
 */
static input_t get_delta_input(void * this)
{
  input_t delta;

  if(this!=NULL)
	{
		kf_calculator_t * obj = (kf_calculator_t *)this;
		delta.angle = obj->_raw_data.angle - obj->_last_raw_data.angle;
		delta.speed = obj->_raw_data.speed - obj->_last_raw_data.speed;
	}
  return delta;
}

/** The function retireves the delta time values. 
 *  To be invoked by the kf_calculator_t::_delta_time(&this_obj). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 * @return: a struct consisting of timer ticks and delta speed_calc_time
 */
static kf_time_t get_delta_time(void * this)
{
  kf_time_t delta;
  
  if(this!=NULL)
  {
    kf_calculator_t * obj = (kf_calculator_t *)this;
    delta.tim_ms = obj->_kf_tim.tim_ms- obj->_kf_last_tim.tim_ms;
    delta.speed_calc_time = obj->_kf_tim.speed_calc_time - obj->_kf_last_tim.speed_calc_time;
  }

  return delta;
}

/** The function sets the kf_calculator_t::kf_time_t::tim_ms and 
 * update the stored previouse timer value. 
 *  To be invoked by the kf_calculator_t::update_current_time(&this_obj, <hal_tick>). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         time_ms - the timer ticks
 */
static void set_current_time(void * this, uint32_t time_ms)
{
  if(this!=NULL)
  {
    kf_calculator_t * obj = (kf_calculator_t *)this;
    obj->_kf_last_tim.tim_ms = obj->_kf_tim.tim_ms;
    obj->_kf_tim.tim_ms = time_ms;
  }
}

/** The function sets the kf_calculator_t::kf_time_t::speed_calc_time and 
 * update the stored previouse value and the kf_calculator_t::speed_calc_data_t. 
 *  To be invoked by the kf_calculator_t::update_speed_time(&this_obj, <hal_tick>). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         time_ms - the ticks at the moment when the last pc signals is passed in
 */
static void set_speed_time(void * this, int32_t time_ms)
{
  if(this!=NULL)
  {
    kf_calculator_t * obj = (kf_calculator_t *)this;
    obj->_kf_last_tim.speed_calc_time = HAL_GetTick();
    obj->_kf_tim.speed_calc_time = HAL_GetTick() - time_ms; //TODO: To be confirmed by Eric
  }
}

/** The function sets the kf_calculator_t::input_t::speed and the
 * previouse value. 
 *  To be invoked by the kf_calculator_t::update_raw_speed(&this_obj, raw_speed). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         speed - the input raw speed
 */
static void set_raw_speed(void * this, float speed)
{
  if(this==NULL)
    return;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  obj->_last_raw_data.speed = obj->_raw_data.speed;
  obj->_raw_data.speed = speed;
}

/** The function sets the kf_calculator_t::input_t::angle and the
 * previouse value. 
 *  To be invoked by the kf_calculator_t::update_raw_angle(&this_obj, raw_angle). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         angle - the input raw angle
 */
static void set_raw_angle(void * this, float angle)
{
  if(this==NULL)
    return;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  obj->_last_raw_data.angle = obj->_raw_data.angle;
  obj->_raw_data.angle = angle;
}

/** The function initializes the matrix in kf_calculator_t::kalman_filter_t
 *  To be invoked by the kf_calculator_t::kf_calculator_init(&this_obj, param_mat). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         kf_param - a set of given intializing parameters
 */
static int32_t initialize_kf_calculator(void * this, kalman_filter_init_t * kf_param)
{
  if(this==NULL)
    return -1;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  
  kf_param->xhat_data[0] = 0;
  kf_param->xhat_data[1] = 0;

  kalman_filter_init(&(obj->_kf_body),kf_param);
  return RM_OK;
}

/** The function intializes the kf_calculator_t::speed_calc_data_t
 *  To be invoked by the kf_calculator_t::speed_calc_data_init(&this_obj, speed_d). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 *         speed_calc - the intializing data for this->_speed_struct
 */
static int32_t initialize_speed_calculator(void * this, speed_calc_data_t * speed_calc)
{
  if(this==NULL)
    return -1;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  obj->_speed_struct = *speed_calc; 
  return RM_OK;
}

/** The function //TODO
 *  To be invoked by the kf_calculator_t::speed_calculator(&this_obj). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 * @return: //TODO
 */
static float speed_calculator(void * this)
{
  if(this==NULL)
    return 0.0f;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  //TODO
}

/** The function calculate the output of kalman filter
 *  To be invoked by the kf_calculator_t::filter(&this_obj). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 * @return: the filtered result
 */
static float filtered_value_calculator(void * this)
{
  if(this==NULL)
    return 0.0f;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  //TODO
  return obj->filtered_value;
}

/** The function retireves the filted value by the Kalman filter. 
 *  To be invoked by the kf_calculator_t.get_value(&this_obj). 
 * 
 * @author: Y.H. Liu
 * @param: this - pointer to an instance of the kf_calculator_t
 * @return: the filted value, a float number
 */
static float ret_value(void * this)
{
  if(this==NULL)
    return 0.0f;
  kf_calculator_t * obj = (kf_calculator_t *)this;
  
  return obj->filtered_value;
}
