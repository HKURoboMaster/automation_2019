/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifndef __REFEREE_SYSTEM_H__
#define __REFEREE_SYSTEM_H__

#ifdef REFEREE_SYSTEM_H_GLOBAL
  #define REFEREE_SYSTEM_H_EXTERN 
#else
  #define REFEREE_SYSTEM_H_EXTERN extern
#endif

#include "sys.h"

typedef void (*ref_send_handler_t)(uint8_t* buf, uint16_t len);

#define REF_PROTOCOL_HEADER                 0xA5
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_CMD_MAX_NUM            20

#define REF_USER_TO_SERVER_MAX_DATA_LEN     64
#define REF_SERVER_TO_USER_MAX_DATA_LEN     32

#pragma pack(push,1)

typedef struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

#pragma pack(pop)

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  fifo_s_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

/**Defined by Y.H. Liu
 * @Jun 16, 2019: declare the structs for referee system data
 * 
 * Each refered to one type of data following HAEDER & CmdID
 * Distinguished by their CmdID
 */
//for bullet type (ID=0x0003)
#define BULLET_17MM 1u
#define BULLET_42MM 2u

#pragma pack(push,2)
//ID=0x0001
typedef struct
{
   uint8_t game_type : 4;
   uint8_t game_progress : 4;
   uint16_t stage_remain_time;
} ext_game_state_t; 
//Id = 0x0001

typedef struct
{
   uint8_t winner; 
} ext_game_result_t;
//Id = 0x0002

typedef struct 
{
   uint16_t red_1_HP; 
   uint16_t red_2_HP; 
   uint16_t red_3_HP; 
   uint16_t red_4_HP; 
   uint16_t red_5_HP; 
   uint16_t red_7_HP; 
   uint16_t red_base_HP; 
   uint16_t blue_1_HP; 
   uint16_t blue_2_HP; 
   uint16_t blue_3_HP; 
   uint16_t blue_4_HP; 
   uint16_t blue_5_HP; 
   uint16_t blue_7_HP; 
   uint16_t blue_base_HP; 
} ext_game_robot_hp_t; 
//Id = 0x0003

typedef struct 
{
   uint32_t event_type; 
} ext_event_data_t; 
//Id = 0x0101

typedef struct 
{
   uint8_t supply_projectile_id;
   uint8_t supply_robot_id;
   uint8_t supply_projectile_step;
   uint8_t supply_projectile_num;
 } ext_supply_projectile_action_t;
//Id = 0x0102

typedef struct
{
   uint8_t supply_projectile_id;
   uint8_t supply_robot_id;
   uint8_t supply_num;
 } ext_supply_projectile_booking_t; 
//Id = 0x0103

typedef struct
{
   uint8_t level;
   uint8_t id;
}ext_referee_warning_t;

typedef struct 
{
   uint8_t robot_id;
   uint8_t robot_level;
   uint16_t remain_HP;
   uint16_t max_HP;
   uint16_t shooter_heat0_cooling_rate;
   uint16_t shooter_heat0_cooling_limit;
   uint16_t shooter_heat1_cooling_rate;
   uint16_t shooter_heat1_cooling_limit;
   uint8_t mains_power_gimbal_output : 1;
   uint8_t mains_power_chassis_output : 1;
   uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t; 
//Id = 0x0201

typedef struct 
{
   uint16_t chassis_volt;
   uint16_t chassis_current;
   float chassis_power;
   uint16_t chassis_power_buffer;
   uint16_t shooter_heat0;
   uint16_t shooter_heat1;  
} ext_power_heat_data_t; 
//Id = 0x0202

typedef struct 
{ 
  float x; 
  float y; 
  float z; 
  float yaw; 
} ext_game_robot_pos_t; 
//Id = 0x203

typedef struct 
{
   uint8_t power_rune_buff; 
}ext_buff_musk_t; 
//Id = 0x0204

typedef struct 
{ 
   uint8_t energy_point;
   uint8_t attack_time;
} aerial_robot_energy_t; 
//Id = 0x0205

typedef struct 
{
   uint8_t armor_id : 4;
   uint8_t hurt_type : 4;
} ext_robot_hurt_t; 
//Id = 0x0206

typedef struct 
{
   uint8_t bullet_type;
   uint8_t bullet_freq;
   float bullet_speed;  
} ext_shoot_data_t; 
//Id = 0x0207

//Self-designed data
typedef struct 
{
   uint16_t data_cmd_id;
   uint16_t send_ID; 
   uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 
//Id = 0x0301 

// When data_cmd_id = 0xD180: data transferred from client and robots
typedef struct {
  float data1;
  float data2;
  float data3; 
  uint8_t masks; 
} client_custom_data_t;

// When data_cmd_id = 0x0200~0x02FF: data transferred from client and robots
typedef struct 
{
   uint8_t * data; 
} robot_interactive_data_t; 

#pragma pack(pop)

typedef enum
{
  gameState       =0x0001u, 
  gameResult      =0x0002u, 
  robotSurvivors  =0x0003u,
  gameEvent       =0x0101u, 
  supplierInfo    =0x0102u, 
  bulletSupply    =0x0103u, 
  refereeWarning  =0x0104u, 
  robotState      =0x0201u, 
  powerHeat       =0x0202u, 
  robotPos        =0x0203u, 
  buffMusk        =0x0204u, 
  aerialEnengy    =0x0205u, 
  robotHurt       =0x0206u, 
  shootData       =0x0207u,
  interactiveHead =0x0301u, 
}cmdID_t;

typedef struct
{
  ext_game_state_t                game_state;
  ext_game_result_t               game_result;
  ext_game_robot_hp_t             game_robot_survivors;
  ext_event_data_t                event_data;
  ext_supply_projectile_action_t  supply_action;
  ext_supply_projectile_booking_t supply_booking;
  ext_referee_warning_t           referee_warning;
  ext_game_robot_state_t          game_robot_state;
  ext_power_heat_data_t           power_heat_data;
  ext_game_robot_pos_t            game_robot_pos;
  ext_buff_musk_t                 buff_musk;
  aerial_robot_energy_t           aerial_energy;
  ext_robot_hurt_t                robot_hurt;
  ext_shoot_data_t                shoot_data;
}referee_data_t;


void referee_param_init(void);
void referee_unpack_fifo_data(void);
uint32_t referee_uart_rx_data_handle(uint8_t *data, uint32_t len);
uint32_t referee_send_data_register(ref_send_handler_t send_t);
void referee_protocol_tansmit(uint16_t cmd_id, void* p_buf, uint16_t len);
	
uint8_t     ref_get_crc8(uint8_t *p_msg, uint32_t len, uint8_t crc8);
uint32_t    ref_verify_crc8(uint8_t *p_msg, uint32_t len);
void        ref_append_crc8(uint8_t *p_msg, uint32_t len);
uint16_t    ref_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
uint32_t    ref_verify_crc16(uint8_t *p_msg, uint16_t len);
void        ref_append_crc16(uint8_t* p_msg, uint32_t len);
/*------ By Y.H. Liu ------*/
ext_power_heat_data_t * get_heat_power(void);
ext_game_robot_state_t * get_robot_state(void);
ext_event_data_t * get_rfid_data(void);
#endif // __REFEREE_SYSTEM_H__
