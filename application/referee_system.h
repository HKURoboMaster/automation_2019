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
//for buff_state (ID=0x0007)
#define BLOOD_RECOVERY        (1 << 0u)
#define ENGINEER_RECOVERING   (1 << 1u)
#define CURING_CARD           (1 << 2u)
#define RESOURCE_PORTECTION   (1 << 3u)
#define B_BUFF_INVOKED_US     (1 << 4u)
#define B_BUFF_INVOKED_ENEMY  (1 << 5u)
#define S_BUFF_INVOKED_US     (1 << 6u)
#define S_BUFF_INVOKED_ENEMY  (1 << 7u)
#define FASTENED_COOLING      (1 << 8u)
#define CASTLE_PROTECTION     (1 << 9u)
#define FULL_DEFENSE          (1 << 10u)
#define DEFENSE_NO_SENTRY     (1 << 11u)
#define DEFENSE_WITH_SENTRY   (1 << 12u)
//ID=0x0001
  typedef __packed struct {
    uint16_t   stageRemianTime; 
    uint8_t    gameProgress;
    uint8_t    robotLevel;
    uint16_t   remainHP;
    uint16_t   maxHP;
  }extGameRobotState_t;
//ID=0x0002
  typedef __packed struct {
    uint8_t   armorType  : 4;
    uint8_t   hurtType   : 4;
  } extRobotHurt_t;
//ID=0x0003
  typedef __packed struct {
    uint8_t 	bulletType;
    uint8_t 	bulletFreq;
    float  	bulletSpeed;
  }extShootData_t;
//ID=0x0004
  typedef __packed struct {
    float     chassisVolt;
    float     chassisCurrent;
    float     chassisPower;
    float     chassisPowerBuffer;
    uint16_t  shooterHeat0;
    uint16_t  shooterHeat1;
  }extPowerHeatData_t;
//ID=0x0005
  typedef __packed struct {
    uint8_t   cardType;
    uint8_t   cardIdx;
  }extRfidDetect_t;
//ID=0x0006
  typedef __packed struct {
    uint8_t  winner;
  }extGameResult_t;
//ID=0x0007
  typedef __packed struct {
    uint16_t   buffMusk;
  } extBuffMusk_t;
//ID=0x0008
  typedef __packed struct {
    float    x;
    float    y;
    float    z;
    float    yaw;
  }extGameRobotPos_t;
//ID=0x0100
  //self-defined info

typedef struct 
{
  extGameRobotState_t gameRobotState;
  extRobotHurt_t      robotHurt;
  extShootData_t      shootData;
  extPowerHeatData_t  powerHeatData;
  extRfidDetect_t     rfidDetect;
  extGameResult_t     result;
  extBuffMusk_t       buffMusk;
  extGameRobotPos_t   robotPos;
} referee_data_t;

typedef enum
{
  gameRobotState=0, robotHurt, shootData, powerHeatData, rfidDetect, result, buffMusk, robotPos, 
  selfDefined=0x0100, icra_upstream, icra_downstream
}cmdID_t;

void referee_param_init(void);
void referee_unpack_fifo_data(void);
uint32_t referee_uart_rx_data_handle(uint8_t *data, uint32_t len);
uint32_t referee_send_data_register(ref_send_handler_t send_t);
void referee_protocol_tansmit(uint16_t cmd_id, void* p_buf, uint16_t len);
	
uint8_t     ref_get_crc8(uint8_t *p_msg, uint32_t len, uint8_t crc8) ;
uint32_t    ref_verify_crc8(uint8_t *p_msg, uint32_t len);
void        ref_append_crc8(uint8_t *p_msg, uint32_t len);
uint16_t    ref_get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
uint32_t    ref_verify_crc16(uint8_t *p_msg, uint16_t len);
void        ref_append_crc16(uint8_t* p_msg, uint32_t len) ;
/*------ By Y.H. Liu ------*/
extPowerHeatData_t * get_heat_power(void);
#endif // __REFEREE_SYSTEM_H__
