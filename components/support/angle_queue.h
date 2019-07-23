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

#ifndef __ANGLE_QUEUE_H__
#define __ANGLE_QUEUE_H__
#include "sys.h"

#define MAX_QUEUE_SPACE 50
#define DELAY	1
#define __infinity 0x7F800000

struct angle_queue
{
	float angles[MAX_QUEUE_SPACE];
	int16_t len;
	int16_t head; 
	int16_t tail;
};

typedef struct angle_queue * angle_queue_p;

void  queue_init(angle_queue_p self);
void  enQueue(angle_queue_p self, float en_value);
float deQueue(angle_queue_p self);

#endif
