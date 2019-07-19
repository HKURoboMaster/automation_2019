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
#include "angle_queue.h"

/**Added by Y.H. Liu
 * @Jul 17, 2019: Define the functions
 * 
 * Implement the queue data structures
 */
void queue_init(angle_queue_p self)
{
	self->head = 0;
	self->tail = 0;
	self->len = 0;
	for(int i=0; i<MAX_QUEUE_SPACE; i++)
	{
		self->angles[i] = 0.0f;
	}
}
void enQueue(angle_queue_p self, float en_value)
{
	if(self->len >= MAX_QUEUE_SPACE)
		return; // queue is full, unable to enQ
	self->angles[self->tail] = en_value;
	self->tail++;
	if(self->tail >= MAX_QUEUE_SPACE)
		self->tail %= MAX_QUEUE_SPACE;
	self->len++;
}
float deQueue(angle_queue_p self)
{
	float ret = 0.0f;
	if(self->len <= 0)
	{
		self->len = 0; 
		int * ret_p = (int *) &ret;
		*ret_p |= __infinity;
		return ret; // queue is empty
	}
	self->len--;
	ret = self->angles[self->head];
	self->head++;
	self->head %= MAX_QUEUE_SPACE;
	return ret;
}
