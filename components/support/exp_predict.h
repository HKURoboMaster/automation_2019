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

#ifndef __EXP_PREDICT_H__
#define __EXP_PREDICT_H__

#define max(x,y) x>=y?x:y
#define min(x,y) x<=y?x:y

#define YAW_AUTO_AIMING     0u
#define PITCH_AUTO_AIMING   1u

void refresh(float input,  unsigned short index);
float get_predict(unsigned short index);
#endif
