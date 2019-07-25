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

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__
// #define HERO_ROBOT

#ifdef SHOOT_TASK_H_GLOBAL
  #define SHOOT_TASK_H_EXTERN 
#else
  #define SHOOT_TASK_H_EXTERN extern
#endif

#define CONTIN_BULLET_NUM 20
#define CONTINUE_SHOOT_TH 200 //continue-shoot threashold: 300 counts

void shoot_task(void const * argument);

#endif // __SHOOT_TASK_H__
