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
#include "exp_predict.h"
#include "sys.h"
#include <math.h>

static float t[3][2];
static float predict[2];
static float time[2];

//TODO: This exp prediction cannot deal with the fluctuations
//TODO: Consider other regression methods
//TODO: The regression should handle a non-linear relationship with high variance
//TODO: And should be computational efficient

/**Added by Y.H. Liu
 * @Jul 22, 2019: Define the function
 * 
 * The function refresh the prediction basing on the current input and the previous two values
 * Param: 
 *     input: a float value, the currently received value from the board
 *     index: either YAW_AUTO_AIMING or PITCH_AUTO_AIMING
 */
void refresh(float input,  unsigned short index)
{
  if(index != YAW_AUTO_AIMING && index != PITCH_AUTO_AIMING)
    return;
  //update the array
  t[0][index] = t[1][index];
  t[1][index] = t[2][index];
  t[2][index] = input;
  //refresh the prediction
  float diff = t[2][index] - t[1][index];
  float root = max(4*diff+1, 0);
  root = sqrtf(1-4*root)/2;
  float a1 = 0.5f + root; a1 = min(a1, 1.25f);
  float a2 = 0.5f - root; a2 = max(a2, 0);
  float err_pred1 = fabsf(a1*a1 + t[0][index] - t[2][index]);
  float err_pred2 = fabsf(a2*a2 + t[0][index] - t[2][index]);
  if(err_pred1<err_pred2)
    predict[index] = a1*a1*a1 + t[0][index];
  else
    predict[index] = a2*a2*a2 + t[0][index];
  //refresh the time
  time[index] = get_time_ms_us();
}

/**Added by Y.H. Liu
 * @Jul 22, 2019: Define the function
 * 
 * The function returned the smoothed result of prediction
 * Param: 
 *     index: either YAW_AUTO_AIMING or PITCH_AUTO_AIMING
 */
float get_predict(unsigned short index)
{
  if(index!=YAW_AUTO_AIMING && index!=PITCH_AUTO_AIMING)
    return 0.0f;
  float delta = predict[index] - t[2][index];
  delta *= (get_time_ms_us() - time[index])/30.0f;
  return t[2][index] + delta;
}
