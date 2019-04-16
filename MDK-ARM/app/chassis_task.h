/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file chassis_task.h
 *  @version 1.0
 *  @date Apr 2019
 *
 *  @brief chassis control task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *  @copyright Haowen Shi, Carnegie Mellon University.
 *
 */

#include "jr_comm.h"

/**@brief  Chassis modes */
typedef enum
{
  CHASSIS_RELAX          = 0, /**< Chassis no power */
  CHASSIS_STOP           = 1, /**< Chassis is stopped/breaking */
  CHASSIS_MOVING         = 4, /**< Chassis is moving */
} chassis_mode_e;

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw; // 
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  chassis_mode_e  ctrl_mode;
  chassis_mode_e  last_ctrl_mode;

  float           gyro_angle;
  float           gyro_palstance;

  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
  
  int16_t         position_ref;
} chassis_t;

void chassis_task(void const *argu);

void chassis_param_init(void);
void power_limit_handler(void);

static void chassis_stop_handler(void);

static void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

extern chassis_t chassis;
