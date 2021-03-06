/****************************************************************************
 *  Copyright (C) 2019 Haowen Shi
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
/** @file tof.h
 *  @version 1.0
 *  @date Apr 2019
 *
 *  @brief the tof ranger finder interface
 *
 *  @copyright 2019 Haowen Shi. All rights reserved.
 *
 */

#ifndef __TOF_H__
#define __TOF_H__

#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"

#define NUM_TOFS (6)

/** @brief struct of all TOF sensors and their states */
typedef struct {
	VL53L1_Dev_t dev;
	uint8_t valid;
	GPIO_TypeDef *port;
	uint16_t mask;
} tof_sensor_t;

extern tof_sensor_t tof_sensors[NUM_TOFS];

void VL53L1_TOF_Config(void);
void VL53L1_TOF_Init(void);

#endif /*__TOF_H__*/
