/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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
/** @file comm_task.h
 *  @version 1.0
 *  @date Mar 2019
 *
 *  @brief Communication with host PC task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#ifndef __COMM_TASK_H__
#define __COMM_TASK_H__

#include "stm32f4xx_hal.h"
#include "data_fifo.h"

#define COMM_TASK_PERIOD (100) // 10Hz communication

#define PC_UART_TX_SIGNAL      ( 1 << 3 )
#define PC_UART_IDLE_SIGNAL    ( 1 << 4 )
#define PC_DMA_FULL_SIGNAL     ( 1 << 5 )

typedef struct {
	int16_t chassis_vel[4]; /**< Command velocity of chassis motors */
} chassis_states_t;

extern osThreadId taskCommHandle;

/** @brief Unpack task of host PC commands */
void comm_task(void const *argu);

/** @brief Task to send motor velocities to motor controller */
void i2c_msg_send_task(void const *argu);

/** @brief Initialization of communication submodule */
void communicate_param_init(void);

void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof);

#endif
