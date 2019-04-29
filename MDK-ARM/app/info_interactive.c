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
/** @file info_interactive.c
 *  @version 1.0
 *  @date Apr 15 2019
 *
 *  @brief Hardware peripheral information
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "info_interactive.h"
#include "comm_task.h"
//#include "info_get_task.h"
//#include "detect_task.h"
#include "chassis_task.h"
#include "bsp_uart.h"
//#include "infantry_info.h"
#include "protocol.h"
#include "string.h"
#include "math.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "stdlib.h"

extern TaskHandle_t taskCommHandle;

/**
  * @brief  Uart send data in non blocking mode
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be sent
  */
void write_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit_DMA(huart, p_data, size);
}

void uart_write_completed_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    //osSignalSet(unpack_task_t, PC_UART_WRITE_SIGNAL);
  }
}

/**
  * @brief  Uart send data in blocking mode, 1000ms is the timeout value.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be sent
  */
void write_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit(huart, p_data, size, 1000);
}

/**
  * @brief  Receives an amount of data in non blocking mode. 
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be received
  */
void read_uart_noblocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Receive_DMA(huart, p_data, size);
}

void uart_read_completed_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    //osSignalSet(unpack_task_t, PC_UART_READ_SIGNAL);
  }
}

/**
  * @brief  Receives an amount of data in blocking mode. 
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be received
  */
extern TaskHandle_t judge_unpack_task_t;
extern TaskHandle_t taskCommHandle;
void read_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Receive(huart, p_data, size, 1000);
}

void uart_idle_interrupt_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
		// TODO: error detection and recovery
    //err_detector_hook(PC_SYS_OFFLINE);
    osSignalSet(taskCommHandle, PC_UART_IDLE_SIGNAL);
  }
}

void uart_dma_full_signal(UART_HandleTypeDef *huart)
{
  if (huart == &COMPUTER_HUART)
  {
    /* remove DMA buffer full interrupt handler */
    //osSignalSet(taskCommHandle, PC_DMA_FULL_SIGNAL);
  }
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt = dma_current_data_counter(dma_stream);
}

void update_chassis_info(void)
{
	pc_send_mesg.chassis_information.ctrl_mode = chassis.ctrl_mode;
  pc_send_mesg.chassis_information.gyro_palstance = 1.f; /**< chassis palstance(degree/s) from gyroscope */
  pc_send_mesg.chassis_information.gyro_angle = 2.f;     /**< chassis angle(degree) relative to ground from gyroscope */
  pc_send_mesg.chassis_information.ecd_palstance = 3.f;  /**< chassis palstance(degree/s) from chassis motor encoder calculated */
  pc_send_mesg.chassis_information.ecd_calc_angle = 4.f; /**< chassis angle(degree) relative to ground from chassis motor encoder calculated */
  pc_send_mesg.chassis_information.x_spd = 5;            /**< TODO: use encoder driver */
  pc_send_mesg.chassis_information.y_spd = 6;            /**< TODO: use encoder driver */
  pc_send_mesg.chassis_information.x_position = 7;       /**< chassis x-axis position(mm) relative to the starting point */
  pc_send_mesg.chassis_information.y_position = 8;       /**< chassis y-axis position(mm) relative to the starting point */
	pc_send_mesg.chassis_information.tof_0 = chassis.range_tof[0];
	pc_send_mesg.chassis_information.tof_1 = chassis.range_tof[1];
	pc_send_mesg.chassis_information.tof_2 = chassis.range_tof[2];
	pc_send_mesg.chassis_information.tof_3 = chassis.range_tof[3];
	pc_send_mesg.chassis_information.tof_4 = chassis.range_tof[4];
	pc_send_mesg.chassis_information.tof_5 = chassis.range_tof[5];
	pc_send_mesg.chassis_information.sw_l = chassis.limit_sw_l;    /**< left limit switch */
	pc_send_mesg.chassis_information.sw_r = chassis.limit_sw_r;    /**< right limit switch */
	pc_send_mesg.chassis_information.enc_exec_done = chassis.enc_exec_done;  /**< encoder execution done, 0 if still in progress */
	
}
