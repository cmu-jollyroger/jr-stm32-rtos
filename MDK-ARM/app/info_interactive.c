#include "info_interactive.h"
#include "comm_task.h"
#include "info_get_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "bsp_can.h"
#include "bsp_can.h"
#include "../bsp/bsp_uart.h"
#include "infantry_info.h"
#include "protocol.h"
#include "string.h"
#include "math.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "stdlib.h"

/**
  * @brief  Uart send data in blocking mode, 1000ms is the timeout value.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * @param  p_data: Pointer to data buffer
  * @param  size: Amount of data to be sent
  */
void write_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  HAL_UART_Transmit(huart, p_data, size, 100);
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt = dma_current_data_counter(dma_stream);
}

void get_infantry_info(void)
{

  float spd_p, spd_i, spd_d;
  float pos_p, pos_i, pos_d;

  /* chassis */
  pc_send_mesg.chassis_information.ctrl_mode      = chassis.ctrl_mode;
  pc_send_mesg.chassis_information.gyro_palstance = chassis.gyro_palstance;
  pc_send_mesg.chassis_information.gyro_angle     = chassis.gyro_angle;
//  pc_send_mesg.chassis_information.ecd_palstance  = palstance_deg;
//  pc_send_mesg.chassis_information.ecd_calc_angle = angle_deg;

//  pc_send_mesg.chassis_information.x_spd          = v_x_mm;
//  pc_send_mesg.chassis_information.y_spd          = v_y_mm;
//  pc_send_mesg.chassis_information.x_position     = position_x_mm; //the absolute x axis position of chassis
//  pc_send_mesg.chassis_information.y_position     = position_y_mm; //the absolute y axis position of chassis

  pc_send_mesg.chassis_information.tof_0          = 0;
  pc_send_mesg.chassis_information.tof_1          = 1;
  pc_send_mesg.chassis_information.tof_2          = 2;
  pc_send_mesg.chassis_information.tof_3          = 3;
  pc_send_mesg.chassis_information.tof_4          = 4;
  pc_send_mesg.chassis_information.tof_5          = 5;

  getSpeedPID(&spd_p, &spd_i, &spd_d, 0);
  getPosPID(&pos_p, &pos_i, &pos_d, 0);

  pc_send_mesg.chassis_information.spd_p          = spd_p;
  pc_send_mesg.chassis_information.spd_i          = spd_i;
  pc_send_mesg.chassis_information.spd_d          = spd_d;
  pc_send_mesg.chassis_information.pos_p          = pos_p;
  pc_send_mesg.chassis_information.pos_i          = pos_i;
  pc_send_mesg.chassis_information.pos_d          = pos_d;

  /* infantry error */
  pc_send_mesg.bottom_error_data.err_sta = DEVICE_NORMAL;
  for (uint8_t i = CHASSIS_GYRO_OFFLINE; i < ERROR_LIST_LENGTH; i++)
  {
    if (g_err.list[i].enable)
    {
      if (g_err.list[i].err_exist)
      {
        pc_send_mesg.bottom_error_data.err_sta = ERROR_EXIST;
        pc_send_mesg.bottom_error_data.err[i]  = ERROR_EXIST;
      }
      else
        pc_send_mesg.bottom_error_data.err[i]  = DEVICE_NORMAL;
    }
    else
      pc_send_mesg.bottom_error_data.err[i] = UNKNOWN_STATE;
  }

  /* structure config */
  pc_send_mesg.structure_config_data.chassis_config = glb_struct.chassis_config;
}