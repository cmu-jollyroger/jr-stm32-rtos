//
// Created by FionaLee on 4/15/19.
//

#ifndef __INFO_INTER_H__
#define __INFO_INTER_H__

#include "stm32f4xx_hal.h"

void write_uart_blocking(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);
void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt);
void get_infantry_info(void);
#endif //JR_STM32_RTOS_INFO_INTERACTIVE_H
