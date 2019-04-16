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

#ifndef __SYS_H__
#define __SYS_H__

#include "stm32f4xx_hal.h"
#include "usart.h"

/*************************chassis setting*******************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS                 76
/* the perimeter of wheel(mm) */
#define PERIMETER              478

/* wheel track distance(mm) */
#define WHEELTRACK             400
/* wheelbase distance(mm) */
#define WHEELBASE              400

/***********************system interface setting****************************/

/* uart relevant */
/**
  * @attention
  * close usart DMA receive interrupt, so need add 
  * uart_receive_handler() before HAL_UART_IROHandler() in uart interrupt function
*/
#define COMPUTER_HUART     (huart2) //connected to NanoPi M4

/* math relevant */
/* radian coefficient */
#define RADIAN_COEF        57.3f
/* circumference ratio */
#define PI                 3.142f

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#endif
