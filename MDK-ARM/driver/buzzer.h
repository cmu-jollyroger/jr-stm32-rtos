/****************************************************************************
 *  Copyright (C) 2019 Haowen Shi.
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
/** @file buzzer.h
 *  @version 1.0
 *  @date Apr 2019
 *
 *  @brief buzzer driver
 *
 *  @copyright Haowen Shi, Carnegie Mellon University.
 *
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "stm32xxx_hal.h"

void buzz_n_times_with_delay(uint16_t n, uint16_t delay);

#endif /* __BUZZER_H__ */