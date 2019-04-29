#include <math.h>
#include <stdlib.h>
#include "tof_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "jr_status.h"
#include "simpleKalmanFilter.h"

/* USER CODE BEGIN Header_tof_task */
/**
* @brief Function implementing the taskTOF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_task */

/* Detect when TOF's are not responding and reset them */
int16_t tof_prev_readings[NUM_TOFS];
int16_t tof_dup_count[NUM_TOFS];

void update_limit_sw(void)
{
	
	// If button on PA1 is pressed (button circuit is active low)
  if (HAL_GPIO_ReadPin(LIMIT_SW_L_GPIO_Port, LIMIT_SW_L_Pin) == GPIO_PIN_SET) {
		chassis.limit_sw_l = 0;
  } else {
		chassis.limit_sw_l = 1;
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	if (HAL_GPIO_ReadPin(LIMIT_SW_R_GPIO_Port, LIMIT_SW_R_Pin) == GPIO_PIN_SET) {
		chassis.limit_sw_r = 0;
	} else {
		chassis.limit_sw_r = 1;
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void tof_task(void const * argu)
{
  /* USER CODE BEGIN tof_task */
	int status;
	uint8_t ToFSensor;
	static VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_DEV Dev = &tof_sensors[0].dev;
	
	taskENTER_CRITICAL();
	
	uint32_t tof_wake_time = osKernelSysTick();
	
  SimpleKalmanFilterInit();
	srand(osKernelSysTick());
	
	taskEXIT_CRITICAL();
	
	for (ToFSensor = 0; ToFSensor < NUM_TOFS; ToFSensor++) {
		tof_prev_readings[ToFSensor] = -1;
		tof_dup_count[ToFSensor] = 0;
	}

  /* Infinite loop */
  while (1) {
		osDelayUntil(&tof_wake_time, 10);  // 100Hz (less than this)
		
		update_limit_sw();
		//continue;
		for (ToFSensor = 0; ToFSensor < NUM_TOFS; ToFSensor++) {
			if (glb_status_tof[ToFSensor] != 0) {
				//printf("skipping tof %d\r\n", ToFSensor);
				continue;
			}
			Dev = &tof_sensors[ToFSensor].dev;
			
			//taskENTER_CRITICAL();
			
			//status = VL53L1_StartMeasurement(Dev);
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
          float real_value = RangingData.RangeMilliMeter;
					float rand_num = (rand() - (float) RAND_MAX / 2.f) / (float)RAND_MAX;
          float measured_value = real_value + rand_num;
          float estimated_value = updateEstimate(measured_value, ToFSensor);
          chassis.range_tof[ToFSensor] = estimated_value;
//					chassis.range_tof[ToFSensor] = RangingData.RangeMilliMeter;
					
					/* Test if TOF has the same value for a period of time */
					if (tof_prev_readings[ToFSensor] == RangingData.RangeMilliMeter) {
						tof_dup_count[ToFSensor] ++;
					} else {
						tof_dup_count[ToFSensor] = 0;
					}
					tof_prev_readings[ToFSensor] = RangingData.RangeMilliMeter;

					//printf("%d,%d,%d,%d,%.2f,%.2f\r\n", ToFSensor, ToFSensor,RangingData.RangeStatus,RangingData.RangeMilliMeter,
					//				(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				} else {
					/* Reset if one TOF fails */
					goto reset_tof;
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			} else {
				tof_dup_count[ToFSensor] ++;
			}
			
			if (tof_dup_count[ToFSensor] > TOF_DUP_MAX_BEFORE_RESET) {
				/* Reset if TOF outputs same data for a long time */
				goto reset_tof;
			}
			
			//taskEXIT_CRITICAL();
		}
		
		osThreadYield();
		
		continue;
		reset_tof:
		taskENTER_CRITICAL();
		VL53L1_TOF_Init();
		taskEXIT_CRITICAL();
		
	}
  /* USER CODE END tof_task */
}
