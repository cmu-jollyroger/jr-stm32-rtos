#include "tof_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "jr_status.h"

/* USER CODE BEGIN Header_tof_task */
/**
* @brief Function implementing the taskTOF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_task */

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
	
	uint32_t tof_wake_time = osKernelSysTick();
	
  /* Infinite loop */
  while (1) {
		osDelayUntil(&tof_wake_time, 50);  // 5Hz
		
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
					chassis.range_tof[ToFSensor] = RangingData.RangeMilliMeter;
					//printf("%d,%d,%d,%d,%.2f,%.2f\r\n", ToFSensor, ToFSensor,RangingData.RangeStatus,RangingData.RangeMilliMeter,
					//				(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
			
			//taskEXIT_CRITICAL();
		}
		
		osThreadYield();
		
	}
  /* USER CODE END tof_task */
}
