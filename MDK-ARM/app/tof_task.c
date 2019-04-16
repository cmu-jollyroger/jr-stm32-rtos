#include "tof_task.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"

/* USER CODE BEGIN Header_tof_task */
/**
* @brief Function implementing the taskTOF thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_task */

void update_limit_sw(void)
{
	
}

void tof_task(void const * argu)
{
  /* USER CODE BEGIN tof_task */
	int status;
	uint8_t ToFSensor;
	static VL53L1_RangingMeasurementData_t RangingData;
	VL53L1_DEV Dev = &tof_sensors[0].dev;
	
	while (1) {
	}
	
  /* Infinite loop */
  while (1) {
		for (ToFSensor = 0; ToFSensor < NUM_TOFS; ToFSensor++) {
			Dev = &tof_sensors[ToFSensor].dev;
			
			status = VL53L1_StartMeasurement(Dev);
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					chassis.range_tof[ToFSensor] = RangingData.RangeMilliMeter;
					printf("%d,%d,%d,%d,%.2f,%.2f\r\n", ToFSensor, ToFSensor,RangingData.RangeStatus,RangingData.RangeMilliMeter,
									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		update_limit_sw();
	}
  /* USER CODE END tof_task */
}
