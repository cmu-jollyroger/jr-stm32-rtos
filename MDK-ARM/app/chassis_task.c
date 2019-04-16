#include "chassis_task.h"

/* chassis task global parameter */
chassis_t chassis;

uint32_t chassis_time_last;
int chassis_time_ms;

/**
 * @brief Get encoder feedback on chassis wheel speeds
 */
void get_motor_feedback() {
	
}

/**
 * @brief Get TOF sensor readings
 */
void get_tof_feedback() {
	
}

void chassis_task(void const *argu) {
	chassis_time_ms = HAL_GetTick() - chassis_time_last;
	chassis_time_last = HAL_GetTick();
	
	while (1) {
		
	}
}

