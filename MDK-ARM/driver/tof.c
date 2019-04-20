#include "tof.h"
#include "main.h"
#include "jr_status.h"

tof_sensor_t tof_sensors[NUM_TOFS];

/** @brief struct containing TOF XSHUT pin configurations */
struct _gpio {
	GPIO_TypeDef *port;
	uint16_t mask;
} xshut[NUM_TOFS] = {
	{TOF_XSHUT_0_GPIO_Port, TOF_XSHUT_0_Pin},
	{TOF_XSHUT_1_GPIO_Port, TOF_XSHUT_1_Pin},
	{TOF_XSHUT_2_GPIO_Port, TOF_XSHUT_2_Pin},
	{TOF_XSHUT_3_GPIO_Port, TOF_XSHUT_3_Pin},
	{TOF_XSHUT_4_GPIO_Port, TOF_XSHUT_4_Pin},
	{TOF_XSHUT_5_GPIO_Port, TOF_XSHUT_5_Pin},
};

void VL53L1_TOF_Config() {
	/* Configure each TOF */
	GPIO_InitTypeDef GPIO_InitStruct;
	for (int i = 0; i < NUM_TOFS; i++) {
		/*Configure GPIO pin : TOF_n_XSHUT_Pin */
		GPIO_InitStruct.Pin = xshut[i].mask;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(xshut[i].port, &GPIO_InitStruct);
		
		tof_sensors[i].port = xshut[i].port;
		tof_sensors[i].mask = xshut[i].mask;
		tof_sensors[i].valid = 0;
		tof_sensors[i].dev.I2cHandle = &hi2c1;
		tof_sensors[i].dev.I2cDevAddr = 0x29 << 1; // default address
	}
	
	printf("configured %d TOF sensors\r\n", NUM_TOFS);
}

/** @brief TOF initialization */
void VL53L1_TOF_Init() {
	VL53L1_Error err;
	VL53L1_DEV Dev = &tof_sensors[0].dev;
	
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	
	/* pull xshut to low for all sensors */
	for (int i = 0; i < NUM_TOFS; i++) {
		HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_RESET);
	}
	
	for (int i = 0; i < NUM_TOFS; i++) {
		HAL_GPIO_WritePin(xshut[i].port, xshut[i].mask, GPIO_PIN_SET);
		HAL_Delay(10);
		
		Dev = &tof_sensors[i].dev;
		Dev->I2cDevAddr = 0x29 << 1; // default address before configuring
		
		err = VL53L1_WaitDeviceBooted(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_WaitDeviceBooted(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_WaitDeviceBooted(%d)\r\n", i);
		}
		
		err = VL53L1_SetDeviceAddress(Dev, (0x29 + i + 1) << 1);
		Dev->I2cDevAddr = (0x29 + i + 1) << 1; //change address even in case of error to reduce cross-talk
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetDeviceAddress(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_SetDeviceAddress(%d)\r\n", i);
		}
		
		err = VL53L1_DataInit(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_DataInit(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_DataInit(%d)\r\n", i);
		}
		
		err = VL53L1_StaticInit(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_StaticInit(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_StaticInit(%d)\r\n", i);
		}
		
		err = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetDistanceMode(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_SetDistanceMode(%d)\r\n", i);
		}
		
		err = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_SetInterMeasurementPeriodMilliSeconds(%d) failed\r\n", i);
			goto on_error;
		} else {
			printf("[OK] VL53L1_SetInterMeasurementPeriodMilliSeconds(%d)\r\n", i);
		}
		
		err = VL53L1_StartMeasurement(Dev);
		if (err != VL53L1_ERROR_NONE) {
			printf("[error] VL53L1_StartMeasurement(%d) failed : %d\r\n", i, err);
			goto on_error;
		} else {
			printf("[OK] VL53L1_StartMeasurement(%d)\r\n", i);
		}
		
		printf("[OK] Initialized TOF #%d\r\n", i);
		
		uint8_t byteData;
		uint16_t wordData;
		
		VL53L1_RdByte(Dev, 0x010F, &byteData);
		printf("   - VL53L1X Model_ID: %02X\r\n", byteData);
		VL53L1_RdByte(Dev, 0x0110, &byteData);
		printf("   - VL53L1X Module_Type: %02X\r\n", byteData);
		VL53L1_RdWord(Dev, 0x010F, &wordData);
		printf("   - VL53L1X: %02X\r\n", wordData);
		tof_sensors[i].valid = 1;
		
		glb_status_tof[i] = 0;
		continue;
		
		/* Error handling */
on_error:
		glb_status_tof[i] = -1;
		continue;
	}
	
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}
