#include "buzzer.h"
#include "main.h"

void buzz_n_times_with_delay(uint16_t n, uint16_t delay) {
	for (int i = 0; i < n; i++) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
		if (i < n - 1) { HAL_Delay(delay); }
	}
}