/*
 * joystickDriver.c
 *
 *  Created on: Dec 2, 2024
 *      Author: seank
 */

#include "joystickDriver.h"


void joystick_getCoords(joyPosTypeDef * joyPos, ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2) {

	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	joyPos->xPos = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	HAL_ADC_Start(&hadc2);

	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);

	joyPos->yPos = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc2);
}
