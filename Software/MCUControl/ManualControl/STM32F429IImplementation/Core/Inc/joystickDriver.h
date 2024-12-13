/*
 * joystickDriver.h
 *
 *  Created on: Dec 2, 2024
 *      Author: seank
 */

#ifndef INC_JOYSTICKDRIVER_H_
#define INC_JOYSTICKDRIVER_H_

#include "stm32f4xx_hal.h"

typedef struct {
	uint32_t xPos;
	uint32_t yPos;
} joyPosTypeDef;

void joystick_getCoords(joyPosTypeDef * joyPos, ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2);

#endif /* INC_JOYSTICKDRIVER_H_ */
