/*
 * motorDriver.h
 *
 *  Created on: Nov 30, 2024
 *      Author: seank
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

// Parameter that affects turning sensitivity
#define TURNFACTOR 1

// Movement Masks
#define FORWARD  0x0001 // 0b0001
#define BACKWARD 0x0002 // 0b0010
#define LEFT     0x0004 // 0b0100
#define RIGHT    0x0008 // 0b1000

typedef struct {
	uint16_t leftPow;
	uint16_t rightPow;
} motorPowTypeDef;

uint32_t mapp(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);

void motor_CalculatePower(uint16_t xPos, uint16_t yPos, motorPowTypeDef * motorPow);

void motor_ApplyPower(uint16_t speedLeft, uint16_t speedRight);

void motor_BrakeAll();

void motor_noPower();

void populateMoves(uint16_t powerLeft, uint16_t powerRight, uint16_t * moveLog);

#endif /* INC_MOTORDRIVER_H_ */
