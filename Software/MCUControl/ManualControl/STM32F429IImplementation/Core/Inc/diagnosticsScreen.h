/*
 * diagnosticsScreen.h
 *
 *  Created on: Dec 9, 2024
 *      Author: seank
 */

#ifndef INC_DIAGNOSTICSSCREEN_H_
#define INC_DIAGNOSTICSSCREEN_H_

#include "LCD_Driver.h"
#include "joystickDriver.h"
#include "motorDriver.h"

void displayMovementBase();

void displayCurrentMove(joyPosTypeDef joyPos);

void displayMoveLog(uint16_t moveLog);

void displayRunTime(uint32_t * runtime_min, uint32_t * runtime_sec);

#endif /* INC_DIAGNOSTICSSCREEN_H_ */
