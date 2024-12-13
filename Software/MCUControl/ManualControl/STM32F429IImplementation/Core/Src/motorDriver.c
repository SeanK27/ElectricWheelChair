/*
 * motorDriver.c
 *
 *  Created on: Nov 30, 2024
 *      Author: seank
 */

#include "motorDriver.h"
#include "stdlib.h"

uint32_t mapp(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor_CalculatePower(uint16_t xPos, uint16_t yPos, motorPowTypeDef * motorPow) {

	uint16_t base = (uint16_t) mapp(xPos, 512, 1024, 0, 100);

	// Adjustment value that is applied to both motors
	int16_t adjust = (int16_t) (mapp(512 - yPos, 0, 1024, 0, 100)) * TURNFACTOR;

	// Apply adjustment
	int16_t rightMotor = base - adjust;
	int16_t leftMotor = base + adjust;

	// When joystick is fully back, send HIGH signal to brakePinLeft then set power to 0.
	if ((rightMotor < 0 && leftMotor < 0) || (xPos <= 515 && yPos <= 515)) {
	    rightMotor = 0;
	    leftMotor = 0;

	    // Store motor power
	    motorPow->leftPow = leftMotor;
	    motorPow->rightPow = rightMotor;
	}

	else {

		//////////////// Removing values >255 or <0 and setting them to 255 and 0 respectively ///////////////////////
		if (rightMotor < 0) {
			rightMotor = 0;
		}

		if (leftMotor < 0) {
			leftMotor = 0;
		}

		if (rightMotor > 100) {
			rightMotor = 100;
		}

		if (leftMotor > 100) {
			leftMotor = 100;
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////


		// Store motor power
		motorPow->leftPow = leftMotor;
		motorPow->rightPow = rightMotor;
	}
}

void motor_ApplyPower(uint16_t powerLeft, uint16_t powerRight) {

	// TODO: Disable Brakes Here

	TIM3->CCR1 = powerLeft;		// Set the left CCR register to change PWM
	TIM3->CCR3 = powerRight;	// Set the right CCR register to change PWM

}

void motor_BrakeAll() {

	// TODO: Enable both Brakes Here
}

void motor_noPower() {

	TIM3->CCR1 = 0;
	TIM3->CCR3 = 0;
}

void populateMoves(uint16_t powerLeft, uint16_t powerRight, uint16_t * moveLog) {

	// BWD
	if (powerLeft < 5 && powerRight < 5) {

		// Check if move is already displayed
		if ((*moveLog & 0xF) != BACKWARD) {

			*moveLog = (*moveLog << 4) | BACKWARD;
		}
	}

	else {

		// LFT
		if (powerLeft < powerRight) {

			// FWD
			if (powerRight - powerLeft < 15) {

				// Check if move is already displayed
				if ((*moveLog & 0xF) != FORWARD) {

				*moveLog = (*moveLog << 4) | FORWARD;
				}
			}

			// LFT
			else {

				// Check if move is already displayed
				if ((*moveLog & 0xF) != LEFT) {

					*moveLog = (*moveLog << 4) | LEFT;
				}
			}
		}

		// RIT
		else {

			// FWD
			if (powerLeft - powerRight < 15) {

				// Check if move is already displayed
				if ((*moveLog & 0xF) != FORWARD) {

					*moveLog = (*moveLog << 4) | FORWARD;
				}
			}

			// RIT
			else {

				// Check if move is already displayed
				if ((*moveLog & 0xF) != RIGHT) {

					*moveLog = (*moveLog << 4) | RIGHT;
				}
			}
		}
	}
}








