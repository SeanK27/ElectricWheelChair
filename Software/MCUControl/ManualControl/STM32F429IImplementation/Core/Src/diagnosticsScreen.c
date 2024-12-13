/*
 * diagnosticsScreen.c
 *
 *  Created on: Dec 9, 2024
 *      Author: seank
 */

#include "diagnosticsScreen.h"
#include "stdio.h"

void displayMovementBase() {
	LCD_Clear(0,LCD_COLOR_WHITE);
	//LCD_Draw_Circle_Fill(120, 80, 50, LCD_COLOR_BLACK);
	//LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
	LCD_Draw_Circle_NoFill(120, 95, 50, 3, LCD_COLOR_BLACK);
	LCD_Draw_Horizontal_Line(0, 155, 240, LCD_COLOR_BLACK);

//	LCD_DisplayChar(80, 50, 'J');
//	LCD_DisplayChar(95, 50, 'o');
//	LCD_DisplayChar(110, 50, 'y');
//	LCD_DisplayChar(123, 50, 's');
//	LCD_DisplayChar(130, 50, 't');
//	LCD_DisplayChar(142, 50, 'i');
//	LCD_DisplayChar(158, 50, 'c');
//	LCD_DisplayChar(168, 50, 'k');
}

void displayCurrentMove(joyPosTypeDef joyPos) {
	/*
	// Neutral
	if ((joyPos.xPos <= 515 && joyPos.xPos >= 510) || (joyPos.xPos <= 515 && joyPos.xPos >= 510)) {

		LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
		LCD_Draw_Circle_Fill(120, 80, 20, LCD_COLOR_BLACK);
	}

	// Forward
	if ((joyPos.xPos <= 515 && joyPos.xPos >= 510) || (joyPos.xPos <= 515 && joyPos.xPos >= 510)) {

		LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
		LCD_Draw_Circle_Fill(120, 80-47+20, 20, LCD_COLOR_BLACK);
	}

	// Backward
	if (Position == 2) {

		LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
		LCD_Draw_Circle_Fill(120, 80+47-20, 20, LCD_COLOR_BLACK);
	}

	// Left
	if (Position == 3) {

		LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
		LCD_Draw_Circle_Fill(120-47+20, 80, 20, LCD_COLOR_BLACK);
	}

	// Right
	if (Position == 4) {

		LCD_Draw_Circle_Fill(120, 80, 47, LCD_COLOR_WHITE);
		LCD_Draw_Circle_Fill(120+47-20, 80, 20, LCD_COLOR_BLACK);
	}
	*/

	uint32_t dispJoyPosY = mapp(joyPos.xPos, 0, 1024, 95+47-20, 95-47+20);

	uint32_t dispJoyPosX = mapp(joyPos.yPos, 0, 1024, 120-47+20, 120+47-20);


	LCD_Draw_Circle_Fill(120, 95, 47, LCD_COLOR_WHITE);
	LCD_Draw_Circle_Fill(dispJoyPosX, dispJoyPosY, 20, LCD_COLOR_BLACK);
}

void displayMoveLog(uint16_t moveLog) {

	LCD_SetTextColor(LCD_COLOR_BLACK);
	LCD_SetFont(&Font16x24);

	// Create a copy of moveLog and y-offset
	uint16_t tempLog = moveLog;
	uint16_t yOffset = 170;

	while (tempLog != 0) {

		// Extract the lowest 4 bits
		uint16_t currentMove = tempLog & 0xF;

		if (currentMove == FORWARD) {

			LCD_DisplayChar(100, yOffset, 'F');
			LCD_DisplayChar(115, yOffset, 'W');
			LCD_DisplayChar(130, yOffset, 'D');
			yOffset += 30;
		}
		if (currentMove == BACKWARD) {

			LCD_DisplayChar(100, yOffset, 'N');
			LCD_DisplayChar(115, yOffset, 'U');
			LCD_DisplayChar(130, yOffset, 'T');
			yOffset += 30;
		}
		if (currentMove == LEFT) {

			LCD_DisplayChar(100, yOffset, 'R');
			LCD_DisplayChar(115, yOffset, 'I');
			LCD_DisplayChar(130, yOffset, 'T');
			yOffset += 30;
		}
		if (currentMove == RIGHT) {

			LCD_DisplayChar(100, yOffset, 'L');
			LCD_DisplayChar(115, yOffset, 'F');
			LCD_DisplayChar(130, yOffset, 'T');
			yOffset += 30;
		}

		// Shift the tempLog to process the next 4 bits
		tempLog >>= 4;
	}
}

void displayRunTime(uint32_t * runtime_min, uint32_t * runtime_sec) {

	*runtime_min = uwTick/60000;

	*runtime_sec = (uwTick % 60000) / 1000;

	// Convert minutes and seconds to strings
	char minStr[3];
	char secStr[3];

	// Convert to strings with two digits
	sprintf(minStr, "%02d", (int) *runtime_min);
	sprintf(secStr, "%02d", (int) *runtime_sec);

	LCD_DisplayChar(80, 100, 'R');
	LCD_DisplayChar(95, 100, 'u');
	LCD_DisplayChar(110, 100, 'n');
	LCD_DisplayChar(123, 100, 't');
	LCD_DisplayChar(130, 100, 'i');
	LCD_DisplayChar(142, 100, 'm');
	LCD_DisplayChar(158, 100, 'e');
	LCD_DisplayChar(168, 98, ':');

	LCD_DisplayChar(90, 140, minStr[0]);
	LCD_DisplayChar(105, 140, minStr[1]);
	LCD_DisplayChar(115, 138, ':');
	LCD_DisplayChar(125, 140, secStr[0]);
	LCD_DisplayChar(140, 140, secStr[1]);
}











