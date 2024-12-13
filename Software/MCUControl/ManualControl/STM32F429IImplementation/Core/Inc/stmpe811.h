/*
 * stmpe811.h
 *
 *  Created on: Oct 31, 2023
 *      Author: Tilen MAJERLE modified significantly by Xavion
 */

#ifndef INC_STMPE811_H_
#define INC_STMPE811_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define COMPILE_TOUCH  1
#define COMPILE_TOUCH_INTERRUPT_SUPPORT    1

#if (COMPILE_TOUCH_INTERRUPT_SUPPORT == 1 && COMPILE_TOUCH == 0)
#error "You cannot have touch interrupt support without compiling all touch functionality"
#endif // (COMPILE_TOUCH_INTERRUPT_SUPPORT == 1 && COMPILE_TOUCH == 0)

/* Private defines */
/* I2C address */
#define STMPE811_ADDRESS                0x82

/* STMPE811 Chip ID on reset */
#define STMPE811_CHIP_ID_VALUE          0x0811  //Chip ID

/* Registers */
#define STMPE811_CHIP_ID                0x00    //STMPE811 Device identification
#define STMPE811_ID_VER                 0x02    //STMPE811 Revision number; 0x01 for engineering sample; 0x03 for final silicon
#define STMPE811_SYS_CTRL1              0x03    //Reset control
#define STMPE811_SYS_CTRL2              0x04    //Clock control
#define STMPE811_SPI_CFG                0x08    //SPI interface configuration
#define STMPE811_INT_CTRL               0x09    //Interrupt control register
#define STMPE811_INT_EN                 0x0A    //Interrupt enable register
#define STMPE811_INT_STA                0x0B    //Interrupt status register
#define STMPE811_GPIO_EN                0x0C    //GPIO interrupt enable register
#define STMPE811_GPIO_INT_STA           0x0D    //GPIO interrupt status register
#define STMPE811_ADC_INT_EN             0x0E    //ADC interrupt enable register
#define STMPE811_ADC_INT_STA            0x0F    //ADC interface status register
#define STMPE811_GPIO_SET_PIN           0x10    //GPIO set pin register
#define STMPE811_GPIO_CLR_PIN           0x11    //GPIO clear pin register
#define STMPE811_MP_STA                 0x12    //GPIO monitor pin state register
#define STMPE811_GPIO_DIR               0x13    //GPIO direction register
#define STMPE811_GPIO_ED                0x14    //GPIO edge detect register
#define STMPE811_GPIO_RE                0x15    //GPIO rising edge register
#define STMPE811_GPIO_FE                0x16    //GPIO falling edge register
#define STMPE811_GPIO_AF                0x17    //alternate function register
#define STMPE811_ADC_CTRL1              0x20    //ADC control
#define STMPE811_ADC_CTRL2              0x21    //ADC control
#define STMPE811_ADC_CAPT               0x22    //To initiate ADC data acquisition
#define STMPE811_ADC_DATA_CHO           0x30    //ADC channel 0
#define STMPE811_ADC_DATA_CH1           0x32    //ADC channel 1
#define STMPE811_ADC_DATA_CH2           0x34    //ADC channel 2
#define STMPE811_ADC_DATA_CH3           0x36    //ADC channel 3
#define STMPE811_ADC_DATA_CH4           0x38    //ADC channel 4
#define STMPE811_ADC_DATA_CH5           0x3A    //ADC channel 5
#define STMPE811_ADC_DATA_CH6           0x3C    //ADC channel 6
#define STMPE811_ADC_DATA_CH7           0x3E    //ADC channel 7
#define STMPE811_TSC_CTRL               0x40    //4-wire touchscreen controller setup
#define STMPE811_TSC_CFG                0x41    //Touchscreen controller configuration
#define STMPE811_WDW_TR_X               0x42    //Window setup for top right X
#define STMPE811_WDW_TR_Y               0x44    //Window setup for top right Y
#define STMPE811_WDW_BL_X               0x46    //Window setup for bottom left X
#define STMPE811_WDW_BL_Y               0x48    //Window setup for bottom left Y
#define STMPE811_FIFO_TH                0x4A    //FIFO level to generate interrupt
#define STMPE811_FIFO_STA               0x4B    //Current status of FIFO
#define STMPE811_FIFO_SIZE              0x4C    //Current filled level of FIFO
#define STMPE811_TSC_DATA_X             0x4D    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_Y             0x4F    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_Z             0x51    //Data port for touchscreen controller data access
#define STMPE811_TSC_DATA_XYZ           0x52    //Data port for touchscreen controller data access
#define STMPE811_TSC_FRACTION_Z         0x56    //Touchscreen controller FRACTION_Z
#define STMPE811_TSC_DATA               0x57    //Data port for touchscreen controller data access
#define STMPE811_TSC_I_DRIVE            0x58    //Touchscreen controller drivel
#define STMPE811_TSC_SHIELD             0x59    //Touchscreen controller shield
#define STMPE811_TEMP_CTRL              0x60    //Temperature sensor setup
#define STMPE811_TEMP_DATA              0x61    //Temperature data access port
#define STMPE811_TEMP_TH                0x62    //Threshold for temperature controlled interrupt

#define STMPE811_I2C                    I2C3
#define STMPE811_I2C_CLOCK              100000

/* STM Related Macros, this techinically should be in a different fileset */
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA

typedef enum {
    STMPE811_Orientation_Portrait_1,  /*!< Portrait orientation mode 1 */
    STMPE811_Orientation_Portrait_2,  /*!< Portrait orientation mode 2 */
    STMPE811_Orientation_Landscape_1, /*!< Landscape orientation mode 1 */
    STMPE811_Orientation_Landscape_2, /*!< Landscape orientation mode 2 */
}STMPE811_Orientation_t;

/**
 * @brief  Enumeration for touch pressed or released
 */
typedef enum {
    STMPE811_State_Pressed,  /*!< Touch detected as pressed */
    STMPE811_State_Released, /*!< Touch detected as released/not pressed */
    STMPE811_State_Ok,       /*!< Result OK. Used on initialization */
    STMPE811_State_Error     /*!< Result error. Used on initialization */
} STMPE811_State_t;

/**
 * @brief  Main structure, which is passed into @ref TM_STMPE811_ReadTouch function
 */
typedef struct {
    uint16_t x;                            /*!< X coordinate on LCD for touch */
    uint16_t y;                            /*!< Y coordinate on LCD for touch */
    STMPE811_State_t pressed;           /*!< Pressed touch status */
    STMPE811_State_t last_pressed;      /*!< Last pressed touch status */
    STMPE811_Orientation_t orientation; /*!< Touch screen orientation to match your LCD orientation */
}STMPE811_t;

/* Backward compatibility */
typedef STMPE811_t STMPE811_TouchData;

/**
 * @brief  Checks if touch data is inside specific rectangle coordinates
 * @param  sd: Pointer to @ref TM_STMPE811_t to get data from
 * @param  xPos: Top-left X position of rectangle
 * @param  yPos: Top-left Y position of rectangle
 * @param  w: Rectangle width
 * @param  h: Rectangle height:
 * @retval Touch inside rectangle status:
 *            - 0: Touch is outside rectangle
 *            - > 0: Touch is inside rectangle
 * @note   Defined as macro for faster execution
 */
#define TM_STMPE811_TouchInRectangle(sd, xPos, yPos, w, h)  (((sd)->x >= (xPos)) && ((sd)->x < (xPos + w)) && ((sd)->y >= (yPos)) && ((sd)->y < (yPos + h)))

STMPE811_State_t STMPE811_ReadTouch(STMPE811_TouchData *data);
uint8_t STMPE811_Read(uint8_t reg);
void STMPE811_Write(uint8_t reg, uint8_t dataToWrite);
STMPE811_State_t STMPE811_Init(void);
bool isSTMPE811_Ready(void);
void STMPE811_DetermineTouchPosition(STMPE811_TouchData * data);

#if COMPILE_TOUCH_INTERRUPT_SUPPORT == 1

void STMPE811_DetermineTouchPosition(STMPE811_TouchData * data);
void enableInterruptSupportForTouch(void);

#endif


#endif /* INC_STMPE811_H_ */
