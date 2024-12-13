/*
 * stmpe811.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Tilen MAJERLE modified significantly by Xavion
 */

#include "stmpe811.h"

#if COMPILE_TOUCH == 1

#define ONEBYTE  1
#define TWOBYTE  2

static void I2C3_MspInit(void);
static void I2C3_Init();
//static void stmpe811_MspInit(void);


/* Private functions */
uint8_t TM_STMPE811_Read(uint8_t reg);
uint16_t TM_STMPE811_ReadX(uint16_t x);
uint16_t TM_STMPE811_ReadY(uint16_t y);

void I2C3_Read(uint8_t address, uint8_t reg, uint8_t * rxData);
void I2C3_Write(uint16_t devAddr, uint8_t reg, uint8_t data);
void I2C3_MulitByteRead(uint8_t address, uint8_t reg, uint8_t * rxData, uint16_t numOfBytes);

static I2C_HandleTypeDef hI2C3;
static HAL_StatusTypeDef HAL_status;

#define DEFAULT_TESTING_TIMEOUT 250000

/* The below function was created by Tilen MAJERLE but modified by Xavion */
STMPE811_State_t STMPE811_Init(void)
{
    //uint8_t bytes[2];
	uint8_t mode;

    // Initalize any other GPIO neeeded
    //stmpe811_MspInit(); // Currently we will be just using the HAL GPIO Init fuction to initialize GPIOs..

    // Initialze I2C3 ports 
    I2C3_MspInit();
    /* Initialize I2C */
    I2C3_Init();

    /* Reset */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL1, 0x02);
    HAL_Delay(5);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL1, 0x00);
    HAL_Delay(2);

    /* Check for STMPE811 Connected */
    uint16_t dataRecieved;
    I2C3_MulitByteRead(STMPE811_ADDRESS, STMPE811_CHIP_ID, (uint8_t * )&dataRecieved, TWOBYTE); // Need to change
    // Flip bytes
    uint16_t chipID = (dataRecieved << 8);
    chipID |= ((dataRecieved & 0xFF00) >> 8);

    if (chipID != STMPE811_CHIP_ID_VALUE) {
    	return STMPE811_State_Error;
    }

    /* Reset */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL1, 0x02);
    HAL_Delay(5);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL1, 0x00);
    HAL_Delay(2);

    /* Get the current register value */
    mode = STMPE811_Read(STMPE811_SYS_CTRL2);
    mode &= ~(0x01);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL2, mode);
    mode = STMPE811_Read(STMPE811_SYS_CTRL2);
    mode &= ~(0x02);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_SYS_CTRL2, mode);

    /* Select Sample Time, bit number and ADC Reference */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_ADC_CTRL1, 0x49);

    /* Wait for 2 ms */
    HAL_Delay(2);

    /* Select the ADC clock speed: 3.25 MHz */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_ADC_CTRL2, 0x01);

    /* Select TSC pins in non default mode */
    mode = STMPE811_Read(STMPE811_GPIO_AF);
    mode |= 0x1E;
    I2C3_Write(STMPE811_ADDRESS, STMPE811_GPIO_AF, mode);

    /* Select 2 nF filter capacitor */
    /* Configuration:
    - Touch average control    : 4 samples
    - Touch delay time         : 500 uS
    - Panel driver setting time: 500 uS
    */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_TSC_CFG, 0x9A);

    /* Configure the Touch FIFO threshold: single point reading */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_TH, 0x01);

    /* Clear the FIFO memory content. */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x01);

    /* Put the FIFO back into operation mode  */
    I2C3_Write( STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x00);

    /* Set the range and accuracy pf the pressure measurement (Z) :
    - Fractional part :7
    - Whole part      :1
    */
    I2C3_Write( STMPE811_ADDRESS, STMPE811_TSC_FRACTION_Z, 0x01);

    /* Set the driving capability (limit) of the device for TSC pins: 50mA */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_TSC_I_DRIVE, 0x01);

    /* Touch screen control configuration (enable TSC):
    - No window tracking index
    - XYZ acquisition mode
    */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_TSC_CTRL, 0x03);

    /* Clear all the status pending bits if any */
    I2C3_Write(STMPE811_ADDRESS, STMPE811_INT_STA, 0xFF);

    /* Enable global interrupts */
    #if COMPILE_TOUCH_INTERRUPT_SUPPORT == 1

    enableInterruptSupportForTouch();

    mode = STMPE811_Read(STMPE811_INT_CTRL);
    mode |= 0x01;
    I2C3_Write(STMPE811_ADDRESS, STMPE811_INT_CTRL, mode);
    
    /* Enable touch interrupt */
    mode = STMPE811_Read(STMPE811_INT_EN);
    mode |= 0x01;
    I2C3_Write(STMPE811_ADDRESS, STMPE811_INT_EN, mode);
    
    #endif // COMPILE_TOUCH_INTERRUPT_SUPPORT
    
    /* Wait for 2 ms delay */
    HAL_Delay(200);

    return STMPE811_State_Ok;

}

uint8_t STMPE811_Read(uint8_t reg)
{
    // I2C Read
    uint8_t readData;
    I2C3_Read(STMPE811_ADDRESS, reg, &readData);

    return readData;
}

void STMPE811_Write(uint8_t reg, uint8_t dataToWrite)
{
    I2C3_Write(STMPE811_ADDRESS, reg, dataToWrite);
}

/* The below function was created by Tilen MAJERLE but modified by Xavion */

STMPE811_State_t STMPE811_ReadTouch(STMPE811_TouchData *structdata)  //TM Function
{
    uint8_t val;

    /* Save state */
    structdata->last_pressed = structdata->pressed;

    /* Read */
    val = STMPE811_Read(STMPE811_TSC_CTRL);
    if ((val & 0x80) == 0) {
        //Not pressed
        structdata->pressed = STMPE811_State_Released;

        //Reset Fifo
        I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x01);
        I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x00);

        return STMPE811_State_Released;
    }

    /* Clear all the status pending bits if any */
    //TM_I2C_Write(STMPE811_I2C, STMPE811_ADDRESS, STMPE811_INT_STA, 0xFF);

    //Pressed
    if (structdata->orientation == STMPE811_Orientation_Portrait_1) {
        structdata->x = 239 - TM_STMPE811_ReadX(structdata->x);
        structdata->y = 319 - TM_STMPE811_ReadY(structdata->y);
    } else if (structdata->orientation == STMPE811_Orientation_Portrait_2) {
        structdata->x = TM_STMPE811_ReadX(structdata->x);
        structdata->y = TM_STMPE811_ReadY(structdata->y);
    } else if (structdata->orientation == STMPE811_Orientation_Landscape_1) {
        structdata->y = TM_STMPE811_ReadX(structdata->y);
        structdata->x = 319 - TM_STMPE811_ReadY(structdata->x);
    } else if (structdata->orientation == STMPE811_Orientation_Landscape_2) {
        structdata->y = 239 - TM_STMPE811_ReadX(structdata->x);
        structdata->x = TM_STMPE811_ReadY(structdata->x);
    }

    //Reset Fifo
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x01);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x00);

    //Check for valid data
    if (structdata->orientation == STMPE811_Orientation_Portrait_1 || structdata->orientation == STMPE811_Orientation_Portrait_2) {
        //Portrait
        if (structdata->x > 0 && structdata->x < 239 && structdata->y > 0 && structdata->y < 319) {
            structdata->pressed = STMPE811_State_Pressed;
            return STMPE811_State_Pressed;
        }
    } else {
        //Landscape
        if (structdata->x > 0 && structdata->x < 319 && structdata->y > 0 && structdata->y < 239) {
            structdata->pressed = STMPE811_State_Pressed;
            return STMPE811_State_Pressed;
        }
    }

    structdata->pressed = STMPE811_State_Released;

    return STMPE811_State_Released;
}

void STMPE811_DetermineTouchPosition(STMPE811_TouchData * data)
{
    //Pressed
    if (data->orientation == STMPE811_Orientation_Portrait_1) {
        data->x = 239 - TM_STMPE811_ReadX(data->x);
        data->y = 319 - TM_STMPE811_ReadY(data->y);
    } else if (data->orientation == STMPE811_Orientation_Portrait_2) {
        data->x = TM_STMPE811_ReadX(data->x);
        data->y = TM_STMPE811_ReadY(data->y);
    } else if (data->orientation == STMPE811_Orientation_Landscape_1) {
        data->y = TM_STMPE811_ReadX(data->y);
        data->x = 319 - TM_STMPE811_ReadY(data->x);
    } else if (data->orientation == STMPE811_Orientation_Landscape_2) {
        data->y = 239 - TM_STMPE811_ReadX(data->x);
        data->x = TM_STMPE811_ReadY(data->x);
    }

    //Reset Fifo
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x01);
    I2C3_Write(STMPE811_ADDRESS, STMPE811_FIFO_STA, 0x00);
}

bool isSTMPE811_Ready(void)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_IsDeviceReady(&hI2C3, STMPE811_ADDRESS, 5, DEFAULT_TESTING_TIMEOUT);
    if(status != HAL_OK)
    {
        return false;
    }
    return true;
}

#if COMPILE_TOUCH_INTERRUPT_SUPPORT == 1

void enableInterruptSupportForTouch(void)
{
    // Initialze the GPIO and enable the interrupt
    // Interrupt is on interrupt Line PA15
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    NVIC_EnableIRQ(EXTI15_10_IRQn);

}

#endif 


//  ******************************** I2C Functions ********************************//
void verifyHAL_I2C_IS_OKAY(){
    if (HAL_status != HAL_OK)
    {
        while(1);
    }
}

static void I2C3_Init()
{

	__HAL_RCC_I2C3_CLK_ENABLE();
    // Configure I2C3
    hI2C3.Instance = STMPE811_I2C;
    hI2C3.Init.ClockSpeed = STMPE811_I2C_CLOCK;
    hI2C3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hI2C3.Init.OwnAddress1 = 0x00; // May be wrong
    hI2C3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hI2C3.Init.GeneralCallMode = I2C_NOSTRETCH_DISABLE;
    hI2C3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // Do we need to configutre I2C Mode? 

    // Initialize I2C3 interface
    HAL_StatusTypeDef status;
    status = HAL_I2C_Init(&hI2C3);
    if (status != HAL_OK)
    {
        for(;;); // Catch error
    }
    return;
}

// GPIO Initializations 
static void I2C3_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Enable Clocks
    // GPIOC
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // GPIOA 
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin : I2C3_SDA_Pin */
    GPIO_InitStruct.Pin = I2C3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : I2C3_SCL_Pin */
    GPIO_InitStruct.Pin = I2C3_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SCL_GPIO_Port, &GPIO_InitStruct);
    
}

// This function should only be used for single BYTE transfers 
void I2C3_Write(uint16_t devAddr, uint8_t reg, uint8_t data)
{
    uint8_t dataConversion = data; // data will be a raw hex value this is mainly for debugging...
    // Learning topic - Is this needed? Or can I just use &data in the function call? 
    HAL_status = HAL_I2C_Mem_Write(&hI2C3, devAddr, reg, I2C_MEMADD_SIZE_8BIT, &dataConversion, ONEBYTE, DEFAULT_TESTING_TIMEOUT);
    verifyHAL_I2C_IS_OKAY();
}

// This function should only be used for single BYTE transfers 
void I2C3_Read(uint8_t address, uint8_t reg, uint8_t * rxData)
{
    // Need to use MEM functions
    HAL_status = HAL_I2C_Mem_Read(&hI2C3, address, reg, I2C_MEMADD_SIZE_8BIT, rxData, ONEBYTE, DEFAULT_TESTING_TIMEOUT);
    verifyHAL_I2C_IS_OKAY();
}

// This function should be used for multiple byte reads from a reg
void I2C3_MulitByteRead(uint8_t address, uint8_t reg, uint8_t * rxData, uint16_t numOfBytes)
{
    HAL_I2C_Mem_Read(&hI2C3, address, reg, I2C_MEMADD_SIZE_8BIT, rxData, numOfBytes, DEFAULT_TESTING_TIMEOUT);
}

/* The below function was created by Tilen MAJERLE but modified by Xavion */
uint16_t TM_STMPE811_ReadX(uint16_t x) { // TM FUNCTION 
    uint8_t data[2];
    int16_t val, dx;
    data[1] = STMPE811_Read(STMPE811_TSC_DATA_X);
    data[0] = STMPE811_Read(STMPE811_TSC_DATA_X + 1);
    val = (data[1] << 8 | (data[0] & 0xFF));

    if (val <= 3000) {
        val = 3900 - val;
    } else {
        val = 3800 - val;
    }

    val /= 15;

    if (val > 239) {
        val = 239;
    } else if (val < 0) {
        val = 0;
    }

    dx = (val > x) ? (val - x) : (x - val);
    if (dx > 4) {
        return val;
    }
    return x;
}

/* The below function was created by Tilen MAJERLE but modified by Xavion */
uint16_t TM_STMPE811_ReadY(uint16_t y) { // TM FUNCTION 
    uint8_t data[2];
    int16_t val, dy;
    data[1] = STMPE811_Read(STMPE811_TSC_DATA_Y);
    data[0] = STMPE811_Read(STMPE811_TSC_DATA_Y + 1);
    val = (data[1] << 8 | (data[0] & 0xFF));

    val -= 360;
    val = val / 11;

    if (val <= 0) {
        val = 0;
    } else if (val >= 320) {
        val = 319;
    }

    dy = (val > y) ? (val - y) : (y - val);
    if (dy > 4) {
        return val;
    }
    return y;
}

#endif
