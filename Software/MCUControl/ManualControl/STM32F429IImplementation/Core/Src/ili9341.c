/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ili9341.h"

static void SPI_MspInit(SPI_HandleTypeDef *hspi);
static void SPI_Error(void);


static SPI_HandleTypeDef SpiHandle;
static uint8_t Is_LCD_IO_Initialized = 0;
uint32_t SpiTimeout = SPI_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9341_Init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  LCD_IO_Init();

  /* Configure LCD */
  ili9341_Write_Reg(0xCA);
  ili9341_Send_Data(0xC3);				//param 1
  ili9341_Send_Data(0x08);				//param 2
  ili9341_Send_Data(0x50);				//param 3
  ili9341_Write_Reg(LCD_POWERB); //CF
  ili9341_Send_Data(0x00);				//param 1
  ili9341_Send_Data(0xC1);				//param 2
  ili9341_Send_Data(0x30);				//param 3
  ili9341_Write_Reg(LCD_POWER_SEQ); //ED
  ili9341_Send_Data(0x64);
  ili9341_Send_Data(0x03);
  ili9341_Send_Data(0x12);
  ili9341_Send_Data(0x81);
  ili9341_Write_Reg(LCD_DTCA);
  ili9341_Send_Data(0x85);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x78);
  ili9341_Write_Reg(LCD_POWERA);
  ili9341_Send_Data(0x39);
  ili9341_Send_Data(0x2C);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x34);
  ili9341_Send_Data(0x02);
  ili9341_Write_Reg(LCD_PRC);
  ili9341_Send_Data(0x20);
  ili9341_Write_Reg(LCD_DTCB);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x00);
  ili9341_Write_Reg(LCD_FRMCTR1);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x1B);
  ili9341_Write_Reg(LCD_DFC);
  ili9341_Send_Data(0x0A);
  ili9341_Send_Data(0xA2);
  ili9341_Write_Reg(LCD_POWER1);
  ili9341_Send_Data(0x10);
  ili9341_Write_Reg(LCD_POWER2);
  ili9341_Send_Data(0x10);
  ili9341_Write_Reg(LCD_VCOM1);
  ili9341_Send_Data(0x45);
  ili9341_Send_Data(0x15);
  ili9341_Write_Reg(LCD_VCOM2);
  ili9341_Send_Data(0x90);
  ili9341_Write_Reg(LCD_MAC);
  ili9341_Send_Data(0xC8);
  ili9341_Write_Reg(LCD_3GAMMA_EN);
  ili9341_Send_Data(0x00);
  ili9341_Write_Reg(LCD_RGB_INTERFACE);
  ili9341_Send_Data(0xC2);
  ili9341_Write_Reg(LCD_DFC);
  ili9341_Send_Data(0x0A);
  ili9341_Send_Data(0xA7);
  ili9341_Send_Data(0x27);
  ili9341_Send_Data(0x04);

  /* Colomn address set */
  ili9341_Write_Reg(LCD_COLUMN_ADDR);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0xEF);

  /* Page address set */
  ili9341_Write_Reg(LCD_PAGE_ADDR);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x01);
  ili9341_Send_Data(0x3F);
  ili9341_Write_Reg(LCD_INTERFACE);
  ili9341_Send_Data(0x01);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x06);

  ili9341_Write_Reg(LCD_GRAM);
  LCD_Delay(200);

  ili9341_Write_Reg(LCD_GAMMA);
  ili9341_Send_Data(0x01);

  ili9341_Write_Reg(LCD_PGAMMA);
  ili9341_Send_Data(0x0F);
  ili9341_Send_Data(0x29);
  ili9341_Send_Data(0x24);
  ili9341_Send_Data(0x0C);
  ili9341_Send_Data(0x0E);
  ili9341_Send_Data(0x09);
  ili9341_Send_Data(0x4E);
  ili9341_Send_Data(0x78);
  ili9341_Send_Data(0x3C);
  ili9341_Send_Data(0x09);
  ili9341_Send_Data(0x13);
  ili9341_Send_Data(0x05);
  ili9341_Send_Data(0x17);
  ili9341_Send_Data(0x11);
  ili9341_Send_Data(0x00);
  ili9341_Write_Reg(LCD_NGAMMA);
  ili9341_Send_Data(0x00);
  ili9341_Send_Data(0x16);
  ili9341_Send_Data(0x1B);
  ili9341_Send_Data(0x04);
  ili9341_Send_Data(0x11);
  ili9341_Send_Data(0x07);
  ili9341_Send_Data(0x31);
  ili9341_Send_Data(0x33);
  ili9341_Send_Data(0x42);
  ili9341_Send_Data(0x05);
  ili9341_Send_Data(0x0C);
  ili9341_Send_Data(0x0A);
  ili9341_Send_Data(0x28);
  ili9341_Send_Data(0x2F);
  ili9341_Send_Data(0x0F);

  ili9341_Write_Reg(LCD_SLEEP_OUT);
  LCD_Delay(200);
  ili9341_Write_Reg(LCD_DISPLAY_ON);
  /* GRAM start writing */
  ili9341_Write_Reg(LCD_GRAM);
}


/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  /* Display On */
  ili9341_Write_Reg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  ili9341_Write_Reg(LCD_DISPLAY_OFF);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_Write_Reg(uint8_t LCD_Reg)
{
  LCD_IO_WriteReg(LCD_Reg);
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9341_Send_Data(uint16_t RegValue)
{
  LCD_IO_WriteData(RegValue);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
uint32_t ili9341_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  /* Read a max of 4 bytes */
  return (LCD_IO_ReadData(RegValue, ReadSize));
}


/******************************* SPI Routines *********************************/

/**
  * @brief  SPI Bus initialization
  */
static void SPI_Init(void)
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI configuration -----------------------------------------------------*/
    SpiHandle.Instance = DISCOVERY_SPI;
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz)
       to verify these constraints:
       - ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
       - l3gd20 SPI interface max baudrate is 10MHz for write/read
       - PCLK2 frequency is set to 90 MHz
    */
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    /* On STM32F429I-Discovery, LCD ID cannot be read then keep a common configuration */
    /* for LCD and GYRO (SPI_DIRECTION_2LINES) */
    /* Note: To read a register a LCD, SPI_DIRECTION_1LINE should be set */
    SpiHandle.Init.Direction      = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase       = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity    = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial  = 7;
    SpiHandle.Init.DataSize       = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit       = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS            = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode         = SPI_TIMODE_DISABLED;
    SpiHandle.Init.Mode           = SPI_MODE_MASTER;

    SPI_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }
}

/**
  * @brief  Reads 4 bytes from device.
  * @param  ReadSize: Number of bytes to read (max 4 bytes)
  * @retval Value read on the SPI
  */
static uint32_t SPI_Read(uint8_t ReadSize)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue;

  status = HAL_SPI_Receive(&SpiHandle, (uint8_t*) &readvalue, ReadSize, SpiTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPI_Error();
  }

  return readvalue;
}

/**
  * @brief  Writes a byte to device.
  * @param  Value: value to be written
  */
static void SPI_Write(uint16_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&SpiHandle, (uint8_t*) &Value, 1, SpiTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    SPI_Error();
  }
}


/**
  * @brief  SPI error treatment function.
  */
static void SPI_Error(void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&SpiHandle);

  /* Re- Initialize the SPI communication BUS */
  SPI_Init();
}

/**
  * @brief  SPI MSP Init.
  * @param  hspi: SPI handle
  */
static void SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable SPI clock */
  DISCOVERY_SPI_CLK_ENABLE();

  /* Enable DISCOVERY_SPI GPIO clock */
  DISCOVERY_SPI_GPIO_CLK_ENABLE();

  /* configure SPI SCK, MOSI and MISO */
  GPIO_InitStructure.Pin    = (DISCOVERY_SPI_SCK_PIN | DISCOVERY_SPI_MOSI_PIN | DISCOVERY_SPI_MISO_PIN);
  GPIO_InitStructure.Mode   = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull   = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed  = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = DISCOVERY_SPI_AF;
  HAL_GPIO_Init(DISCOVERY_SPI_GPIO_PORT, &GPIO_InitStructure);
}

/********************************* LINK LCD ***********************************/

/**
  * @brief  Configures the LCD_SPI interface.
  */
void LCD_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(Is_LCD_IO_Initialized == 0)
  {
    Is_LCD_IO_Initialized = 1;

    /* Configure in Output Push-Pull mode */
    LCD_WRX_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin     = LCD_WRX_PIN;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(LCD_WRX_GPIO_PORT, &GPIO_InitStructure);

    LCD_RDX_GPIO_CLK_ENABLE();
    GPIO_InitStructure.Pin     = LCD_RDX_PIN;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(LCD_RDX_GPIO_PORT, &GPIO_InitStructure);

    /* Configure the LCD Control pins ----------------------------------------*/
    LCD_NCS_GPIO_CLK_ENABLE();

    /* Configure NCS in Output Push-Pull mode */
    GPIO_InitStructure.Pin     = LCD_NCS_PIN;
    GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull    = GPIO_NOPULL;
    GPIO_InitStructure.Speed   = GPIO_SPEED_FAST;
    HAL_GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStructure);

    /* Set or Reset the control line */
    LCD_CS_LOW();
    LCD_CS_HIGH();

    SPI_Init();
  }
}

/**
  * @brief  Writes register value.
  */
void LCD_IO_WriteData(uint16_t RegValue)
{
  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Reset LCD control line(/CS) and Send data */
  LCD_CS_LOW();
  SPI_Write(RegValue);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Writes register address.
  */
void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Reset WRX to send command */
  LCD_WRX_LOW();

  /* Reset LCD control line(/CS) and Send command */
  LCD_CS_LOW();
  SPI_Write(Reg);

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();
}

/**
  * @brief  Reads register value.
  * @param  RegValue Address of the register to read
  * @param  ReadSize Number of bytes to read
  * @retval Content of the register value
  */
uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  uint32_t readvalue = 0;

  /* Select: Chip Select low */
  LCD_CS_LOW();

  /* Reset WRX to send command */
  LCD_WRX_LOW();

  SPI_Write(RegValue);

  readvalue = SPI_Read(ReadSize);

  /* Set WRX to send data */
  LCD_WRX_HIGH();

  /* Deselect: Chip Select high */
  LCD_CS_HIGH();

  return readvalue;
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
