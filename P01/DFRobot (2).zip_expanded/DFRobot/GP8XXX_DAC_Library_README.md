# STM32 Library for GP8XXX DAC Series

Devolopers:

Aruna S. Kumara J.A. ,Lakshitha M. Perera K. , Thilini I. Hettiarachchi, Maheshi Ruwanthika R. M. (University of Moratuwa, Sri Lanka)
Arunya P. Senadeera S.D. (Advanced Telecommunication Laboratory, Asian Institute of Technology, Thailand)


Release Date: 12/06/2024

Version: 1.00

This is a library for the STM32 microcontrollers that provides an easy-to-use interface for controlling the GP8XXX series of digital-to-analog converters (DACs) from "DFROBOT Gravity" over I2C.

## Supported DAC Modules

This library currently supports the following DAC modules from the GP8XXX series by DFRobot:

- GP8211s
- GP8403
- GP8413  
- GP8503
- GP8512

Support for additional models may be added in future updates.

## Features

- Initialize and configure the DAC with various settings
- Set the output voltage for the DAC
- Support for different DAC output ranges (5V, 10V)(if applicable)
- Support for multiple DAC channels (if applicable)
- Store the output voltage value in the DAC `s internal memory
- Examples and sample code included

## Requirements

- STM32 microcontroller
- GP8XXX DAC from DFRobot
- STM32 HAL drivers

## Installation

1. Add the library source files to your project
2. Include the  `DFR_i2c.h ` header file in your code

## Usage

1. Create instances of the  `DFRobot_GP8XXX_IIC ` struct for each DAC model you want to use.
2. Initialize each DAC with the desired configuration using the  `GP8XXX_IIC_begin() ` function, providing the appropriate GPIO pins for SCL and SDA (these pins should be configured as "GPIO_Output").
3. Set the output range using  `GP8XXX_IIC_setDACOutRange() `.
4. Set the output voltage using  `GP8XXX_IIC_setDACOutVoltage() `, specifying the channel if the DAC model supports multiple channels.
5. Store the output voltage value using  `GP8XXX_IIC_store() `.
6. Refer to the examples and sample code for more details.

Note: You can use any two available GPIO pins for SCL and SDA. These pins should be configured as "GPIO_Output" and do not configure them as I2C pins.

## Documentation

The header file  `DFR_i2c.h ` contains detailed documentation for the library functions and their usage.

## Examples (containing only the relevent lines of code)

```c

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DFR_i2c.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

DFRobot_GP8XXX_IIC gp8211s;
DFRobot_GP8XXX_IIC gp8403;

/* USER CODE END PTD */


int main(void)
{
  /* USER CODE BEGIN 2 */

  /* for GP8211s */
  	GP8XXX_IIC_begin(&gp8211s, GP8211S_identifier, 0x58, GPIOB, GPIO_PIN_8,GPIOB,GPIO_PIN_9); // possible device address 0x58
  	GP8XXX_IIC_setDACOutRange(&gp8211s, eOutputRange10V); // 5V or 10V
  	GP8XXX_IIC_setDACOutVoltage(&gp8211s, 32767 ); // 0 to 32767
  	GP8XXX_IIC_store(&gp8211s);

  	/* for GP8403 */
  	GP8XXX_IIC_begin(&gp8403, GP8403_identifier, 0x58, GPIOA, GPIO_PIN_5, GPIOA,GPIO_PIN_6); // possible device addresses 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F
  	GP8XXX_IIC_setDACOutRange(&gp8403, eOutputRange10V); // 5V or 10V
  	GP8XXX_IIC_setDACOutVoltage(&gp8403, 4095 ,1 );// 0 to 4065
  	GP8XXX_IIC_store(&gp8403);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}