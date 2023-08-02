/**
  @page STM32L476RG-Nucleo AN4636 STM32L476RG-Nucleo Readme File
  
  @verbatim
  ******************************************************************************
  * @file STM32L476RG-Nucleo/readme.txt 
  * @author   MCD Application Team
  * @version  V1.0.0
  * @date     08/09/2017
  * @brief    Description of the AN4636 Application note's firmware.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
   @endverbatim


@par Description
This firmware gives examples to evaluate LC sensor metering with STM32L4.

There are 4 demonstrations mode available that can be selected by the line #define LC_SENSOR_DEMO x. 
- LC_SENSOR_DEMO 1: Basic counting demonstration with 1 sensor to use with the detection accessory board MB1199 or other metal accessory.
- LC_SENSOR_DEMO 2: Counting demonstration with 2 sensors: rotation/direction detections to use with a rotating wheel.
- LC_SENSOR_DEMO 3: Tachometer demonstration with 2 sensors: rotation/direction detections to use with a rotating wheel.
- LC_SENSOR_DEMO 4: Counting demonstration with 2 x 2 LC sensors: rotation/direction detections to use with a rotating wheel.

There are 2 hardwares configurations that can be selected by the line #define BOARD_CONFIG x.
- BOARD_CONFIG 1: Configuration with Vmid on PA4 pin
- BOARD_CONFIG 2: Configuration with Vmid on PA5 pin

Counters results can displayed on terminal and can be captured in a file for the evaluation results.
- USART2_ENABLE 0:USART2 communication disabled
- USART2_ENABLE 1: USART2 communication enabled
The Hyperterminal configuration is 912600B and 8-bit data with 1 stop bit.

The LcConfig.Mode variable defined in LcSensorConfig() function allows to choice low power or standard mode for debug.

The RTC tamper1 (connected to Blue user button) allows to enable and disable USART communication to perform power consumption measurements.

@par Directory contents 

  - inc: containing the user header files  
     - STM32L476RG-Nucleo/inc/stm32l4xx_hal_conf.h    Library Configuration files
     - STM32L476RG-Nucleo/inc/stm32l4xx_it.h          Interrupt handlers header files
     - STM32L476RG-Nucleo/inc/lc_sensor_metering.h    Lc sensor header file
     - STM32L476RG-Nucleo/inc/main.h                  Main header file
     - STM32L476RG-Nucleo/inc/constants.h             Constants file
     - stm32l4xx_hal_conf.h                           HAL configuration file
 
  - src: containg the user source files  
    - STM32L476RG-Nucleo/src/lc_sensor_metering.c     Lc sensor source file 
    - STM32L476RG-Nucleo/src/stm32l4xx_it.c           Interrupt handlers
    - STM32L476RG-Nucleo/src/main.c                   Main program
    - STM32L476RG-Nucleo/src/stm32l4xx_hal_msp.c      MSP Initialization and de-Initialization codes

@par Hardware and Software environment 

    - This example runs on STM32L4x Devices.
            
    - This example has been tested with STMicroelectronics STM32L476RG-NUCELO board and can be easily tailored to any other 
      supported device and development board.
    
    - IOs configuration for the STM32L4xx devices
      +-------------------------------------+
      | LC IOs  |     PINs    | LC Function |
      |---------|-------------|-------------|
      | LC_IO1  | PA4 or PA5  | Vmid        |
      | LC_IO2  | PC5         | Sensor1     |
      | LC_IO3  | PB2         | Sensor2     | 
      | LC_IO4  | PB4         | Sensor3     |
      | LC_IO5  | PB6         | Sensor4     |
      +-------------------------------------+
             
@par How to use it? 

In order to load the code, you have do the following:

 - EWARM: 
    - Open the Project.eww workspace
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)
	
- SW4STM32:
	- Open SW4STM32 toolchain
	- Import the project: STM32L476RG-Nucleo\SW4STM32\AN4636_STM32L476RG\.cproject
	- Rebuil all files (Build Release)
	- Run program: Run AN4636_STM32L476RG Release
        
 * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
 */

