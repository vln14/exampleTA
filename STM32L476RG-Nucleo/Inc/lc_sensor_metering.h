/**
******************************************************************************
* @file    lc_sensor_metering.h
* @author  MCD Application Team
* @version V1.0.0
* @date    8-September-2017
* @brief   LC sensor system information.
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
* All rights reserved.</center></h2>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
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
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LC_SENSOR_METERING_H
#define __LC_SENSOR_METERING_H


#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "math.h"

  /* external defines -----------------------------------------------------------*/
  /** @defgroup LC_Appli_Config To configure application
  * @{
  */
  /** @brief Select demonstration mode:
  * 1 = Basic counting demo with 1 sensor
  * 2 = Counting demo with 2 sensors rotation and direction detection
  * 3 = Tachometer demo with 2 sensors rotation and direction detection
  * 4 = Counting demo with 2 x 2 sensors 
  */
#define LC_SENSOR_DEMO 1

  /** @brief Select board configuration (Hardware dependant):
  * 1 =  board configuration 1 with Vmid on PA4 pin
  * 2 = board configuration 2 with Vmid on PA5 pin
  */
#define BOARD_CONFIG 2

  /** @brief Enable USART2 communication to send status infos to user or others peripherals
  * 0 = USART2 disabled
  * 1 = USART2 enabled 
  */
#define USART2_ENABLE 1

  /** @brief Select excitation mode:
  * 0 = LC excitation to Vss
  * 1 = LC excitation to Vdda
  */
#define LC_EXCIT_MODE 1
  /**
  * @}
  */

  /** @defgroup LC_Debug Debug purpose
  * @{
  */
  /* #define COMP1_OUT_ENABLED */        /*!< Uncomment to enable comparator 1 output for pulses count observation (debug only) - PB0 pin */
  /* #define COMP2_OUT_ENABLED */        /*!< Uncomment to enable comparator 2 output for pulses count observation (debug only) - PB11 pin */
  /**
  * @}
  */

  /** @defgroup LC_Hardware Hardware configuration
  * @{
  */
#if (LC_SENSOR_DEMO == 1)               /* Demo1 configuration - Basic counting demo with 1 sensor */
#define SENSOR_1_ENABLED              /*!< 1st LC sensor enabled */
#endif /* LC_SENSOR_DEMO 1 */

#if (LC_SENSOR_DEMO == 2)               /* Demo2 configuration - Counting demo with 2 sensors rotation/direction detection*/
#define SENSOR_1_ENABLED              /*!< 1st LC sensor enabled */
#define SENSOR_2_ENABLED              /*!< 2nd LC sensors enabled */
#endif /* LC_SENSOR_DEMO 2 */

#if (LC_SENSOR_DEMO == 3)               /* Demo3 configuration - Tachometer demo with 2 sensors rotation/direction detection */
#define SENSOR_1_ENABLED              /*!< 1st LC sensor enabled */
#define SENSOR_2_ENABLED              /*!< 2nd LC sensors enabled */
#endif /* LC_SENSOR_DEMO 3 */

#if (LC_SENSOR_DEMO == 4)               /* Demo4 configuration - Counting demo with 2 x 2 sensors*/
#define SENSOR_1_ENABLED              /*!< 1st LC sensor enabled */
#define SENSOR_2_ENABLED              /*!< 2nd LC sensors enabled */
#define SENSOR_3_ENABLED              /*!< 3rd LC sensors enabled */
#define SENSOR_4_ENABLED              /*!< 4th LC sensors enabled */
#endif /* LC_SENSOR_DEMO 4 */
  /**
  * @}
  */

  /* Board Configuration 1
  IOs configuration for the STM32L4xx devices
  +-------------------------------+
  | LC IOs  | PINs  | LC Function |
  |---------|-------|-------------|
  | LC_IO1  | PA4   | Vmid        |
  | LC_IO2  | PC5   | Sensor1     |
  | LC_IO3  | PB2   | Sensor2     |
  | LC_IO4  | PB4   | Sensor3     |
  | LC_IO5  | PB6   | Sensor4     |
  +-------------------------------+

  DAC Outputs for the STM32L4xx devices
  +---------------------------------------+
  |                 | DAC1                |
  |-----------------|---------------------|
  | DAC1 OUT1       | PA4 (LC_IO1)        |
  |-----------------|---------------------|
  | DAC1 OUT2       | COMP1 Input (minus) |
  |                 | COMP2 Input (minus) |
  +---------------------------------------+

  COMP Inputs for the STM32L4xx devices
  +-------------------------------------------------+
  |         Sensor1 / Sensor2       | COMP1         |
  |---------------------------------|---------------|
  | Inverting Input (minus)         | DAC1 OUT2     |
  |---------------------------------|---------------|
  | Non Inverting Input (plus)      | PC5 (LC_IO2)  |
  |                                 | PB2 (LC_IO3)  |
  +-------------------------------------------------+

  +-------------------------------------------------+
  |         Sensor3 / Sensor4       | COMP2         |
  |---------------------------------|---------------|
  | Inverting Input (minus)         | DAC1 OUT2     |
  |---------------------------------|---------------|
  | Non Inverting Input (plus)      | PB4 (LC_IO4)  |
  |                                 | PB6 (LC_IO5)  |
  +-------------------------------------------------+
  */
#if (BOARD_CONFIG == 1)
  /** @defgroup LC_Hardware_Config1 Board config 1 - IOs definitions and DAC outputs
  * @{
  */
#define DAC_CHANNEL_LC_VMID DAC_CHANNEL_1       /*!< DAC_CHANNEL_1 or DAC_CHANNEL_2 value for Vmid generation */
#define DAC_CR_LC_EN_VMID DAC_CR_EN1            /*!< DAC_CR_EN1 or DAC_CR_EN2 value vor Vmid generation */

#define DAC_CHANNEL_LC_VCMP DAC_CHANNEL_2 /*!< DAC_CHANNEL_1 or DAC_CHANNEL_2 value for Vcmp generation*/
#define DAC_CR_LC_EN_VCMP DAC_CR_EN2  /*!< DAC_CR_EN1 or DAC_CR_EN2 value for Vcmp generation*/

#define GPIO_LC_IO1 GPIOA             /*!< GPIO group used for LC sensor IO1  - DAC output */
#define GPIO_LC_IO1_PIN_NUMBER 4      /*!< GPIO PIN number used for LC sensor IO1 */
#define GPIO_LC_IO1_PIN GPIO_PIN_4    /*!< GPIO PIN used for LC sensor IO1 */

#define GPIO_LC_IO2 GPIOC             /*!< GPIO group used for LC sensor IO2 */
#define GPIO_LC_IO2_PIN_NUMBER 5      /*!< GPIO PIN number used for LC sensor IO2 */
#define GPIO_LC_IO2_PIN GPIO_PIN_5    /*!< GPIO PIN used for LC sensor IO2 */

#define GPIO_LC_IO3 GPIOB             /*!< GPIO group used for LC sensor IO3 */
#define GPIO_LC_IO3_PIN_NUMBER 2      /*!< GPIO PIN number used for LC sensor IO3 */
#define GPIO_LC_IO3_PIN GPIO_PIN_2    /*!< GPIO PIN used for LC sensor IO3 */

#define GPIO_LC_IO4 GPIOB             /*!< GPIO group used for LC sensor IO4 */
#define GPIO_LC_IO4_PIN_NUMBER 4      /*!< GPIO PIN number used for LC sensor IO4 */
#define GPIO_LC_IO4_PIN GPIO_PIN_4    /*!< GPIO PIN used for LC sensor IO4 */

#define GPIO_LC_IO5 GPIOB             /*!< GPIO group used for LC sensor IO5 */
#define GPIO_LC_IO5_PIN_NUMBER 6      /*!< GPIO PIN number used for LC sensor IO5 */
#define GPIO_LC_IO5_PIN GPIO_PIN_6    /*!< GPIO PIN used for LC sensor IO5 */
  /**
  * @}
  */
#endif /* BOARD_CONFIG 1*/

  /* Board Configuration 2
  IOs configuration
  +-------------------------------+
  | LC IOs  | PINs  | LC Function |
  |---------|-------|-------------|
  | LC_IO1  | PA5   | Vmid        |
  | LC_IO2  | PC5   | Sensor1     |
  | LC_IO3  | PB2   | Sensor2     |
  | LC_IO4  | PB4   | Sensor3     |
  | LC_IO5  | PB6   | Sensor4     |
  +-------------------------------+

  DAC Outputs for the STM32L4xx devices
  +---------------------------------------+
  |                 | DAC1                |
  |-----------------|---------------------|
  | DAC1 OUT1       | COMP1 Input (minus) |
  |                 | COMP2 Input (minus) |
  |-----------------|---------------------|
  | DAC1 OUT2       | PA5 (LC_IO1)        |
  +---------------------------------------+

  COMP Inputs for the STM32L4xx devices
  +-------------------------------------------------+
  |         Sensor1 / Sensor2       | COMP1         |
  |---------------------------------|---------------|
  | Inverting Input (minus)         | DAC1 OUT1     |
  |---------------------------------|---------------|
  | Non Inverting Input (plus)      | PC5 (LC_IO2)  |
  |                                 | PB2 (LC_IO3)  |
  +-------------------------------------------------+

  +-------------------------------------------------+
  |         Sensor3 / Sensor4       | COMP2         |
  |---------------------------------|---------------|
  | Inverting Input (minus)         | DAC1 OUT1     |
  |---------------------------------|---------------|
  | Non Inverting Input (plus)      | PB4 (LC_IO4)  |
  |                                 | PB6 (LC_IO5)  |
  +-------------------------------------------------+
  */
#if (BOARD_CONFIG == 2)
  /** @defgroup LC_Hardware_Config2 Board config 2 - IOs definitions and DAC outputs
  * @{
  */
#define DAC_CHANNEL_LC_VMID DAC_CHANNEL_2 /*!< DAC_CHANNEL_1 or DAC_CHANNEL_2 value for Vmid generation*/
#define DAC_CR_LC_EN_VMID DAC_CR_EN2  /*!< DAC_CR_EN1 or DAC_CR_EN2 value vor Vmid generation*/

#define DAC_CHANNEL_LC_VCMP DAC_CHANNEL_1 /*!< DAC_CHANNEL_1 or DAC_CHANNEL_2 value for Vcmp generation*/
#define DAC_CR_LC_EN_VCMP DAC_CR_EN1  /*!< DAC_CR_EN1 or DAC_CR_EN2 value for Vcmp generation*/

#define GPIO_LC_IO1 GPIOA             /*!< GPIO group used for LC sensor IO1 - DAC output */
#define GPIO_LC_IO1_PIN_NUMBER 5      /*!< GPIO PIN number used for LC sensor IO1 */
#define GPIO_LC_IO1_PIN GPIO_PIN_5    /*!< GPIO PIN used for LC sensor IO1 */

#define GPIO_LC_IO2 GPIOC             /*!< GPIO group used for LC sensor IO2 */
#define GPIO_LC_IO2_PIN_NUMBER 5      /*!< GPIO PIN number used for LC sensor IO2 */
#define GPIO_LC_IO2_PIN GPIO_PIN_5    /*!< GPIO PIN used for LC sensor IO2 */

#define GPIO_LC_IO3 GPIOB             /*!< GPIO group used for LC sensor IO3 */
#define GPIO_LC_IO3_PIN_NUMBER 2      /*!< GPIO PIN number used for LC sensor IO3 */
#define GPIO_LC_IO3_PIN GPIO_PIN_2    /*!< GPIO PIN used for LC sensor IO3 */

#define GPIO_LC_IO4 GPIOB             /*!< GPIO group used for LC sensor IO4 */
#define GPIO_LC_IO4_PIN_NUMBER 4      /*!< GPIO PIN number used for LC sensor IO4 */
#define GPIO_LC_IO4_PIN GPIO_PIN_4    /*!< GPIO PIN used for LC sensor IO4 */

#define GPIO_LC_IO5 GPIOB             /*!< GPIO group used for LC sensor IO5 */
#define GPIO_LC_IO5_PIN_NUMBER 6      /*!< GPIO PIN number used for LC sensor IO5 */
#define GPIO_LC_IO5_PIN GPIO_PIN_6    /*!< GPIO PIN used for LC sensor IO5 */
#endif /* BOARD_CONFIG 2 */

#define DAC_CR_LC_DIS_VMID ~(DAC_CR_LC_EN_VMID)                 /*!< Disable Vmid */
#define DAC_CR_LC_DIS_VCMP ~(DAC_CR_LC_EN_VCMP)                 /*!< Disable Vcmp */
#define DAC_CR_LC_EN_VMID_EN_VCMP (DAC_CR_EN1 | DAC_CR_EN2)     /*!< Enable DAC1 and DAC2 */
#define DAC_CR_LC_DIS_VMID_DIS_VCMP ~(DAC_CR_EN1 | DAC_CR_EN2)  /*!< Disable DAC1 and DAC2*/
  /**
  * @}
  */

  /* for GNUC Compiler */
#if defined (__GNUC__)
  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_0_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  __asm volatile("str %1, [%0]\n" \
                 "str %2, [%0]\n" \
                 :: "r" (__GROUP__),\
                 "r" (__OUTPUT_PP__),\
                 "r" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 1 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_1_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  __ASM volatile("str %1, [%0]\n" \
                 "mov r0, r0\n" \
                 "str %2, [%0]\n" \
                 :: "r" (__GROUP__),\
                 "r" (__OUTPUT_PP__),\
                 "r" (__ANALOG__):"r0","r1","r2");
  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 2 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_2_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  __ASM volatile("str %1, [%0]\n" \
                 "mov r0, r0\n" \
                 "str %2, [%0]\n" \
                 :: "r" (__GROUP__),\
                 "r" (__OUTPUT_PP__),\
                 "r" (__ANALOG__):"r0","r1","r2");

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 3 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_3_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  __ASM volatile("str %1, [%0]\n" \
                 "mov r0, r0\n" \
                 "mov r0, r0\n" \
                 "mov r0, r0\n" \
                 "str %2, [%0]\n" \
                 :: "r" (__GROUP__),\
                 "r" (__OUTPUT_PP__),\
                 "r" (__ANALOG__):"r0","r1","r2");

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 4 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_4_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  __ASM volatile("str %1, [%0]\n" \
                 "mov r0, r0\n" \
                 "mov r0, r0\n" \
                 "mov r0, r0\n" \
                 "mov r0, r0\n" \
                 "str %2, [%0]\n" \
                 :: "r" (__GROUP__),\
                 "r" (__OUTPUT_PP__),\
                 "r" (__ANALOG__):"r0","r1","r2");
  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + x cycles.
  * @param  group: specifies the IO group.
  * @param  output_pp: specifies register value to set IO in output PP state
  * @param  analog: specifies register value to set IO in analog state
  * @param  texcit: specifies number of cycles to loop
  * @retval None
  */
  __attribute__((optimize("-O0"))) __STATIC_INLINE void __LC_EXCIT(uint32_t group, uint32_t output_pp, uint32_t analog, uint32_t texcit)
  {
    __DSB();
    __ISB();
    __ASM volatile(\
                   "push {r0-r3}\n"\
                   "str %1, [%0]\n" \
                   "1:\n" \
                   "subs %3, #1\n" \
                   "bne 1b\n" \
                   "str %2, [%0]\n" \
                   "pop {r0-r3}\n"\
                   :: "r" (group), \
                   "r" (output_pp), \
                   "r" (analog), \
                   "r" (texcit): "r0", "r1", "r2", "r3");
  }

  /* for IAR Compiler */
#elif defined (__ICCARM__)
  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_0_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  asm volatile("str %1, [%0]\n" \
               "str %2, [%0]\n" \
               :: "r0" (__GROUP__),\
               "r1" (__OUTPUT_PP__),\
               "r2" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 1 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_1_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  asm volatile("str %1, [%0]\n" \
               "mov r0, r0\n" \
               "str %2, [%0]\n" \
               :: "r0" (__GROUP__),\
               "r1" (__OUTPUT_PP__),\
               "r2" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 2 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_2_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  asm volatile("str %1, [%0]\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "str %2, [%0]\n" \
               :: "r0" (__GROUP__),\
               "r1" (__OUTPUT_PP__),\
               "r2" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 3 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_3_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  asm volatile("str %1, [%0]\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "str %2, [%0]\n" \
               :: "r0" (__GROUP__),\
               "r1" (__OUTPUT_PP__),\
               "r2" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + 4 cycle.
  * @param  __GROUP__: specifies the IO group.
  * @param  __OUTPUT_PP__: specifies register value to set IO in output PP state
  * @param  __ANALOG__: specifies register value to set IO in analog state
  * @retval None
  */
#define __LC_EXCIT_4_NOP(__GROUP__, __OUTPUT_PP__, __ANALOG__)         \
  asm volatile("str %1, [%0]\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "mov r0, r0\n" \
               "str %2, [%0]\n" \
               :: "r0" (__GROUP__),\
               "r1" (__OUTPUT_PP__),\
               "r2" (__ANALOG__));

  /**
  * @brief  Macros used to start oscillations on LC sensor with minimum time + x cycles.
  * @param  group: specifies the IO group.
  * @param  output_pp: specifies register value to set IO in output PP state
  * @param  analog: specifies register value to set IO in analog state
  * @param  texcit: specifies number of cycles to loop
  * @retval None
  */
  __STATIC_INLINE void __LC_EXCIT(uint32_t group, uint32_t output_pp, uint32_t analog, uint32_t texcit)
  {
    __DSB();
    __ISB();
    __ASM volatile(\
                   "push {r0-r3}\n"\
                   "str %1, [%0]\n" \
                   "1:\n" \
                   "subs %3, #1\n" \
                   "bne 1b\n" \
                   "str %2, [%0]\n" \
                   "pop {r0-r3}\n"\
                   :: "r0" (group), \
                   "r1" (output_pp), \
                   "r2" (analog), \
                   "r3" (texcit));
  }
#else
#error Unknown compiler
#endif /* define compiler */

  /**
  * @brief  Macro used to insert a waiting time
  * @param  __TIME__: specifies the ARR value to load in One pulse Timer.
  * @retval None
  */
#define __LC_WAIT(__TIME__) \
  do {\
    TIM6->ARR = (__TIME__);             /* Reload ARR value*/\
    TIM6->SR = ~(TIM_FLAG_UPDATE);      /* Clear 1st dummy IT when starting the counter */\
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);\
    RCC->CFGR = RCC_CFGR_HPRE_DIV64;    /* Decrease the HCLK clock during sleep */\
    TIM6->CR1 |= TIM_CR1_CEN;           /* Start the counter (single-shot) */\
    __SEV();\
    __WFE();\
    __WFE();                            /* Sleep */\
    RCC->CFGR = RCC_CFGR_HPRE_DIV1;     /* Go back to full speed */\
  } while(0)

  /**
  * @brief  Macro used to insert a waiting during measurement
  * @param  __TIME__: specifies the ARR value to load in One pulse Timer.
  * @retval None
  */
#define __LC_MEASURE(__TIME__) \
  do {\
    TIM6->ARR = (__TIME__);             /* Reload ARR value*/\
    TIM6->SR = ~(TIM_FLAG_UPDATE);      /* Clear 1st dummy IT when starting the counter */\
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);\
    RCC->CFGR = RCC_CFGR_HPRE_DIV64;    /* Decrease the HCLK clock during sleep */\
    DAC1->CR &= DAC_CR_LC_DIS_VMID;     /* Disable Vmid */\
    TIM6->CR1 |= TIM_CR1_CEN;           /* Start the counter (single-shot) */\
    __SEV();\
    __WFE();\
    __WFE();                            /* Sleep */\
    RCC->CFGR = RCC_CFGR_HPRE_DIV1;     /* Go back to full speed */\
  } while(0)

  /* Exported types ------------------------------------------------------------*/
  /**
  * @brief  LC sensor mode enumeration
  */
  typedef enum
  {
    LC_SENSOR_STD,
    LC_SENSOR_LOW_POWER,
  } LcSensorModeTypeEnum;

  /**
  * @brief  LC sensor state enumeration
  */
  typedef enum
  {
    NO_METAL,
    METAL
  } LcSensor_StatusTypeEnum;

  /**
  * @brief  Periodic calibration state enumeration
  */
  typedef enum
  {
    CAL_TO_PERFORM,
    CAL_DONE,
    CAL_ON_GOING,
    CAL_FAILED
  } CalStateTypeEnum;

  /**
  * @brief  Periodic calibration mode enumeration
  */
  typedef enum
  {
    STATIC,
    DYNAMIC
  } CalModeTypeEnum;

  /**
  * @brief  Periodic calibration status enumeration
  */
  typedef enum
  {
    CAL_ENABLED,
    CAL_DISABLED
  } CalStatusTypeEnum;

  /**
  * @brief  First calibration configuration structure
  */
  typedef struct
  {
    CalStatusTypeEnum Status;             /*!< ENABLE or DISABLE */
    CalModeTypeEnum Mode;                 /*!< STATIC or DYNAMIC (with rotating wheel) */
    volatile uint32_t DacVcmpMin;         /*!< Set the minimum DAC value to use during calibration for COMP threshold */
    volatile uint32_t DacVcmpMax;         /*!< Set the maximum DAC value to use during calibration for COMP threshold */
    volatile uint32_t DacVcmpStep;        /*!< Set the DAC step value during calibration for COMP threshold */
    volatile uint32_t PulseCount;         /*!< Count number targeted - used for STATIC calibration */
    volatile uint32_t PulseCountDeltaMin; /*!< Set the difference to guarantee between min an max pulse count - used for DYNAMIC*/
    volatile uint32_t Measures;           /*!< Set the number of measurements to perform during calibration*/
  }
  LcSensor_FirstCalConf_f;

  /**
  * @brief  Periodic calibration configuration structure
  */
  typedef struct
  {
    CalStatusTypeEnum Status;             /*!< ENABLE or DISABLE */
    CalModeTypeEnum Mode;                 /*!< STATIC or DYNAMIC (with rotating wheel) */
    uint32_t CountDrift;                  /*!< Valid periodic calibration values only if the drift does not exceed this value*/
  }
  LcSensor_PeriodicCalConf_f;

  /**
  * @brief  Calibration state structure
  */
  typedef struct
  {
    CalStateTypeEnum State;                     /*!< Calibration state CAL_DONE or CAL_ON_GOING */
    volatile uint32_t Sensor1CountTransitions; /*!< Variable used to store transitions number during calibration */
    volatile uint32_t Sensor1CountMax;          /*!< Variable used to store current maximum count for sensor 1 calibration*/
    volatile uint32_t Sensor1CountMin;          /*!< Variable used to store current minimum count for sensor 1 calibration*/
    volatile uint32_t Sensor2CountTransitions; /*!< Variable used to store transitions number during calibration */
    volatile uint32_t Sensor2CountMax;          /*!< Variable used to store current maximum count for sensor 2 calibration*/
    volatile uint32_t Sensor2CountMin;          /*!< Variable used to store current minimum count for sensor 2 calibration*/
    uint32_t Sensor3CountTransitions; /*!< Variable used to store transitions number during calibration */
    uint32_t Sensor3CountMax;  /*!< Variable used to store current maximum count for sensor 3 calibration*/
    uint32_t Sensor3CountMin;  /*!< Variable used to store current minimum count for sensor 3 calibration*/
    uint32_t Sensor4CountTransitions; /*!< Variable used to store transitions number during calibration */
    uint32_t Sensor4CountMax;  /*!< Variable used to store current maximum count for sensor 4 calibration*/
    uint32_t Sensor4CountMin;  /*!< Variable used to store current minimum count for sensor 4 calibration*/
  }
  LcSensor_CalState_f;

  /**
  * @brief  LC sensor initialisation structure
  */
  typedef struct
  {
    LcSensorModeTypeEnum Mode;            /*!< Mode LC_SENSOR_STD or LC_SENSOR_LOW_POWER */
    double_t Sampling;                    /*!< Measurement sampling time (value in Hz) - Maximum value is 500Hz*/
    uint32_t DacVmid;                     /*!< Medium (0x800) (Set LC mid point VMID to Vdda/2)*/
    uint32_t DacVcmp;                     /*!< Set to medium value with margin (ex: 0xC00) (Set COMP threshold about 750mV below Vmid at Vdd=3V) - Value updated if calibration enabled */
    uint32_t DacVmidRefreshTime;          /*!< Set the DAC stabilization time before measurement - Depends to Sampling value */
    uint32_t TimeBetweenSensor;           /*!< Default time between 2 sensors measurements */
    LcSensor_FirstCalConf_f FirstCal;/*!< First calibration parameters */
    LcSensor_PeriodicCalConf_f PeriodicCal; /*!< Periodic calibration parameters */
  }
  LcSensor_Init_f;

  /**
  * @brief  Sensors configuration structure
  */
  typedef struct
  {
    double_t Ls;                          /*!< Inductor value */
    double_t Cs;                          /*!< Capacitor value */
    double_t FreqOsc;                     /*!< Oscillation frequency */
    volatile uint32_t Texcit;             /*!< Excitation time - Minimum value is 1 */
    volatile uint32_t Tcapture;           /*!< Capture time during oscillation */
    volatile double_t CountDetectPercent; /*!< Percent of maximum pulses count for detection threshold ex:0.8 for 80% */
  }
  LcSensor_SensorInit_f;

  /**
  * @brief  LC status
  */
  typedef struct
  {
    LcSensor_CalState_f CalStatus;        /*!< Variables updated at each calibration step */
    volatile uint32_t MeasuresCount;      /*!< Variable updated at each measurement to store measurements number */
    volatile uint32_t EdgeCount;          /*!< Variable used to store edge count number */
    volatile uint32_t EdgeCountPos;       /*!< Variable used to store edge count number */
    volatile uint32_t EdgeCountNeg;       /*!< Variable used to store edge count number */
    volatile uint32_t Errors;             /*!< variable used to store errors count number */
    volatile uint32_t PreviousEdgeCount;  /*!< Variable used to store previous edge count number */
    volatile uint32_t EdgeCount2;         /*!< Variable used to store edge count number for demo 4*/
    volatile uint32_t EdgeCountPos2;      /*!< Variable used to store edge count number for demo4 */
    volatile uint32_t EdgeCountNeg2;      /*!< Variable used to store edge count number for demo4 */
    volatile uint32_t Errors2;            /*!< variable used to store errors count number for demo4 */
    volatile uint32_t PreviousEdgeCount2; /*!< Variable used to store previous edge count number  for demo4 */
    volatile uint32_t WakeUpCounter;      /*!< Variable used to store current wakeup counter */
    volatile double_t Sampling;           /*!< Variable used to store the real sampling value*/
    volatile double_t Rpm;                /*!< Variable used to store Rotations Per Minutes */
    volatile double_t Rps;                /*!< Variable used to store Rotations Per Seconds */
  }
  LcSensor_LcStatus_f;

  /**
  * @brief  Sensors variables structure
  */
  typedef struct
  {
    uint32_t CountTmp1;                  /*!< tmp variable used to store counter status */
    uint32_t CountTmp2;                  /*!< tmp variable used to store counter status */
    uint32_t CountDetect;                /*!< Count number for detection */
    uint32_t CountValue;                 /*!< Variable used to store current oscillations number */
    uint32_t CountMax;                   /*!< Variable used to store maximum count number (air)*/
    uint32_t CountMin;                   /*!< Variable used to store maximum count number (metal)*/
    uint32_t Transitions;                /*!< Variable used to count transitions number */
    LcSensor_StatusTypeEnum PreviousStatus;/*!< Status METAL or NO_METAL */
    LcSensor_StatusTypeEnum Status;      /*!< Previous status METAL or NO_METAL */
  }
  LcSensor_Sensor_f;

  /**
  * @brief  LC dynamic variables structure used to drive LC sensor
  */
  typedef struct
  {
    uint32_t IO1_GPIO_GROUP;            /*!< To store IO1 GPIOx_MODER address */
    uint32_t IO1_ANALOG;                /*!< To store GPIOx_MODER for IO1 analog state */
    uint32_t IO1_OUTPUT_PP;             /*!< To store GPIOx_MODER for IO1 output PP */
    uint32_t IO2_GPIO_GROUP;            /*!< To store IO2 GPIOx_MODER address */
    uint32_t IO2_ANALOG;                /*!< To store GPIOx_MODER for IO2 analog state */
    uint32_t IO2_OUTPUT_PP;             /*!< To store GPIOx_MODER for IO2 output PP */
    uint32_t IO3_GPIO_GROUP;            /*!< To store IO3 GPIOx_MODER address */
    uint32_t IO3_ANALOG;                /*!< To store GPIOx_MODER for IO3 analog state */
    uint32_t IO3_OUTPUT_PP;             /*!< To store GPIOx_MODER for IO3 output PP */
    uint32_t IO4_GPIO_GROUP;            /*!< To store IO4 GPIOx_MODER address */
    uint32_t IO4_ANALOG;                /*!< To store GPIOx_MODER for IO4 analog state */
    uint32_t IO4_OUTPUT_PP;             /*!< To store GPIOx_MODER for IO4 output PP */
    uint32_t IO5_GPIO_GROUP;            /*!< To store IO5 GPIOx_MODER address */
    uint32_t IO5_ANALOG;                /*!< To store GPIOx_MODER for IO5 analog state */
    uint32_t IO5_OUTPUT_PP;             /*!< To store GPIOx_MODER for IO5 output PP */
  }
  LcSensor_IO_f;

  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* External variables --------------------------------------------------------*/
  /** @addtogroup LC_Exported_Variables
  * @{
  */
  extern LcSensor_Init_f LcConfig;              /*!< Lc application configuration */
  extern LcSensor_LcStatus_f LcStatus;          /*!< Lc application status */
  extern LcSensor_SensorInit_f LcConfigSensor1; /*!< Sensor1 configuration */
  extern LcSensor_SensorInit_f LcConfigSensor2; /*!< Sensor2 configuration */
  extern LcSensor_SensorInit_f LcConfigSensor3; /*!< Sensor3 configuration */
  extern LcSensor_SensorInit_f LcConfigSensor4; /*!< Sensor4 configuration */
  extern LcSensor_Sensor_f LcSensor1;           /*!< Sensor1 status */
  extern LcSensor_Sensor_f LcSensor2;           /*!< Sensor2 status */
  extern LcSensor_Sensor_f LcSensor3;           /*!< Sensor3 status */
  extern LcSensor_Sensor_f LcSensor4;           /*!< Sensor4 status */
  extern LcSensor_IO_f LcIO;                    /*!< Variables to drive sensors */
  /**
  * @}
  */
  /* Exported functions --------------------------------------------------------*/
  /** @addtogroup LC_Exported_Functions
  * @{
  */
  void LcSensorConfig(void);                    /*!< Configure application */
  void LcSensorDemo(void);                      /*!< Start LC sensor demonstration */
  void LcSensorInit(void);                      /*!< Initialize all variables used for Lc Sesnor demo */
  /**
  * @}
  */

  /* Private typedef -----------------------------------------------------------*/
  /* Private macros ------------------------------------------------------------*/
  /**
  * @brief  Compute Tx Buffer size.
  * @retval Size of Transmission buffer
  */
#define TXBUFFERSIZE(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)) - 1) /*!< Size of Transmission buffer */
  /* Private variables ---------------------------------------------------------*/
  /* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __LC_SENSOR_METERING_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

