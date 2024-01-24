/**
  ******************************************************************************
  * @file    lc_sensor_metering.c
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

#define _LC_SENSOR_METERING_C

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stdio.h"
#include "stm32l4xx_hal.h"
#include "lc_sensor_metering.h"
#include "stm32l4xx_it.h"
#include "math.h"

/* Private typedef ----------------------------------------------------------*/

/* Private constants ----------------------------------------------------------*/
LcSensor_Init_f LcConfig;                   /* Lc Sensor configuration */
LcSensor_SensorInit_f LcConfigSensor1;      /* Lc Sensor 1 Configuration */
LcSensor_SensorInit_f LcConfigSensor2;      /* Lc Sensor 2 Configuration */
LcSensor_SensorInit_f LcConfigSensor3;      /* Lc Sensor 3 Configuration */
LcSensor_SensorInit_f LcConfigSensor4;      /* Lc Sensor 4 Configuration */
LcSensor_Sensor_f LcSensor1;                /* Lc Sensor 1 variables */
LcSensor_Sensor_f LcSensor2;                /* Lc Sesnor 2 variables */
LcSensor_Sensor_f LcSensor3;                /* Lc sensor 3 variables */
LcSensor_Sensor_f LcSensor4;                /* Lc Sensor 4 variables */
LcSensor_IO_f LcIO;                         /* LC sensor Variables used to have a fast state modification - Updated in LcSensorInit() function */

/* Function prototypes ------------------------------------------------------*/
static void
static void LC_Calibration(void);
static uint32_t LC_Compute_Tcapture(uint32_t count_max, double_t freq_osc);
static uint32_t LC_Compute_CountDetect(uint32_t count_max, uint32_t count_min);
static uint32_t LC_Compute_CountDetectPer(uint32_t count_max, double_t percent);

/* Private macro -------------------------------------------------------------*/

/* Private define */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef Hrtc;
DAC_HandleTypeDef Hdac1;
COMP_HandleTypeDef Hcomp1;
LPTIM_HandleTypeDef Hlptim1;
TIM_HandleTypeDef Htim6;

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
COMP_HandleTypeDef Hcomp2;          /* Used only if Sensor3 and Sensor4 used */
LPTIM_HandleTypeDef Hlptim2;        /* Enabled only if Sensor3 and Sensor4 used */
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */

#if (USART2_ENABLE == 1)
UART_HandleTypeDef Huart2;
uint8_t aMsgBuffer[30];              /* Buffer used to send message by USART2 */
#endif /* USART2_ENABLE */

LcSensor_LcStatus_f LcStatus;        /* Lc Sensor status */
__IO uint8_t UserButtonStatus = 1;   /* Switch to 1 or 0 after User Button interrupt  */


/* Private function prototypes -----------------------------------------------*/
static void ErrorHandler(void);
static void LC_Init_GPIO(void);
static void SystemClock_ConfigMSI_24M(void);
static void LC_Init_RTC(void);
static void LC_init_AlarmB(void);
static void LC_Init_DAC1(void);
static void LC_Init_COMP1(void);
static void LC_Init_TIM6(void);
static void LC_Init_LPTIM1(void);
static void StopEntry(void);

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
static void Init_COMP2_LcSensor(void);          /* Enabled only if Sensor3 and Sensor4 used */
static void Init_LPTIM2_LcSensor(void);         /* Enabled only if Sensor3 and Sensor4 used */
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */

#if (USART2_ENABLE == 1)
static void LC_Init_USART2(void);
static void LC_Init_AlarmA(void);
#endif /* USART2_ENABLE */

/**
  * @brief  Initialize and configure LC sensor demo
  *         Can be tuned by user and must be called before LcSensorDemo() function
  * @param  None
  * @retval None
  */
void LcSensorConfig(void)
{
  /* Lc sensor configuration */
  LcConfig.Mode = LC_SENSOR_LOW_POWER;          /* LC sensor mode: LC_SENSOR_LOW_POWER or LC_SENSOR_STD for debug */
  LcConfig.Sampling = 100;                      /* Lc measurement sampling - Value in Hz (up to 500Hz)*/
  LcConfig.TimeBetweenSensor = 75;              /* Waiting time between 2 sensors measurements (used only if more than 1 sensor is enabled)- TimeBetweenSensor = (TimeBetweenSensor_seconds * Freq_MSI /64) */
  LcConfig.DacVmid = 0x800;                     /* DacVmid=(4096*Vmid)/Vdd */
  LcConfig.DacVcmp = 0xC00;                     /* DacVcmp=(4096*Vcmp)/Vdd - Updated if first calibration enabled*/
  LcConfig.DacVmidRefreshTime = 0;              /* 0 to disable Waiting time */
  /* Dac Vmid refresh time before measurement depends to sampling value and Cref - DacVmidRefreshTime = (DacVmidRefreshTime_seconds * FreqMSI / 64)
  Examples for 10mv Accuracy and 1 sensor:
  If Sampling = 500   => Voltage drop during hold phase inferior to accuracy so more waiting time not required => DacVmidRefreshTime = 0
  If Sampling = 100   => Voltage drop during hold phase inferior to accuracy so more waiting time not required => DacVmidRefreshTime = 0
  If Sampling = 32    => DacVmidRefreshTime_seconds = 656us   => DacVmidRefreshTime = 246
  If Sampling = 10    => DacVmidRefreshTime_seconds = 1.75ms  => DacVmidRefreshTime = 656
  */

  /* First calibration configuration */
  LcConfig.FirstCal.Status = CAL_DISABLED;      /* enable or disable first calibration CAL_ENABLED or CAL_DISABLED*/
  LcConfig.FirstCal.Mode = DYNAMIC;             /* STATIC (air) or DYNAMIC (with rotating wheel and transitions metal and no_metal) */
  LcConfig.FirstCal.Measures = 25;              /* Minimum measurements number to perform during calibration phase */
  LcConfig.FirstCal.DacVcmpMax = 0xE00;         /* Starting value for Vcmp calibration - DacStart=(4096*Vcmp)/Vdd */
  LcConfig.FirstCal.DacVcmpMin = 0x850;         /* Stop value for Vcmp calibration -DacStop=(4096*Vcmp)/Vdd */
  LcConfig.FirstCal.DacVcmpStep = 10;           /* Step value for Vcmp calibration -DacStep=(4096*Vcmp)/Vdd */
  LcConfig.FirstCal.PulseCount = 20;            /* Pulses number targeted - used for STATIC calibration */
  LcConfig.FirstCal.PulseCountDeltaMin = 20;    /* Set the difference to guarantee between min an max pulse count - used for DYNAMIC calibration */

  /* Periodic calibration configuration */
  LcConfig.PeriodicCal.Status = CAL_DISABLED;   /* enable or disable periodic calibration: CAL_ENABLED or CAL_DISABLED*/
  LcConfig.PeriodicCal.Mode = DYNAMIC;          /* STATIC or DYNAMIC (with rotating wheel) */
  LcConfig.PeriodicCal.CountDrift = 5;          /* Valid periodic calibration values only if the drift does not exceed this value*/

#ifdef SENSOR_1_ENABLED
  /* Lc sensor 1 configuration */
  LcConfigSensor1.Ls = 470e-6;                  /* Ls value in H - ie:470e-6*/
  LcConfigSensor1.Cs = 220e-12;                 /* Cs value in F - ie:220e-12*/
  LcConfigSensor1.CountDetectPercent = 0.8;     /* Detection threshold in percent - ie: 0.8 - not used if Dynamic calibration enabled */
  LcConfigSensor1.Texcit = 2;                   /* Excitation time - Texcit=(Texcit_sec*Freq_MSI)/3 */
  LcConfigSensor1.Tcapture = 19;                /* Capture time - Tcapture=(Tcapture_sec+~40us)/(Freq_MSI/64) - Value updated if calibration enabled */
#endif /* SENSOR_1_ENABLED */

#ifdef SENSOR_2_ENABLED
  /* Lc sensor 2 configuration */
  LcConfigSensor2.Ls = 470e-6;
  LcConfigSensor2.Cs = 220e-12;
  LcConfigSensor2.CountDetectPercent = 0.8;
  LcConfigSensor2.Texcit = 2;
  LcConfigSensor2.Tcapture = 19;
#endif /* SENSOR_2_ENABLED */

#ifdef SENSOR_3_ENABLED
  /* Lc sensor 3 configuration */
  LcConfigSensor3.Ls = 470e-6;
  LcConfigSensor3.Cs = 220e-12;
  LcConfigSensor3.CountDetectPercent = 0.8;
  LcConfigSensor3.Texcit = 2;
  LcConfigSensor3.Tcapture = 19;
#endif /* SENSOR_3_ENABLED */

#ifdef SENSOR_4_ENABLED
  /* Lc sensor 4 configuration */
  LcConfigSensor4.Ls = 470e-6;
  LcConfigSensor4.Cs = 220e-12;
  LcConfigSensor4.CountDetectPercent = 0.8;
  LcConfigSensor4.Texcit = 2;
  LcConfigSensor4.Tcapture = 19;
#endif /* SENSOR_4_ENABLED */
}

/**
  * @brief  Initialize all variables used in application and used in RTC WakeUp interrupt to have a short time for registers modifications (to minimize instructions during measurements)
  *         Mandatory: Must be called before RTC WakeUp enabling (HAL_RTCEx_SetWakeUpTimer_IT function) to keep context
  * @param  None
  * @retval None
  */
void LcSensorInit(void)
{
  /* Initialize Lc status variables */
  LcStatus.MeasuresCount = 0;                   /* Reset measures counter to 0 */
  LcStatus.EdgeCount = 0;                       /* Reset edge counter to 0 */
  LcStatus.EdgeCountPos = 0;                    /* Reset positive edge counter to 0 */
  LcStatus.EdgeCountNeg = 0;                    /* Reset negative edge counter to 0 */
  LcStatus.Errors = 0;                          /* Reset errors counter to 0 */
  LcStatus.EdgeCountPos2 = 0;                   /* Reset positive edge counter to 0 */
  LcStatus.EdgeCountNeg2 = 0;                   /* Reset negative edge counter to 0 */
  LcStatus.Errors2 = 0;                         /* Reset errors counter to 0 */
  LcStatus.Rpm = 0;                             /* Reset Rotations Per Minutes to 0 */
  LcStatus.Rps = 0;                             /* Reset Rotations Per Seconds to 0 */
  LcStatus.WakeUpCounter = (uint32_t) (round(((1.0 / LcConfig.Sampling) / (2 / 32.768e3))) - 1); /* WakeUpCounter - LSE = 32.768e3Hz and RTC_PreScaler = 2 */
  LcStatus.Sampling = (1.0 / ((LcStatus.WakeUpCounter + 1) * (2.0 / 32.768e3))); /* The real sampling value is stored */

  /* Initialize LcSensor status */
  LcSensor1.Status = NO_METAL;                  /* Variable used to store sensor status - 0 = NO_METAL , 1 = METAL*/
  LcSensor1.PreviousStatus = NO_METAL;          /* Variable used to store previous sensor status (for transition detection)- 0 = NO_METAL , 1 = METAL*/
  LcSensor2.Status = NO_METAL;                  /* Variable used to store sensor status - 0 = NO_METAL , 1 = METAL*/
  LcSensor2.PreviousStatus = NO_METAL;          /* Variable used to store previous sensor status (for transition detection)- 0 = NO_METAL , 1 = METAL*/
  LcSensor3.Status = NO_METAL;                  /* Variable used to store sensor status - 0 = NO_METAL , 1 = METAL*/
  LcSensor3.PreviousStatus = NO_METAL;          /* Variable used to store previous sensor status (for transition detection)- 0 = NO_METAL , 1 = METAL*/
  LcSensor4.Status = NO_METAL;                  /* Variable used to store sensor status - 0 = NO_METAL , 1 = METAL*/
  LcSensor4.PreviousStatus = NO_METAL;          /* Variable used to store previous sensor status (for transition detection)- 0 = NO_METAL , 1 = METAL*/

  /* Configure LC IO Direction mode (Output or Analog) for fast modifications */
  LcIO.IO1_GPIO_GROUP = (uint32_t)(&GPIO_LC_IO1->MODER);
  LcIO.IO1_OUTPUT_PP = GPIO_LC_IO1->MODER;
  LcIO.IO1_OUTPUT_PP &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO1_PIN_NUMBER * 2));
  LcIO.IO1_OUTPUT_PP |= ((GPIO_MODE_OUTPUT_PP) << (GPIO_LC_IO1_PIN_NUMBER * 2));
  LcIO.IO1_ANALOG = GPIO_LC_IO1->MODER;
  LcIO.IO1_ANALOG &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO1_PIN_NUMBER * 2));
  LcIO.IO1_ANALOG |= ((GPIO_MODE_ANALOG) << (GPIO_LC_IO1_PIN_NUMBER * 2));

  LcIO.IO2_GPIO_GROUP = (uint32_t)(&GPIO_LC_IO2->MODER);
  LcIO.IO2_OUTPUT_PP = GPIO_LC_IO2->MODER;
  LcIO.IO2_OUTPUT_PP &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO2_PIN_NUMBER * 2));
  LcIO.IO2_OUTPUT_PP |= ((GPIO_MODE_OUTPUT_PP) << (GPIO_LC_IO2_PIN_NUMBER * 2));
  LcIO.IO2_ANALOG = GPIO_LC_IO2->MODER;
  LcIO.IO2_ANALOG &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO2_PIN_NUMBER * 2));
  LcIO.IO2_ANALOG |= ((GPIO_MODE_ANALOG) << (GPIO_LC_IO2_PIN_NUMBER * 2));

  LcIO.IO3_GPIO_GROUP = (uint32_t)(&GPIO_LC_IO3->MODER);
  LcIO.IO3_OUTPUT_PP = GPIO_LC_IO3->MODER;
  LcIO.IO3_OUTPUT_PP &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO3_PIN_NUMBER * 2));
  LcIO.IO3_OUTPUT_PP |= ((GPIO_MODE_OUTPUT_PP) << (GPIO_LC_IO3_PIN_NUMBER * 2));
  LcIO.IO3_ANALOG = GPIO_LC_IO3->MODER;
  LcIO.IO3_ANALOG &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO3_PIN_NUMBER * 2));
  LcIO.IO3_ANALOG |= ((GPIO_MODE_ANALOG) << (GPIO_LC_IO3_PIN_NUMBER * 2));

  LcIO.IO4_GPIO_GROUP = (uint32_t)(&GPIO_LC_IO4->MODER);
  LcIO.IO4_OUTPUT_PP = GPIO_LC_IO4->MODER;
  LcIO.IO4_OUTPUT_PP &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO4_PIN_NUMBER * 2));
  LcIO.IO4_OUTPUT_PP |= ((GPIO_MODE_OUTPUT_PP) << (GPIO_LC_IO4_PIN_NUMBER * 2));
  LcIO.IO4_ANALOG = GPIO_LC_IO4->MODER;
  LcIO.IO4_ANALOG &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO4_PIN_NUMBER * 2));
  LcIO.IO4_ANALOG |= ((GPIO_MODE_ANALOG) << (GPIO_LC_IO4_PIN_NUMBER * 2));

  LcIO.IO5_GPIO_GROUP = (uint32_t)(&GPIO_LC_IO5->MODER);
  LcIO.IO5_OUTPUT_PP = GPIO_LC_IO5->MODER;
  LcIO.IO5_OUTPUT_PP &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO5_PIN_NUMBER * 2));
  LcIO.IO5_OUTPUT_PP |= ((GPIO_MODE_OUTPUT_PP) << (GPIO_LC_IO5_PIN_NUMBER * 2));
  LcIO.IO5_ANALOG = GPIO_LC_IO5->MODER;
  LcIO.IO5_ANALOG &= ~((GPIO_MODER_MODE0) << (GPIO_LC_IO5_PIN_NUMBER * 2));
  LcIO.IO5_ANALOG |= ((GPIO_MODE_ANALOG) << (GPIO_LC_IO5_PIN_NUMBER * 2));
}

/**
  * @brief  Start LC sensor demo
  * @param  None
  * @retval None
  */
void LcSensorDemo(void)
{
  /* GPIO Init */
  LC_Init_GPIO();

  /* System clock configuration */
  SystemClock_ConfigMSI_24M();

  /* Initializes RTC */
  LC_Init_RTC();

  /* Initializes DAC as comparator threshold voltage for VCOMP and VMID = VDD/2 */
  LC_Init_DAC1();

  /* Initialize peripherals used by LC sensors */
#if defined(SENSOR_1_ENABLED) || defined(SENSOR_2_ENABLED)
  /* Initializes COMP1 as pulses detector */
  LC_Init_COMP1();

  /* Initializes LPTIM1 as LC sensor pulses counter */
  LC_Init_LPTIM1();
#endif /* SENSOR_1_ENABLED or SENSOR_2_ENABLED */

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
  /* Initializes COMP2 as pulses detector */
  Init_COMP2_LcSensor();

  /* Initializes LPTIM2 as LC sensor pulses counter */
  Init_LPTIM2_LcSensor();
#endif /* SENSOR_3_ENABLED or SENSOR_4_ENABLED */

  /* Initializes TIM6 as one pulse timer to be in Sleep mode while the LPTIM is counting */
  LC_Init_TIM6();

#ifdef COMP1_OUT_ENABLED
  /* Enable PB0 Comparator output for debug only */
  GPIOB->MODER &= 0xFFFFFFFE;
#else /* not COMP1_OUT_ENABLED */
  /* Disable PB0 Comparator output */
  GPIOB->MODER |= 0x00000003;
#endif /* COMP1_OUT_ENABLED */

#ifdef COMP2_OUT_ENABLED
  /* Enable PB11 Comparator output for debug only */
  GPIOB->MODER &= 0xFFBFFFFF;
#else /* COMP2_OUT_ENABLED */
  /* Disable PB0 Comparator output */
  GPIOB->MODER |= 0x00C00000;
#endif /* COMP2_OUT_ENABLED */

  /* WFE to wake up when an interrupt moves from inactive to pended */
  HAL_PWR_EnableSEVOnPend();

  /* Suspend system tick */
  HAL_SuspendTick();

  /* Initialize LC variables */
  LcSensorInit();

  /* Perform first calibration */
  LC_Calibration();

#if (USART2_ENABLE == 1)
  /* Enable periodic USART2 communication to send info to user or to other peripheral */
  LC_Init_USART2();
  LC_Init_AlarmA();
#endif /* USART2_ENABLE */

  if (LcConfig.PeriodicCal.Status == CAL_ENABLED)
  {
    /* Enable periodic calibration managed by RTC ALARM_B */
    LC_init_AlarmB();
    LcStatus.CalStatus.State = CAL_DONE;
  }

  /* Set and start WakeUp timer */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&Hrtc, LcStatus.WakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
  {
    /* SetWakeUpTimer Error */
    ErrorHandler();
  }

  /* Enter the application in STOP mode */
  if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
  {
    StopEntry();
  }

  /* Infinite loop */
  while (1)
  {}
}

/**
  * @brief  Compute capture time
  * @param  count_max: maximum oscillation pulse count
  *         freq_osc: LC oscillation frequency
  * @retval Capture time
  */
static uint32_t LC_Compute_Tcapture(uint32_t count_max, double_t freq_osc)
{
  /* Tcapture = (CountMax / FreqOsc + margin_seconds) * MSI_Frequency / DivFactor */
  return (uint32_t)((((count_max / freq_osc) * 1.05) * HAL_RCC_GetSysClockFreq()) / 64);
}

/**
  * @brief  Compute oscillation count detect threshold
  * @param  count_max: maximum oscillation pulse count
  *         count_min: minimum oscillation pulse count
  * @retval Count Detect value
  */
static uint32_t LC_Compute_CountDetect(uint32_t count_max, uint32_t count_min)
{
  return (uint32_t)((count_max + count_min) / 2.0);
}

/**
  * @brief  Compute oscillation count detect threshold
  * @param  count_max: maximum oscillation pulse count
  *         percent: Percent value to compute detection threshold
  * @retval Count Detect value in percent
  */
static uint32_t LC_Compute_CountDetectPer(uint32_t count_max, double_t percent)
{
  return (uint32_t)(count_max * percent);
}

/**
  * @brief  Allows to perform first LC sensors calibration
  *        STATIC calibration => to perform with AIR (witout metal)
  *        DYNAMIC calibration => need a rotating wheel (Metal/No Metal states)
  * @param  None
  * @retval None
  */
static void LC_Calibration(void)
{
  static int32_t vcmp_tmp = 0;
  static int32_t vcmp_start = 0;
  static int32_t vcmp_stop = 0;
  static int32_t vcmp_step = 0;
  static uint32_t loop = 0;
  static double_t M_PI_VALUE = 3.14159f;          /* PI value */

  /* Update calibration status */
  LcStatus.CalStatus.State = CAL_ON_GOING;

  /* Compute theoric LC oscillations frequency for each sensor
  FREQ_LC_OSC = 1/(2*Pi*sqrt(Ls*Cs))
  */
  LcConfigSensor1.FreqOsc = 1 / (2 * M_PI_VALUE * sqrt(LcConfigSensor1.Ls * LcConfigSensor1.Cs));
  LcConfigSensor2.FreqOsc = 1 / (2 * M_PI_VALUE * sqrt(LcConfigSensor2.Ls * LcConfigSensor2.Cs));
  LcConfigSensor3.FreqOsc = 1 / (2 * M_PI_VALUE * sqrt(LcConfigSensor3.Ls * LcConfigSensor3.Cs));
  LcConfigSensor4.FreqOsc = 1 / (2 * M_PI_VALUE * sqrt(LcConfigSensor4.Ls * LcConfigSensor4.Cs));

  loop = LcConfig.FirstCal.Measures;

  if (LcConfig.FirstCal.Status == CAL_ENABLED)
  {
    /* Time capture window increased to catch maximum of osccilations */
    LcConfigSensor1.Tcapture = 200;
    LcConfigSensor2.Tcapture = 200;
    LcConfigSensor3.Tcapture = 200;
    LcConfigSensor4.Tcapture = 200;

    if (LcConfig.FirstCal.Mode == STATIC)
    {
      /* Calibration enabled - Vcmp and CountMax values updated */
      /* Initialize search limits for Vcmp values*/
      if ((LcConfig.FirstCal.DacVcmpMin < 0x800) && (LcConfig.FirstCal.DacVcmpMax < 0x800))
      {
        vcmp_start = (int32_t) LcConfig.FirstCal.DacVcmpMax;
        vcmp_stop = (int32_t) LcConfig.FirstCal.DacVcmpMin;
        vcmp_step = (int32_t) - LcConfig.FirstCal.DacVcmpStep;
      }
      else if ((LcConfig.FirstCal.DacVcmpMin > 0x800) && (LcConfig.FirstCal.DacVcmpMax > 0x800))
      {
        vcmp_start = (int32_t) LcConfig.FirstCal.DacVcmpMin;
        vcmp_stop = (int32_t) LcConfig.FirstCal.DacVcmpMax;
        vcmp_step = (int32_t) LcConfig.FirstCal.DacVcmpStep;
      }
      else
      {
        /* Bad Dac Vcmp values used for calibration */
        ErrorHandler();
      }

      vcmp_tmp = vcmp_start;

      /* To search comparator threshold */
      do
      {
        /* Enable and set DAC1 */
        SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID | DAC_CR_LC_EN_VCMP);
        if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VCMP, DAC_ALIGN_12B_R, (uint32_t) vcmp_tmp) != HAL_OK)
        {
          /* DAC_SetValue Error */
          ErrorHandler();
        }

        /* Enable WakeUp timer to start measurements*/
        if (HAL_RTCEx_SetWakeUpTimer_IT(&Hrtc, LcStatus.WakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
        {
          /* SetWakeUpTimer Error */
          ErrorHandler();
        }

        /* Remove first measurements*/
        while (LcStatus.MeasuresCount <= loop)
        {}

        /* Initialize variables used for search step */
        LcConfig.DacVcmp = (uint32_t) (vcmp_tmp);
        LcStatus.MeasuresCount = 0;
#ifdef SENSOR_1_ENABLED
        LcSensor1.CountMax = 0;
        LcStatus.CalStatus.Sensor1CountMax = 0;
        LcStatus.CalStatus.Sensor1CountTransitions = 0;
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
        LcSensor2.CountMax = 0;
        LcStatus.CalStatus.Sensor2CountMax = 0;
        LcStatus.CalStatus.Sensor2CountTransitions = 0;
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
        LcSensor3.CountMax = 0;
        LcStatus.CalStatus.Sensor3CountMax = 0;
        LcStatus.CalStatus.Sensor3CountTransitions = 0;
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
        LcSensor4.CountMax = 0;
        LcStatus.CalStatus.Sensor4CountMax = 0;
        LcStatus.CalStatus.Sensor4CountTransitions = 0;
#endif /* SENSOR_4_ENABLED */

        /* Wait end of measurements */
        while (LcStatus.MeasuresCount <= loop)
        {}

        /* Disable WakeUp timer to stop measurements*/
        if (HAL_RTCEx_DeactivateWakeUpTimer(&Hrtc) != HAL_OK)
        {
          /* DesactivateWakeUpTimer Error */
          ErrorHandler();
        }

        /* Store calibration results */
        LcSensor1.CountMax = LcStatus.CalStatus.Sensor1CountMax;
        LcSensor2.CountMax = LcStatus.CalStatus.Sensor2CountMax;
        LcSensor3.CountMax = LcStatus.CalStatus.Sensor3CountMax;
        LcSensor4.CountMax = LcStatus.CalStatus.Sensor4CountMax;

        /* Break if count max <= expected pulse count */
#ifdef SENSOR_1_ENABLED
        if (LcSensor1.CountMax <= LcConfig.FirstCal.PulseCount)
        {
          break;
        }
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
        if (LcSensor2.CountMax <= LcConfig.FirstCal.PulseCount)
        {
          break;
        }
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
        if (LcSensor3.CountMax <= LcConfig.FirstCal.PulseCount)
        {
          break;
        }
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
        /* Break if count max <= expected pulse count */
        if (LcSensor4.CountMax <= LcConfig.FirstCal.PulseCount)
        {
          break;
        }
#endif /* SENSOR_4_ENABLED */

        /* Increment Vcmp value */
        vcmp_tmp += vcmp_step;

        /* Detect if Vcmp limit has been reached */
        if (vcmp_tmp > vcmp_stop)
        {
          LcStatus.CalStatus.State = CAL_FAILED;
          /* First Vcmp calibration Error */
          ErrorHandler();
        }
      }
      while (1);

      /* Restore the last working Vcmp value */
      LcConfig.DacVcmp = (uint32_t) (vcmp_tmp);

      /* Set DAC with last Vcmp value working */
      if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VCMP, DAC_ALIGN_12B_R, LcConfig.DacVcmp) != HAL_OK)
      {
        LcStatus.CalStatus.State = CAL_FAILED;
        /* DAC_SetValue Error */
        ErrorHandler();
      }

      /* Disable WakeUp timer to stop measurements*/
      if (HAL_RTCEx_DeactivateWakeUpTimer(&Hrtc) != HAL_OK)
      {
        /* DesactivateWakeUpTimer Error */
        ErrorHandler();
      }

      /* Update count detect number threshold */
      LcSensor1.CountDetect = LC_Compute_CountDetectPer(LcSensor1.CountMax, LcConfigSensor1.CountDetectPercent);
      LcSensor2.CountDetect = LC_Compute_CountDetectPer(LcSensor2.CountMax, LcConfigSensor2.CountDetectPercent);
      LcSensor3.CountDetect = LC_Compute_CountDetectPer(LcSensor3.CountMax, LcConfigSensor3.CountDetectPercent);
      LcSensor4.CountDetect = LC_Compute_CountDetectPer(LcSensor4.CountMax, LcConfigSensor4.CountDetectPercent);
    }

    else if (LcConfig.FirstCal.Mode == DYNAMIC)
    {
      /* Calibration enabled - Vcmp, CountMax and CountMin values updated */
      /* Enable DAC */
      SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID | DAC_CR_LC_EN_VCMP);

      /* Initialize search limits for Vcmp values*/
      if ((LcConfig.FirstCal.DacVcmpMin < 0x800) && (LcConfig.FirstCal.DacVcmpMax < 0x800))
      {
        vcmp_start = (int32_t) LcConfig.FirstCal.DacVcmpMax;
        vcmp_stop = (int32_t) LcConfig.FirstCal.DacVcmpMin;
        vcmp_step = (int32_t) - LcConfig.FirstCal.DacVcmpStep;
      }
      else if ((LcConfig.FirstCal.DacVcmpMin > 0x800) && (LcConfig.FirstCal.DacVcmpMax > 0x800))
      {
        vcmp_start = (int32_t) LcConfig.FirstCal.DacVcmpMin;
        vcmp_stop = (int32_t) LcConfig.FirstCal.DacVcmpMax;
        vcmp_step = (int32_t) LcConfig.FirstCal.DacVcmpStep;
      }
      else
      {
        /* Bad Dac Vcmp values used for calibration */
        ErrorHandler();
      }

      vcmp_tmp = vcmp_start;

      /* To search comparator threshold */
      do
      {
        if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VCMP, DAC_ALIGN_12B_R, (uint32_t) vcmp_tmp) != HAL_OK)
        {
          /* DAC_SetValue Error */
          ErrorHandler();
        }

        /* Initialize variables used for search step */
        LcConfig.DacVcmp = (uint32_t) vcmp_tmp;

        /* Enable WakeUp timer to start measurements*/
        if (HAL_RTCEx_SetWakeUpTimer_IT(&Hrtc, LcStatus.WakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
        {
          /* SetWakeUpTimer Error */
          ErrorHandler();
        }

        /* Remove first measurements*/
        while (LcStatus.MeasuresCount <= loop)
        {}

        /* Initialize all variables */
        LcSensor1.CountMax = 0;
        LcSensor2.CountMax = 0;
        LcSensor3.CountMax = 0;
        LcSensor4.CountMax = 0;
        LcSensor1.CountMin = 0xFFFF;
        LcSensor2.CountMin = 0xFFFF;
        LcSensor3.CountMin = 0xFFFF;
        LcSensor4.CountMin = 0xFFFF;
        LcStatus.CalStatus.Sensor1CountMax = 0;
        LcStatus.CalStatus.Sensor2CountMax = 0;
        LcStatus.CalStatus.Sensor3CountMax = 0;
        LcStatus.CalStatus.Sensor4CountMax = 0;
        LcStatus.CalStatus.Sensor1CountMin = 0xFFFF;
        LcStatus.CalStatus.Sensor2CountMin = 0xFFFF;
        LcStatus.CalStatus.Sensor3CountMin = 0xFFFF;
        LcStatus.CalStatus.Sensor4CountMin = 0xFFFF;

        LcStatus.MeasuresCount = 0;
        LcStatus.CalStatus.Sensor1CountTransitions = 0;
        LcStatus.CalStatus.Sensor2CountTransitions = 0;
        LcStatus.CalStatus.Sensor3CountTransitions = 0;
        LcStatus.CalStatus.Sensor4CountTransitions = 0;

        /* Wait end of measurements */
        while (LcStatus.MeasuresCount <= loop)
        {}

        /* Disable WakeUp timer to stop measurements*/
        if (HAL_RTCEx_DeactivateWakeUpTimer(&Hrtc) != HAL_OK)
        {
          /* DesactivateWakeUpTimer Error */
          ErrorHandler();
        }

        /* Store calibration results */
        LcSensor1.CountMax = LcStatus.CalStatus.Sensor1CountMax;
        LcSensor1.CountMin = LcStatus.CalStatus.Sensor1CountMin;
        LcSensor2.CountMax = LcStatus.CalStatus.Sensor2CountMax;
        LcSensor2.CountMin = LcStatus.CalStatus.Sensor2CountMin;
        LcSensor3.CountMax = LcStatus.CalStatus.Sensor3CountMax;
        LcSensor3.CountMin = LcStatus.CalStatus.Sensor3CountMin;
        LcSensor4.CountMax = LcStatus.CalStatus.Sensor4CountMax;
        LcSensor4.CountMin = LcStatus.CalStatus.Sensor4CountMin;

        /* Delta between min and max count number checking */
        if ((LcSensor1.CountMax - LcSensor1.CountMin) <= LcConfig.FirstCal.PulseCountDeltaMin)
        {
          break;
        }
        else if ((LcSensor2.CountMax - LcSensor2.CountMin) <= LcConfig.FirstCal.PulseCountDeltaMin)
        {
          break;
        }
        else if ((LcSensor3.CountMax - LcSensor3.CountMin) <= LcConfig.FirstCal.PulseCountDeltaMin)
        {
          break;
        }
        else if ((LcSensor4.CountMax - LcSensor4.CountMin) <= LcConfig.FirstCal.PulseCountDeltaMin)
        {
          break;
        }

        /* Increment Vcmp value */
        vcmp_tmp += vcmp_step;

        /* Detect if Vcmp limit has been reached */
        if (vcmp_tmp > vcmp_stop)
        {
          LcStatus.CalStatus.State = CAL_FAILED;
          /* First Vcmp calibration Error */
          ErrorHandler();
        }
      }
      while (1);

      /* Recall the last Vcmp working */
      LcConfig.DacVcmp = (uint32_t) (vcmp_tmp);

      /* Set DAC with last Vcmp value working */
      if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VCMP, DAC_ALIGN_12B_R, LcConfig.DacVcmp) != HAL_OK)
      {
        /* DAC_SetValue Error */
        ErrorHandler();
      }

      /* Compute Count_detect based on CountMin and CountMax values */
      LcSensor1.CountDetect = LC_Compute_CountDetect(LcSensor1.CountMax, LcSensor1.CountMin);
      LcSensor2.CountDetect = LC_Compute_CountDetect(LcSensor2.CountMax, LcSensor2.CountMin);
      LcSensor3.CountDetect = LC_Compute_CountDetect(LcSensor3.CountMax, LcSensor3.CountMin);
      LcSensor4.CountDetect = LC_Compute_CountDetect(LcSensor4.CountMax, LcSensor4.CountMin);
    }


    /* Define capture time to minimize the time capture window and the power consumption */
    LcConfigSensor1.Tcapture = LC_Compute_Tcapture(LcSensor1.CountMax, LcConfigSensor1.FreqOsc);
    LcConfigSensor2.Tcapture = LC_Compute_Tcapture(LcSensor2.CountMax, LcConfigSensor2.FreqOsc);
    LcConfigSensor3.Tcapture = LC_Compute_Tcapture(LcSensor3.CountMax, LcConfigSensor3.FreqOsc);
    LcConfigSensor4.Tcapture = LC_Compute_Tcapture(LcSensor4.CountMax, LcConfigSensor4.FreqOsc);
  }
  else
  {
    /* Calibration disabled - CountMax and CountMin values updated */
    /* Enable WakeUpTimer for calibration */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&Hrtc, LcStatus.WakeUpCounter, RTC_WAKEUPCLOCK_RTCCLK_DIV2) != HAL_OK)
    {
      /* SetWakeUpTimer Error */
      ErrorHandler();
    }

    /* Remove first measurements */
    while (LcStatus.MeasuresCount <= loop)
    {}

    /* Initialize all variables */
    LcSensor1.CountMax = 0;
    LcSensor2.CountMax = 0;
    LcSensor3.CountMax = 0;
    LcSensor4.CountMax = 0;
    LcSensor1.CountMin = 0xFFFF;
    LcSensor2.CountMin = 0xFFFF;
    LcSensor3.CountMin = 0xFFFF;
    LcSensor4.CountMin = 0xFFFF;
    LcStatus.CalStatus.Sensor1CountMax = 0;
    LcStatus.CalStatus.Sensor2CountMax = 0;
    LcStatus.CalStatus.Sensor3CountMax = 0;
    LcStatus.CalStatus.Sensor4CountMax = 0;
    LcStatus.CalStatus.Sensor1CountMin = 0xFFFF;
    LcStatus.CalStatus.Sensor2CountMin = 0xFFFF;
    LcStatus.CalStatus.Sensor3CountMin = 0xFFFF;
    LcStatus.CalStatus.Sensor4CountMin = 0xFFFF;

    LcStatus.MeasuresCount = 0;
    LcStatus.CalStatus.Sensor1CountTransitions = 0;
    LcStatus.CalStatus.Sensor2CountTransitions = 0;
    LcStatus.CalStatus.Sensor3CountTransitions = 0;
    LcStatus.CalStatus.Sensor4CountTransitions = 0;

    /* Wait end of measurements */
    while (LcStatus.MeasuresCount <= loop)
    {}

    /* Disable WAkeUpTimer */
    if (HAL_RTCEx_DeactivateWakeUpTimer(&Hrtc) != HAL_OK)
    {
      /* DesactivateWakeUpTimer Error */
      ErrorHandler();
    }

    /* Store calibration results */
    LcSensor1.CountMax = LcStatus.CalStatus.Sensor1CountMax;
    LcSensor1.CountMin = LcStatus.CalStatus.Sensor1CountMin;
    LcSensor2.CountMax = LcStatus.CalStatus.Sensor2CountMax;
    LcSensor2.CountMin = LcStatus.CalStatus.Sensor2CountMin;
    LcSensor3.CountMax = LcStatus.CalStatus.Sensor3CountMax;
    LcSensor3.CountMin = LcStatus.CalStatus.Sensor3CountMin;
    LcSensor4.CountMax = LcStatus.CalStatus.Sensor4CountMax;
    LcSensor4.CountMin = LcStatus.CalStatus.Sensor4CountMin;

    /* Update count detect number threshold */
    LcSensor1.CountDetect = LC_Compute_CountDetectPer(LcSensor1.CountMax, LcConfigSensor1.CountDetectPercent);
    LcSensor2.CountDetect = LC_Compute_CountDetectPer(LcSensor2.CountMax, LcConfigSensor2.CountDetectPercent);
    LcSensor3.CountDetect = LC_Compute_CountDetectPer(LcSensor3.CountMax, LcConfigSensor3.CountDetectPercent);
    LcSensor4.CountDetect = LC_Compute_CountDetectPer(LcSensor4.CountMax, LcConfigSensor4.CountDetectPercent);
  }

  /* Initialize Lc variables */
  LcStatus.MeasuresCount = 0;
  LcStatus.EdgeCount = 0;
  LcStatus.EdgeCountPos = 0;
  LcStatus.EdgeCountNeg = 0;
  LcStatus.Errors = 0;
  LcStatus.Rpm = 0;
  LcStatus.Rps = 0;

  /* Update calibration status */
  LcStatus.CalStatus.State = CAL_DONE;
}

/**
  * @brief  Input Tamper1 callback
  *          Used to enable or disable USART2 communication
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc)
{
  if (UserButtonStatus == 0)
  {
    UserButtonStatus = 1;

#if (USART2_ENABLE == 1)
    /* Enable AlarmA - USART2 communication */
    LC_Init_AlarmA();

    if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
    {
      /* Enable GPIOs clock */
      __HAL_RCC_GPIOA_CLK_ENABLE();

      /* Restore GPIOA for USART communication */
      GPIOA->MODER &= 0xEBFFFFAF;
    }

    /* Transmit message */
    sprintf((char*)aMsgBuffer, "USART2 Enabled\n\r\n");
    HAL_UART_Transmit(&Huart2, aMsgBuffer, TXBUFFERSIZE(aMsgBuffer), 0xFFFF);
    memset((char*)aMsgBuffer, 0, TXBUFFERSIZE(aMsgBuffer));

    if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
    {
      /* Restore GPIOA config for LC sensor*/
      GPIOA->MODER |= 0x00000050;

      /* Disable GPIOs clock */
      __HAL_RCC_GPIOA_CLK_DISABLE();
    }
#endif /* USART2_ENABLE */
  }
  else
  {
    UserButtonStatus = 0;

#if (USART2_ENABLE == 1)
    if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
    {
      /* Enable GPIOs clock */
      __HAL_RCC_GPIOA_CLK_ENABLE();

      /* Restore GPIOA for USART communication */
      GPIOA->MODER &= 0xEBFFFFAF;
    }

    /* Transmit message */
    sprintf((char*)aMsgBuffer, "USART2 Disabled\n\r\n");
    HAL_UART_Transmit(&Huart2, aMsgBuffer, TXBUFFERSIZE(aMsgBuffer), 0xFFFF);
    memset((char*)aMsgBuffer, 0, TXBUFFERSIZE(aMsgBuffer));

    if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
    {
      /* Restore GPIOA config for LC sensor*/
      GPIOA->MODER |= 0x00000050;

      /* Disable GPIOs clock */
      __HAL_RCC_GPIOA_CLK_DISABLE();
    }

    /* Disable AlarmA - USART2 communication */
    HAL_RTC_DeactivateAlarm(hrtc, RTC_ALARM_A);
#endif /* USART2_ENABLE */
  }
}

/**
  * @brief  Input AlarmA callback
  *         Used to send counters informations to user by USART2 / Could be used to communicate with others peripherals
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  RTC_AlarmTypeDef salarma;
  /* Get alarm configuration to rearm alarm */
  if (HAL_RTC_GetAlarm(hrtc, &salarma, RTC_ALARM_A, RTC_FORMAT_BIN) != HAL_OK)
  {
    ErrorHandler();
  }

  /* Update Alarm with new values - 2 seconds for example*/
  salarma.AlarmTime.Seconds += 2;
  if ((uint8_t) salarma.AlarmTime.Seconds >= 60)
  {
    salarma.AlarmTime.Seconds = (uint8_t) ((int8_t) salarma.AlarmTime.Seconds - 60);

  }

  /* Rearm alarm */
  if (HAL_RTC_SetAlarm_IT(hrtc, &salarma, RTC_FORMAT_BIN) != HAL_OK)
  {
    ErrorHandler();
  }

#if (USART2_ENABLE == 1)
  if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
  {
    /* Enable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Restore GPIOA for USART communication */
    GPIOA->MODER &= 0xEBFFFFAF;
  }

#if (LC_SENSOR_DEMO == 1)
  /* local variables from global volatile variables to avoid compilations and executions issues */
  uint8_t status;
  uint32_t edge_count;
  status = LcSensor1.Status;
  edge_count = LcStatus.EdgeCount;

  /* Fill buffer */
  sprintf((char*)aMsgBuffer, "S:%d\tE:%lu\r\n", status, edge_count);

#elif (LC_SENSOR_DEMO == 2)
  /* local variables from global volatiles variable to avoid compilations and executions issues */
  uint32_t edge_count_pos;
  uint32_t edge_count_neg;
  edge_count_pos = LcStatus.EdgeCountPos;
  edge_count_neg = LcStatus.EdgeCountNeg;

  /* Fill buffer */
  sprintf((char*)aMsgBuffer, "P:%lu\tN:%lu\r\n", edge_count_pos, edge_count_neg);

#elif (LC_SENSOR_DEMO == 3)
  /* local variables from global volatiles variable to avoid compilations and executions issues */
  uint32_t edge_count;
  uint32_t previous_edge_count;
  uint32_t measures_count;

  edge_count = LcStatus.EdgeCount;
  previous_edge_count = LcStatus.PreviousEdgeCount;
  measures_count = LcStatus.MeasuresCount;

  /* Compute RPS and RPM */
  LcStatus.Rps = ((edge_count - previous_edge_count) * LcStatus.Sampling);
  LcStatus.Rps /= measures_count * 4;
  LcStatus.Rpm = LcStatus.Rps * 60;

  /* Fill buffer */
  sprintf((char*)aMsgBuffer, "%0.0f\n\r", LcStatus.Rpm);


#elif (LC_SENSOR_DEMO == 4)
  /* local variables from global volatile variables to avoid compilations and executions issues */
  uint32_t edge_count_pos;
  uint32_t edge_count_neg;
  uint32_t edge_count_pos2;
  uint32_t edge_count_neg2;
  edge_count_pos = LcStatus.EdgeCountPos;
  edge_count_neg = LcStatus.EdgeCountNeg;
  edge_count_pos2 = LcStatus.EdgeCountPos2;
  edge_count_neg2 = LcStatus.EdgeCountNeg2;

  /* Fill buffer */
  sprintf((char*)aMsgBuffer, "P1:%lu\tN1:%lu\tP2:%lu\tN2:%lu\r\n", edge_count_pos, edge_count_neg, edge_count_pos2, edge_count_neg2);
#else
#error Unknown LC_SENSOR_DEMO
#endif /* LC_SENSOR_DEMO */

  /* USART2 Transmit */
  HAL_UART_Transmit(&Huart2, aMsgBuffer, TXBUFFERSIZE(aMsgBuffer), 0xFFFF);

  if (LcConfig.Mode == LC_SENSOR_LOW_POWER)
  {
    /* Restore GPIOA config for LC sensor*/
    GPIOA->MODER |= 0x00000050;

    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE();
  }
#endif /* USART2_ENABLE */

  /* Reset Counters */
  LcStatus.PreviousEdgeCount = LcStatus.EdgeCount;
  LcStatus.PreviousEdgeCount2 = LcStatus.EdgeCount2;
  LcStatus.MeasuresCount = 0;
}

/**
  * @brief  Input AlarmB callback
  *         Used to perform periodic calibration - Measurements are still working during calibration phase, so this calibration is performed in 2 steps:
  *             - prepare calibration state and rearm alarm for calibration (to let some time for measurements)
  *             - analyze results, apply calibration values if measurements are validated and rearm alarm for the next calibration
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
  RTC_TimeTypeDef  stimestructure;
  RTC_AlarmTypeDef salarmstructure;

  /* Get alarm configuration to rearm alarm */
  if (HAL_RTC_GetAlarm(hrtc, &salarmstructure, RTC_ALARM_B, RTC_FORMAT_BIN) != HAL_OK)
  {
    ErrorHandler();
  }

  /* First calibration step to prepare and let some time to perform periodic calibration */
  if (LcStatus.CalStatus.State == CAL_DONE)
  {
#ifdef SENSOR_1_ENABLED
    LcConfigSensor1.Tcapture = 150;
    LcStatus.CalStatus.Sensor1CountTransitions = 0;
    LcStatus.CalStatus.Sensor1CountMax = 0;
    LcStatus.CalStatus.Sensor1CountMin = 0xFFFF;
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
    LcConfigSensor2.Tcapture = 150;
    LcStatus.CalStatus.Sensor2CountTransitions = 0;
    LcStatus.CalStatus.Sensor2CountMax = 0;
    LcStatus.CalStatus.Sensor2CountMin = 0xFFFF;
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
    LcConfigSensor3.Tcapture = 150;
    LcStatus.CalStatus.Sensor3CountTransitions = 0;
    LcStatus.CalStatus.Sensor3CountMax = 0;
    LcStatus.CalStatus.Sensor3CountMin = 0xFFFF;
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
    LcConfigSensor4.Tcapture = 150;
    LcStatus.CalStatus.Sensor4CountTransitions = 0;
    LcStatus.CalStatus.Sensor4CountMax = 0;
    LcStatus.CalStatus.Sensor4CountMin = 0xFFFF;
#endif /* SENSOR_4_ENABLED */

    /* Rearm alarm to let some time to perform calibration measurements - 2 seconds for example*/
    HAL_RTC_GetTime(hrtc, &stimestructure, RTC_FORMAT_BIN);

    salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
    salarmstructure.AlarmTime.Seconds = (uint8_t) ((int8_t) stimestructure.Seconds + 2);
    if (salarmstructure.AlarmTime.Seconds >= 60)
    {
      salarmstructure.AlarmTime.Seconds = (uint8_t) ((int8_t) salarmstructure.AlarmTime.Seconds - 60);
    }

    LcStatus.CalStatus.State = CAL_ON_GOING;
  }
  /* Second calibration step to prepare and let some time to perform periodic calibration */
  else if (LcStatus.CalStatus.State == CAL_ON_GOING)
  {
    if (LcConfig.PeriodicCal.Mode == STATIC)
    {
#ifdef SENSOR_1_ENABLED
      /* Perform modifications only if no transitions have been detected  and status stay at NO_METAL*/
      if ((LcStatus.CalStatus.Sensor1CountTransitions == 0) && (LcSensor1.Status == NO_METAL))
      {
        /* Sensor 1 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */

        if (LcStatus.CalStatus.Sensor1CountMax > (LcSensor1.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor1CountMax < (LcSensor1.CountMax + LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax value */
          LcSensor1.CountMax = LcStatus.CalStatus.Sensor1CountMax;

          /* Compute new count threshold number */
          LcSensor1.CountDetect = LC_Compute_CountDetectPer(LcSensor1.CountMax, LcConfigSensor1.CountDetectPercent);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor1.Tcapture = LC_Compute_Tcapture(LcSensor1.CountMax, LcConfigSensor1.FreqOsc);
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
      /* Perform modifications only if no transitions have been detected */
      if (LcStatus.CalStatus.Sensor2CountTransitions == 0)
      {
        /* Sensor 2 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor2CountMax > (LcSensor2.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor2CountMax < (LcSensor2.CountMax + LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax value */
          LcSensor2.CountMax = LcStatus.CalStatus.Sensor2CountMax;

          /* Compute new count threshold number */
          LcSensor2.CountDetect = LC_Compute_CountDetectPer(LcSensor2.CountMax, LcConfigSensor2.CountDetectPercent);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor2.Tcapture = LC_Compute_Tcapture(LcSensor2.CountMax, LcConfigSensor2.FreqOsc);
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
      /* Perform modifications only if no transitions have been detected */
      if (LcStatus.CalStatus.Sensor3CountTransitions == 0)
      {
        /* Sensor 3 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor3CountMax > (LcSensor3.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor3CountMax < (LcSensor3.CountMax + LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax value */
          LcSensor3.CountMax = LcStatus.CalStatus.Sensor3CountMax;

          /* Compute new count threshold number */
          LcSensor3.CountDetect = LC_Compute_CountDetectPer(LcSensor3.CountMax, LcConfigSensor3.CountDetectPercent);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor3.Tcapture = LC_Compute_Tcapture(LcSensor3.CountMax, LcConfigSensor3.FreqOsc);
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
      /* Perform modifications only if no transitions have been detected */
      if (LcStatus.CalStatus.Sensor4CountTransitions == 0)
      {
        /* Sensor 4 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor4CountMax > (LcSensor4.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor4CountMax < (LcSensor4.CountMax + LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax value */
          LcSensor4.CountMax = LcStatus.CalStatus.Sensor4CountMax;

          /* Compute new count threshold number */
          LcSensor4.CountDetect = LC_Compute_CountDetectPer(LcSensor4.CountMax, LcConfigSensor4.CountDetectPercent);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor4.Tcapture = LC_Compute_Tcapture(LcSensor4.CountMax, LcConfigSensor4.FreqOsc);
#endif /* SENSOR_4_ENABLED */
    }
    else if (LcConfig.PeriodicCal.Mode == DYNAMIC)
    {
#ifdef SENSOR_1_ENABLED
      /* Perform modifications only if the wheel is in rotation ( 2 rotations min with 4 transitions) */
      if (LcStatus.CalStatus.Sensor1CountTransitions > 4)
      {
        /* Sensor 1 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor1CountMax < (LcSensor1.CountMax + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor1CountMax > (LcSensor1.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor1CountMin < (LcSensor1.CountMin + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor1CountMin > (LcSensor1.CountMin - LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax and CountMin values */
          LcSensor1.CountMax = LcStatus.CalStatus.Sensor1CountMax;
          LcSensor1.CountMin = LcStatus.CalStatus.Sensor1CountMin;

          /* Compute CountDetectPercent based on CountMin and CountMax values */
          LcSensor1.CountDetect = LC_Compute_CountDetect(LcSensor1.CountMax, LcSensor1.CountMin);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor1.Tcapture = LC_Compute_Tcapture(LcSensor1.CountMax, LcConfigSensor1.FreqOsc);
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
      /* Perform modifications only if the wheel is in rotation ( 2 rotations min with 4 transitions) */
      if (LcStatus.CalStatus.Sensor2CountTransitions > 4)
      {
        /* Sensor 2 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor2CountMax < (LcSensor2.CountMax + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor2CountMax > (LcSensor2.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor2CountMin < (LcSensor2.CountMin + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor2CountMin > (LcSensor2.CountMin - LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax and CountMin values */
          LcSensor2.CountMax = LcStatus.CalStatus.Sensor2CountMax;
          LcSensor2.CountMin = LcStatus.CalStatus.Sensor2CountMin;

          /* Compute CountDetectPercent based on CountMin and CountMax values */
          LcSensor2.CountDetect = LC_Compute_CountDetect(LcSensor2.CountMax, LcSensor2.CountMin);
        }

        /* Define capture time to minimize the time capture window and the power consumption */
        LcConfigSensor2.Tcapture = LC_Compute_Tcapture(LcSensor2.CountMax, LcConfigSensor2.FreqOsc);
      }
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
      /* Perform modifications only if the wheel is in rotation ( 2 rotations min with 4 transitions) */
      if (LcStatus.CalStatus.Sensor3CountTransitions > 4)
      {
        /* Sensor 3 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor3CountMax < (LcSensor3.CountMax + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor3CountMax > (LcSensor3.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor3CountMin < (LcSensor3.CountMin + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor3CountMin > (LcSensor3.CountMin - LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax and CountMin values */
          LcSensor3.CountMax = LcStatus.CalStatus.Sensor3CountMax;
          LcSensor3.CountMin = LcStatus.CalStatus.Sensor3CountMin;

          /* Compute CountDetectPercent based on CountMin and CountMax values */
          LcSensor3.CountDetect = LC_Compute_CountDetect(LcSensor3.CountMax, LcSensor3.CountMin);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor3.Tcapture = LC_Compute_Tcapture(LcSensor3.CountMax, LcConfigSensor3.FreqOsc);
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
      /* Perform modifications only if the wheel is in rotation ( 2 rotations min with 4 transitions) */
      if (LcStatus.CalStatus.Sensor4CountTransitions > 4)
      {
        /* Sensor 4 update if needed - if new value exceeds previous value +/- max drift value, new value is treated as noise */
        if (LcStatus.CalStatus.Sensor4CountMax < (LcSensor4.CountMax + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor4CountMax > (LcSensor4.CountMax - LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor4CountMin < (LcSensor4.CountMin + LcConfig.PeriodicCal.CountDrift) &&
            LcStatus.CalStatus.Sensor4CountMin > (LcSensor4.CountMin - LcConfig.PeriodicCal.CountDrift))
        {
          /* Update with new CountMax and CountMin values */
          LcSensor4.CountMax = LcStatus.CalStatus.Sensor4CountMax;
          LcSensor4.CountMin = LcStatus.CalStatus.Sensor4CountMin;

          /* Compute CountDetectPercent based on CountMin and CountMax values */
          LcSensor4.CountDetect = LC_Compute_CountDetect(LcSensor4.CountMax, LcSensor4.CountMin);
        }
      }
      /* Define capture time to minimize the time capture window and the power consumption */
      LcConfigSensor4.Tcapture = LC_Compute_Tcapture(LcSensor4.CountMax, LcConfigSensor4.FreqOsc);
#endif /* SENSOR_4_ENABLED */
    }
    else
    {
      /* Bad Calibration mode selected */
      ErrorHandler();
    }

    /* Get time */
    HAL_RTC_GetTime(hrtc, &stimestructure, RTC_FORMAT_BIN);

    /* Rearm alarm for the next periodic calibration - 5 minutes for example */
    salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_SECONDS;
    salarmstructure.AlarmTime.Minutes = (uint8_t) ((int8_t) stimestructure.Minutes + 5);
    if (salarmstructure.AlarmTime.Minutes >= 60)
    {
      salarmstructure.AlarmTime.Minutes = (uint8_t) ((int8_t) salarmstructure.AlarmTime.Minutes - 60);
    }

    LcStatus.CalStatus.State = CAL_DONE;
  }

  /* Rearm alarm */
  if (HAL_RTC_SetAlarm_IT(hrtc, &salarmstructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    ErrorHandler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
*/
static void ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {}
}

/**
  * @brief GPIO Init LC sensor
  * @retval None
  */
static void LC_Init_GPIO(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Allows to release NJTRST pin and to use PB4 pin for LC sensor*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF12_COMP2;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
                        | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
                        | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                        | GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure COMP1 & COMP2 non-inverting input pin configuration */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_LC_IO2_PIN;
  HAL_GPIO_Init(GPIO_LC_IO2, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_LC_IO3_PIN;
  HAL_GPIO_Init(GPIO_LC_IO3, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_LC_IO4_PIN;
  HAL_GPIO_Init(GPIO_LC_IO4, &GPIO_InitStruct);

#ifdef COMP1_OUT_ENABLED
  /* Configure COMP1 output pin configuration for debug*/
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF12_COMP1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif /* ENABLE_COMP_OUT */

#ifdef COMP2_OUT_ENABLED
  /* Configure COMP1 output pin configuration for debug*/
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF12_COMP2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif /* ENABLE_COMP_OUT */

#if (LC_EXCIT_MODE == 0)
  /* Reset LC_IOx to have the pin grounded when the I/O state changes from analog to output */
  GPIO_LC_IO2->BRR = (uint32_t)GPIO_LC_IO2_PIN;
  GPIO_LC_IO3->BRR = (uint32_t)GPIO_LC_IO3_PIN;
  GPIO_LC_IO4->BRR = (uint32_t)GPIO_LC_IO4_PIN;
  GPIO_LC_IO5->BRR = (uint32_t)GPIO_LC_IO5_PIN;
#elif (LC_EXCIT_MODE == 1)
  /* Set LC_IOx to have the pin to Vdd when the I/O state changes from analog to output */
  GPIO_LC_IO2->BSRR = (uint32_t)GPIO_LC_IO2_PIN;
  GPIO_LC_IO3->BSRR = (uint32_t)GPIO_LC_IO3_PIN;
  GPIO_LC_IO4->BSRR = (uint32_t)GPIO_LC_IO4_PIN;
  GPIO_LC_IO5->BSRR = (uint32_t)GPIO_LC_IO5_PIN;
#else
#error Unknown LC_EXCIT_MODE
#endif /* LC_EXCIT_MODE */

}

/**
  * @brief RTC Init LC sensor initiates WakeUpTimer after STOP mode to sense the LC detection
  * @retval None
  */
static void LC_Init_RTC(void)
{
  /* RTC INITIALIZATION */
  RTC_TamperTypeDef stamper;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;

  /**Initialize RTC and set the Time and Date
  */
  Hrtc.Instance = RTC;
  Hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  Hrtc.Init.AsynchPrediv = 1;
  Hrtc.Init.SynchPrediv = 16389;
  Hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  Hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  Hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  Hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&Hrtc) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
  /**Enable the WakeUp
  */
  /* Reset WUTF bit */
  Hrtc.Instance->ISR &= 0xFFFFFBFF;

  /* Disable all used wakeup source - will be managed outside this function*/
  if (HAL_RTCEx_DeactivateWakeUpTimer(&Hrtc) != HAL_OK)
  {
    /* DesactivateWakeUpTimer Error */
    ErrorHandler();
  }

  /**Enable the RTC Tamper 1
  */
  stamper.Tamper = RTC_TAMPER_1;
  stamper.Interrupt = RTC_TAMPER1_INTERRUPT;
  stamper.Trigger = RTC_TAMPERTRIGGER_RISINGEDGE;
  stamper.NoErase = RTC_TAMPER_ERASE_BACKUP_ENABLE;
  stamper.MaskFlag = RTC_TAMPERMASK_FLAG_DISABLE;
  stamper.Filter = RTC_TAMPERFILTER_DISABLE;
  stamper.SamplingFrequency = RTC_TAMPERSAMPLINGFREQ_RTCCLK_DIV32768;
  stamper.PrechargeDuration = RTC_TAMPERPRECHARGEDURATION_1RTCCLK;
  stamper.TamperPullUp = RTC_TAMPER_PULLUP_ENABLE;
  stamper.TimeStampOnTamperDetection = RTC_TIMESTAMPONTAMPERDETECTION_ENABLE;

  if (HAL_RTCEx_SetTamper_IT(&Hrtc, &stamper) != HAL_OK)
  {
    /* SetTamper_IT Error */
    ErrorHandler();
  }

  /** Configure Date and Time for alarm A and alarm B using
  */
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Sunday January 1st 2017 */
  sdatestructure.Year = 17;
  sdatestructure.Month = RTC_MONTH_JANUARY;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_SUNDAY;

  if (HAL_RTC_SetDate(&Hrtc, &sdatestructure, RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&Hrtc, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
}

#if (USART2_ENABLE == 1)
/**
  * @brief RTC ALARM_A Init function initiates Alarm A to communicate with user or other peripherals
  * @retval None
  */
static void LC_Init_AlarmA(void)
{
  RTC_DateTypeDef  sdate;
  RTC_TimeTypeDef  stime;
  RTC_AlarmTypeDef salarm;

  salarm.Alarm = RTC_ALARM_A;
  salarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  salarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
  salarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  salarm.AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
  salarm.AlarmTime.Hours = 0x00;
  salarm.AlarmTime.Minutes = 0x00;
  salarm.AlarmTime.SubSeconds = 0x00;

  HAL_RTC_GetTime(&Hrtc, &stime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&Hrtc, &sdate, RTC_FORMAT_BIN);

  salarm.AlarmTime.Seconds = (uint8_t) ((int8_t) stime.Seconds + 2);
  if (salarm.AlarmTime.Seconds >= 60)
  {
    salarm.AlarmTime.Seconds = (uint8_t) ((int8_t) salarm.AlarmTime.Seconds - 60);
  }

  if (HAL_RTC_SetAlarm_IT(&Hrtc, &salarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
}
#endif /* USART2_ENABLE */

/**
  * @brief RTC ALARM_B Init function initiates Alarm B to perform periodic calibrations
  * @retval None
  */
static void LC_init_AlarmB(void)
{
  RTC_AlarmTypeDef salarmstructure;

  salarmstructure.Alarm = RTC_ALARM_B;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_SECONDS;
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT_24;
  salarmstructure.AlarmTime.Hours = 0x00;
  salarmstructure.AlarmTime.Minutes = 0x05;
  salarmstructure.AlarmTime.Seconds = 0x00;
  salarmstructure.AlarmTime.SubSeconds = 0x00;

  if (HAL_RTC_SetAlarm_IT(&Hrtc, &salarmstructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
}
/**
  * @brief  Configures Comparator 1
  * @param  None
  * @retval None
  */
static void LC_Init_COMP1(void)
{
  /* Configure the COMP peripheral */
  Hcomp1.Instance = COMP1;
  if (DAC_CHANNEL_LC_VCMP == DAC_CHANNEL_2)
  {
    Hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
    Hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  }
  else
  {
    Hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
    Hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  }
  Hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  Hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  Hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  Hcomp1.Init.Mode = COMP_MODE_MEDIUMSPEED;
  Hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  Hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;

  if (HAL_COMP_Init(&Hcomp1) != HAL_OK)
  {
    /* Error */
    ErrorHandler();
  }
}

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
/**
  * @brief  Configures Comparator 2 (only if sensor 3 or sensor 4 enabled)
  * @param  None
  * @retval None
  */
static void Init_COMP2_LcSensor(void)
{
  /* Configure the COMP peripheral */
  Hcomp2.Instance = COMP2;
  if (DAC_CHANNEL_LC_VCMP == DAC_CHANNEL_2)
  {
    Hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
    Hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  }
  else
  {
    Hcomp2.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
    Hcomp2.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO2;
  }
  Hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  Hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  Hcomp2.Init.BlankingSrce = COMP_BLANKINGSRCE_NONE;
  Hcomp2.Init.Mode = COMP_MODE_MEDIUMSPEED;
  Hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  Hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;

  if (HAL_COMP_Init(&Hcomp2) != HAL_OK)
  {
    /* Error */
    ErrorHandler();
  }
}
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */


/**
  * @brief Initiates LPTIM1 settings to acquire pulses from LC sensor
  * @retval None
  */
static void LC_Init_LPTIM1(void)
{

  /* Initialize the LPTIM peripheral */
  /*
  *  Instance        = LPTIM1
  *  Clock Source    = LowPowerOSCillator
  *  Counter source  = External event.
  *  Clock prescaler = 1 (No division)
  *  Counter Trigger = Trigger source COMP1
  *  Output Polarity = High
  *  Update mode     = Immediate (Registers are immediately updated after any
  *                    write access)
  */

  Hlptim1.Instance = LPTIM1;
  Hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_ULPTIM;
  Hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  Hlptim1.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  Hlptim1.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  Hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  Hlptim1.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  Hlptim1.Init.Trigger.SampleTime = LPTIM_TRIGSAMPLETIME_DIRECTTRANSITION;
  Hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  Hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  Hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  Hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_COMP1;
  Hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;

  /* Initialize LPTIM peripheral according to the passed parameters */
  if (HAL_LPTIM_Init(&Hlptim1) != HAL_OK)
  {
    ErrorHandler();
  }

  /* Start counting */
  if (HAL_LPTIM_Counter_Start(&Hlptim1, 0xFFFF) != HAL_OK)
  {
    ErrorHandler();
  }

}

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
/**
  * @brief Initiates LPTIM2 settings to acquire pulses from LC sensor
  *        (only if sensor 3 or sensor 4 enabled)
  * @retval None
  */
void Init_LPTIM2_LcSensor(void)
{

  Hlptim2.Instance = LPTIM2;
  Hlptim2.Init.Clock.Source = LPTIM_CLOCKSOURCE_ULPTIM;
  Hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  Hlptim2.Init.UltraLowPowerClock.Polarity = LPTIM_CLOCKPOLARITY_RISING;
  Hlptim2.Init.UltraLowPowerClock.SampleTime = LPTIM_CLOCKSAMPLETIME_DIRECTTRANSITION;
  Hlptim2.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  Hlptim2.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_RISING;
  Hlptim2.Init.Trigger.SampleTime = LPTIM_TRIGSAMPLETIME_DIRECTTRANSITION;
  Hlptim2.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  Hlptim2.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  Hlptim2.Init.CounterSource = LPTIM_COUNTERSOURCE_EXTERNAL;
  Hlptim2.Init.Input1Source = LPTIM_INPUT1SOURCE_COMP2;
  Hlptim2.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;

  /* Initialize LPTIM peripheral according to the passed parameters */
  if (HAL_LPTIM_Init(&Hlptim2) != HAL_OK)
  {
    ErrorHandler();
  }

  /* Start counting */
  if (HAL_LPTIM_Counter_Start(&Hlptim2, 0xFFFF) != HAL_OK)
  {
    ErrorHandler();
  }
}
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */

/**
  * @brief Initiates DAC1 settings to generate COMP1 threshold negative input and VDD/2 reference voltage
  * @retval None
  */
static void LC_Init_DAC1(void)
{
  DAC_ChannelConfTypeDef sconfig;

  /**DAC Initialization
  */
  Hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&Hdac1) != HAL_OK)
  {
    /* Initiliazation Error */
    ErrorHandler();
  }

  /**DAC channel VMID config
  */
  /* Enable the Internal Low Speed oscillator (LSI) */
  __HAL_RCC_LSI_ENABLE();
  /* Wait till LSI is ready */
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
  {}

  sconfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
  sconfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 1;
  sconfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 1;
  sconfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 1;
  sconfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sconfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sconfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sconfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&Hdac1, &sconfig, DAC_CHANNEL_LC_VMID) != HAL_OK)
  {
    /* Channel configuration Error */
    ErrorHandler();
  }

  /**DAC channel VCOMP config
  */
  sconfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sconfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sconfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  sconfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_ENABLE;
  sconfig.DAC_SampleAndHoldConfig.DAC_SampleTime = 1;
  sconfig.DAC_SampleAndHoldConfig.DAC_HoldTime = 1;
  sconfig.DAC_SampleAndHoldConfig.DAC_RefreshTime = 1;
  if (HAL_DAC_ConfigChannel(&Hdac1, &sconfig, DAC_CHANNEL_LC_VCMP) != HAL_OK)
  {
    /* Channel configuration Error */
    ErrorHandler();
  }

  /* Set S/H Hold time */
  DAC1->SHHR = 0x00010001;

  /* Set S/H Refresh time */
  DAC1->SHRR = 0x00010001;

  /* Set DAC VMID DHR register */
  if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VMID, DAC_ALIGN_12B_R, LcConfig.DacVmid) != HAL_OK)
  {
    /* Setting value Error */
    ErrorHandler();
  }

  /* Set DAC COMP THRESHOLD DHR register */
  if (HAL_DAC_SetValue(&Hdac1, DAC_CHANNEL_LC_VCMP, DAC_ALIGN_12B_R, LcConfig.DacVcmp) != HAL_OK)
  {
    /* Setting value Error */
    ErrorHandler();
  }

  /* Start / Stop managed in RTC_WKUP_IRQHandler */
  if (HAL_DAC_Start(&Hdac1, DAC_CHANNEL_LC_VMID) != HAL_OK)
  {
    /* DAC Start Error */
    ErrorHandler();
  }

  if (HAL_DAC_Start(&Hdac1, DAC_CHANNEL_LC_VCMP) != HAL_OK)
  {
    /* DAC Start Error */
    ErrorHandler();
  }
}

/**
  * @brief  TIM6 Configuration
  *         The TIM6 timer serves low power timebase during LC sensing
  *         It is configured in one-shot mode to setup small duration
  *         during which the core will be in WFE mode.
  * @param  None
  * @retval None
  */
static void LC_Init_TIM6(void)
{

  Htim6.Instance = TIM6;
  Htim6.Init.Prescaler = 0;
  Htim6.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  Htim6.Init.Period = 100;
  HAL_TIM_Base_Init(&Htim6);

  if (HAL_TIM_OnePulse_Init(&Htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }

  /* Enable update interrupts to use them as events for sleep mode*/
  TIM6->DIER |= TIM_IT_UPDATE;

  TIM6->SR = ~(TIM_FLAG_UPDATE);
  NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);

}

/**
  * @brief  This function is used to execute once the parameters needed to enter Low Power (I/Os, VREFINT, etc...)
  * @param  None
  * @retval None
  */
static void StopEntry(void)
{
  /* GPIO config */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Change MODER to Analog state to decrease low power consumption */
  GPIOA->MODER = 0xFFFFFFFF;
  GPIOB->MODER = 0xFFFFFFFF;
  GPIOC->MODER = 0xFFFFFFFF;
  GPIOD->MODER = 0xFFFFFFFF;
  GPIOE->MODER = 0xFFFFFFFF;
  GPIOH->MODER = 0xFFFFFFFF;

#ifdef COMP1_OUT_ENABLED
  /* Eenable PB0 Comparator output for debug only */
  GPIOB->MODER &= 0xFFFFFFFE;
#endif /* COMP1_OUT_ENABLED */
#ifdef COMP2_OUT_ENABLED
  /* Enable PB11 Comparator output for debug only */
  GPIOB->MODER &= 0xFFBFFFFF;
#endif /* COMP2_OUT_ENABLED */

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

  /* Disable the SysTick timer: must be done after clock config! */
  HAL_SuspendTick();

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* The voltage scaling allows optimizing the power consumption when the device is
  clocked below the maximum system frequency, to update the voltage scaling value
  regarding system frequency refer to product datasheet.  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Power control voltage scaling Error */
    ErrorHandler();
  }

  /* Enter Stop Mode                                             */
  /* The UltraLow power should be put there !!! */
  /* VREFINT is inhibited in STOP mode and wake up is delayed until VREFINT is ready (no fast wake up mode) */
  HAL_PWR_EnableSEVOnPend();
  HAL_PWR_EnableSleepOnExit();

  /* Initialize Lc variables */
  LcSensorInit();

  /* Enter STOP2Mode */
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = (MSI)
  *            MSI Range                      = 9
  *            SYSCLK(Hz)                     = 24MHz
  *            HCLK(Hz)                       = 24MHz
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Main regulator output voltage  = Scale1 mode
  * @param  None
  * @retval None
  */
static void SystemClock_ConfigMSI_24M(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9; /* MSI 24Mhz */

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }

#if (USART2_ENABLE == 1)
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
#else /* not USART2_ENABLE */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
#endif /* USART2_ENABLE */
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    /* Power Control Voltage Scaling Error */
    ErrorHandler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#if (USART2_ENABLE == 1)
/**
  * @brief  USART2 Configuration
  *         It is used to send counter informations to user
  * @param  None
  * @retval None
  */
static void LC_Init_USART2(void)
{

  Huart2.Instance = USART2;
  Huart2.Init.BaudRate = 921600;
  Huart2.Init.WordLength = UART_WORDLENGTH_8B;
  Huart2.Init.StopBits = UART_STOPBITS_1;
  Huart2.Init.Parity = UART_PARITY_NONE;
  Huart2.Init.Mode = UART_MODE_TX_RX;
  Huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  Huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  Huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&Huart2) != HAL_OK)
  {
    ErrorHandler();
  }
}
#endif/* USART2_ENABLE */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

