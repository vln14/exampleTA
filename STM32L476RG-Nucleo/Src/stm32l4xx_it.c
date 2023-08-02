/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    8-September-2017
  * @brief   Interrupt Service Routines.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "lc_sensor_metering.h"
#include "stm32l4xx_hal_rtc.h"

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef Hrtc;
extern TIM_HandleTypeDef Htim6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  while (1)
  {}
}

/**
* @brief This function handles NMI interrupt.
*/
void NMI_Handler(void)
{
#ifdef DEBUG
  asm ("BKPT 0");
#endif
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC tamper and time stamp, CSS on LSE interrupts through EXTI line 19.
  */
void TAMP_STAMP_IRQHandler(void)
{
  HAL_RTCEx_TamperTimeStampIRQHandler(&Hrtc);
}

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 20.
  *        It performs LC measurements
  */
void RTC_WKUP_IRQHandler(void)
{

  /* Clear the WAKEUPTIMER interrupt pending bit */
  RTC->ISR &= ~RTC_FLAG_WUTF;

  /* Clear the EXTI's line Flag for RTC WakeUpTimer */
  __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();

  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Refresh DAC1 reference power supply from Analog state to Output  and Refresh DAC1 to feed COMP threshold*/
  SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID);

  /* Waiting time for DAC VMID sampling: Start TIM6 counter in single-shot, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  if (LcConfig.DacVmidRefreshTime > 12)
  {
    __LC_WAIT(LcConfig.DacVmidRefreshTime);
  }

  /* Enable the selected comparator */
#if defined(SENSOR_1_ENABLED) || defined(SENSOR_2_ENABLED)
  SET_BIT(COMP1->CSR, COMP_CSR_EN);
#endif /* defined(SENSOR_1_ENABLED) || defined(SENSOR_2_ENABLED) */
#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
  SET_BIT(COMP2->CSR, COMP_CSR_EN);
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */

#ifdef SENSOR_1_ENABLED
  /* Refresh DAC1 channel1 reference power supply from Analog state to Output  and Refresh DAC1 channel2 to feed COMP threshold*/
  SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID_EN_VCMP);

  /* COMP_NONINVERTINGINPUT_IO1 selected */
  CLEAR_BIT(COMP1->CSR, COMP_CSR_INPSEL);

  /* LPTIM1 counter */
  LcSensor1.CountTmp1 = LPTIM1->CNT;

  /* Set IO2 and start oscillations on sensor and back to analog state */
  __LC_EXCIT(LcIO.IO2_GPIO_GROUP, LcIO.IO2_OUTPUT_PP, LcIO.IO2_ANALOG, LcConfigSensor1.Texcit);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* Start TIM6 counter in single-shot, disable DAC, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_MEASURE(LcConfigSensor1.Tcapture);

  /* LPTIM1 counter */
  LcSensor1.CountTmp2 = LPTIM1->CNT;

#if defined(SENSOR_2_ENABLED) || defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
  /* Waiting time between 2 LC measurements: Start TIM6 counter in single-shot, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_WAIT(LcConfig.TimeBetweenSensor);
#endif /* defined(SENSOR_2_ENABLED) || defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */
#endif /* SENSOR_1_ENABLED */

#ifdef SENSOR_2_ENABLED
  /* Refresh DAC1 channel1 reference power supply from Analog state to Output  and Refresh DAC1 channel2 to feed COMP threshold*/
  SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID_EN_VCMP);

  /* COMP_NONINVERTINGINPUT_IO2 selected */
  SET_BIT(COMP1->CSR, COMP_CSR_INPSEL);

  /* LPTIM1 counter */
  LcSensor2.CountTmp1 = LPTIM1->CNT;

  /* Set IO3 and start oscillations on sensor and back to analog state */
  __LC_EXCIT(LcIO.IO3_GPIO_GROUP, LcIO.IO3_OUTPUT_PP, LcIO.IO3_ANALOG, LcConfigSensor2.Texcit);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* Start TIM6 counter in single-shot, disable DAC, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_MEASURE(LcConfigSensor2.Tcapture);

  /* LPTIM1 counter */
  LcSensor2.CountTmp2 = LPTIM1->CNT;

#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
  /* Waiting time between 2 LC measurements: Start TIM6 counter in single-shot, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_WAIT(LcConfig.TimeBetweenSensor);
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */
#endif /* SENSOR_2_ENABLED */

#ifdef SENSOR_3_ENABLED
  /* Refresh DAC1 channel1 reference power supply from Analog state to Output  and Refresh DAC1 channel2 to feed COMP threshold*/
  SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID_EN_VCMP);

  /* COMP_NONINVERTINGINPUT_IO1 selected */
  CLEAR_BIT(COMP2->CSR, COMP_CSR_INPSEL);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* LPTIM1 counter */
  LcSensor3.CountTmp1 = LPTIM2->CNT;

  /* Set IO4 and start oscillations on sensor and back to analog state */
  __LC_EXCIT(LcIO.IO4_GPIO_GROUP, LcIO.IO4_OUTPUT_PP, LcIO.IO4_ANALOG, LcConfigSensor3.Texcit);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* Start TIM6 counter in single-shot, disable DAC, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_MEASURE(LcConfigSensor3.Tcapture);

  /* LPTIM1 counter */
  LcSensor3.CountTmp2 = LPTIM2->CNT;

#ifdef SENSOR_4_ENABLED
  /* Waiting time between 2 LC measurements: Start TIM6 counter in single-shot, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_WAIT(LcConfig.TimeBetweenSensor);
#endif /* SENSOR_4_ENABLED */
#endif /* SENSOR_3_ENABLED */

#ifdef SENSOR_4_ENABLED
  /* Refresh DAC1 channel1 reference power supply from Analog state to Output  and Refresh DAC1 channel2 to feed COMP threshold*/
  SET_BIT(DAC1->CR, DAC_CR_LC_EN_VMID_EN_VCMP);

  /* COMP_NONINVERTINGINPUT_IO2 selected */
  SET_BIT(COMP2->CSR, COMP_CSR_INPSEL);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* LPTIM1 counter */
  LcSensor4.CountTmp1 = LPTIM2->CNT;

  /* Set IO5 and start oscillations on sensor and back to analog state */
  __LC_EXCIT(LcIO.IO5_GPIO_GROUP, LcIO.IO5_OUTPUT_PP, LcIO.IO5_ANALOG, LcConfigSensor4.Texcit);

  /* DAC COMP threshold can be stopped very soon (internal S/H) */
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VCMP);

  /* Start TIM6 counter in single-shot, disable DAC, decrease HCLK during sleep and go back to full speed after TIM6 event*/
  __LC_MEASURE(LcConfigSensor4.Tcapture);

  /* LPTIM1 counter */
  LcSensor4.CountTmp2 = LPTIM2->CNT;
#endif /* SENSOR_4_ENABLED */

  /* Power down the DAC1 (Holding capacitor on Vmid pin)*/
  CLEAR_BIT(DAC1->CR, DAC_CR_LC_EN_VMID);

  /* Power down the comparator */
#if defined(SENSOR_1_ENABLED) || defined(SENSOR_2_ENABLED)
  CLEAR_BIT(COMP1->CSR, COMP_CSR_EN);
#endif /* defined(SENSOR_1_ENABLED) || defined(SENSOR_2_ENABLED) */
#if defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED)
  CLEAR_BIT(COMP2->CSR, COMP_CSR_EN);
#endif /* defined(SENSOR_3_ENABLED) || defined(SENSOR_4_ENABLED) */

  /* Increment measures number */
  LcStatus.MeasuresCount++;

  /* Process LC sensor state */
#ifdef SENSOR_1_ENABLED
  /* Compute pulses number */
  if (LcSensor1.CountTmp1 <= LcSensor1.CountTmp2)
  {
    LcSensor1.CountValue = LcSensor1.CountTmp2 - LcSensor1.CountTmp1;
  }
  else
  {
    LcSensor1.CountValue = LcSensor1.CountTmp2 + (0xFFFF - LcSensor1.CountTmp1);
  }

  /* If oscillations are damped more than Count_detect then update Sensor 1 status */
  if (LcSensor1.CountValue < LcSensor1.CountDetect)
  {
    LcSensor1.Status = METAL;
  }
  else
  {
    LcSensor1.Status = NO_METAL;
  }

  /* Periodic calibration if required */
  if (LcStatus.CalStatus.State == CAL_ON_GOING)
  {
    if (LcSensor1.CountValue > LcStatus.CalStatus.Sensor1CountMax)
    {
      LcStatus.CalStatus.Sensor1CountMax = LcSensor1.CountValue;
    }
    if (LcSensor1.CountValue < LcStatus.CalStatus.Sensor1CountMin)
    {
      LcStatus.CalStatus.Sensor1CountMin = LcSensor1.CountValue;
    }
    if ((LcSensor1.Status || LcSensor1.PreviousStatus) && !(LcSensor1.Status && LcSensor1.PreviousStatus ))
    {
      LcStatus.CalStatus.Sensor1CountTransitions += 1;
    }
  }

#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
  /* Compute pulses number */
  if (LcSensor2.CountTmp1 <= LcSensor2.CountTmp2)
  {
    LcSensor2.CountValue = LcSensor2.CountTmp2 - LcSensor2.CountTmp1;
  }
  else
  {
    LcSensor2.CountValue = LcSensor2.CountTmp2 + (0xFFFF - LcSensor2.CountTmp1);
  }

  /* If oscillations are damped more than Count_detect then update Sensor 2 status */
  if (LcSensor2.CountValue < LcSensor2.CountDetect)
  {
    LcSensor2.Status = METAL;
  }
  else
  {
    LcSensor2.Status = NO_METAL;
  }

  /* Periodic calibration if required */
  if (LcStatus.CalStatus.State == CAL_ON_GOING)
  {
    if (LcSensor2.CountValue > LcStatus.CalStatus.Sensor2CountMax)
    {
      LcStatus.CalStatus.Sensor2CountMax = LcSensor2.CountValue;
    }
    if (LcSensor2.CountValue < LcStatus.CalStatus.Sensor2CountMin)
    {
      LcStatus.CalStatus.Sensor2CountMin = LcSensor2.CountValue;
    }
    if ((LcSensor2.Status || LcSensor2.PreviousStatus) && !(LcSensor2.Status && LcSensor2.PreviousStatus ))
    {
      LcStatus.CalStatus.Sensor2CountTransitions += 1;
    }
  }
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
  /* Compute pulses number */
  if (LcSensor3.CountTmp1 <= LcSensor3.CountTmp2)
  {
    LcSensor3.CountValue = LcSensor3.CountTmp2 - LcSensor3.CountTmp1;
  }
  else
  {
    LcSensor3.CountValue = LcSensor3.CountTmp2 + (0xFFFF - LcSensor3.CountTmp1);
  }

  /* If oscillations are damped more than Count_detect then update Sensor 3 status */
  if (LcSensor3.CountValue < LcSensor3.CountDetect)
  {
    LcSensor3.Status = METAL;
  }
  else
  {
    LcSensor3.Status = NO_METAL;
  }

  /* Detect sensor transitions*/
  if ((LcSensor3.Status || LcSensor3.PreviousStatus) && !(LcSensor3.Status && LcSensor3.PreviousStatus ))
  {
    LcSensor3.Transitions ++;
  }

  /* Periodic calibration if required */
  if (LcStatus.CalStatus.State == CAL_ON_GOING)
  {
    if (LcSensor3.CountValue > LcStatus.CalStatus.Sensor3CountMax)
    {
      LcStatus.CalStatus.Sensor3CountMax = LcSensor3.CountValue;
    }
    if (LcSensor3.CountValue < LcStatus.CalStatus.Sensor3CountMin)
    {
      LcStatus.CalStatus.Sensor3CountMin = LcSensor3.CountValue;
    }
    if ((LcSensor3.Status || LcSensor3.PreviousStatus) && !(LcSensor3.Status && LcSensor3.PreviousStatus ))
    {
      LcStatus.CalStatus.Sensor3CountTransitions += 1;
    }
  }
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
  /* Compute pulses number */
  if (LcSensor4.CountTmp1 <= LcSensor4.CountTmp2)
  {
    LcSensor4.CountValue = LcSensor4.CountTmp2 - LcSensor4.CountTmp1;
  }
  else
  {
    LcSensor4.CountValue = LcSensor4.CountTmp2 + (0xFFFF - LcSensor4.CountTmp1);
  }

  /* If oscillations are damped more than Count_detect then update Sensor 4 status */
  if (LcSensor4.CountValue < LcSensor4.CountDetect)
  {
    LcSensor4.Status = METAL;
  }
  else
  {
    LcSensor4.Status = NO_METAL;
  }

  /* Periodic calibration if required */
  if (LcStatus.CalStatus.State == CAL_ON_GOING)
  {
    if (LcSensor4.CountValue > LcStatus.CalStatus.Sensor4CountMax)
    {
      LcStatus.CalStatus.Sensor4CountMax = LcSensor4.CountValue;
    }
    if (LcSensor4.CountValue < LcStatus.CalStatus.Sensor4CountMin)
    {
      LcStatus.CalStatus.Sensor4CountMin = LcSensor4.CountValue;
    }
    if ((LcSensor4.Status || LcSensor4.PreviousStatus) && !(LcSensor4.Status && LcSensor4.PreviousStatus ))
    {
      LcStatus.CalStatus.Sensor4CountTransitions += 1;
    }
  }
#endif /* SENSOR_4_ENABLED */

#if (LC_SENSOR_DEMO == 1)
  /* Process edge count for Demo1*/
  /* Detect sensors transitions for Counting demo with 1 LC sensor*/
  if ((LcSensor1.Status || LcSensor1.PreviousStatus) && !(LcSensor1.Status && LcSensor1.PreviousStatus ))
  {
    LcStatus.EdgeCount++;
  }
#endif /* LC_SENSOR_DEMO 1 */

#if ((LC_SENSOR_DEMO == 2) || (LC_SENSOR_DEMO == 3) || (LC_SENSOR_DEMO == 4))
  /* Basic SW state machine for Counting demo with 2 LC sensors - Demo2, Demo3 or Demo4 */
  /* Detect sensors transitions and direction - Quarter precision*/
  if ((LcSensor1.Status || LcSensor1.PreviousStatus) && !(LcSensor1.Status && LcSensor1.PreviousStatus ))
  {
    LcSensor1.Transitions = 1;
  }
  else
  {
    LcSensor1.Transitions = 0;
  }
  if ((LcSensor2.Status || LcSensor2.PreviousStatus) && !(LcSensor2.Status && LcSensor2.PreviousStatus ))
  {
    LcSensor2.Transitions = 1;
  }
  else
  {
    LcSensor2.Transitions = 0;
  }

  /* Errors in case of erroneous transition (more than 1 bit changing between 2 consecutives tates - Action must be take */
  if (LcSensor1.Transitions == 1 && LcSensor2.Transitions == 1)
  {
    LcStatus.Errors++;
  }

  /* Update counters if sensor 1 transition detected*/
  else if (LcSensor1.Transitions == 1)
  {
    if (LcSensor1.Status == LcSensor2.PreviousStatus)
    {
      LcStatus.EdgeCount--;
      LcStatus.EdgeCountNeg++;
    }
    else
    {
      LcStatus.EdgeCount++;
      LcStatus.EdgeCountPos++;
    }
  }
  /* Update counters if sensor2 transition detected*/
  else if (LcSensor2.Transitions == 1)
  {
    if (LcSensor2.Status == LcSensor1.PreviousStatus)
    {
      LcStatus.EdgeCount++;
      LcStatus.EdgeCountPos++;
    }
    else
    {
      LcStatus.EdgeCount--;
      LcStatus.EdgeCountNeg++;
    }
  }
#endif /* (LC_SENSOR_DEMO 2) || (LC_SENSOR_DEMO 3) || (LC_SENSOR_DEMO 4)*/

#if (LC_SENSOR_DEMO == 4)
  /* Basic SW state machine for Counting demo with 2 x 2 LC sensors - Demo4 */
  /* Detect sensors transitions and direction - Quarter precision*/
  if ((LcSensor3.Status || LcSensor3.PreviousStatus) && !(LcSensor3.Status && LcSensor3.PreviousStatus ))
  {
    LcSensor3.Transitions = 1;
  }
  else
  {
    LcSensor3.Transitions = 0;
  }
  if ((LcSensor4.Status || LcSensor4.PreviousStatus) && !(LcSensor4.Status && LcSensor4.PreviousStatus ))
  {
    LcSensor4.Transitions = 1;
  }
  else
  {
    LcSensor4.Transitions = 0;
  }

  /* Errors in case of erroneous transition (more than 1 bit changing between 2 consecutives tates - Action must be take */
  if (LcSensor3.Transitions == 1 && LcSensor4.Transitions == 1)
  {
    LcStatus.Errors++;
  }

  /* Update counters if sensor 3 transition detected*/
  else if (LcSensor3.Transitions == 1)
  {
    if (LcSensor3.Status == LcSensor4.PreviousStatus)
    {
      LcStatus.EdgeCount2--;
      LcStatus.EdgeCountNeg2++;
    }
    else
    {
      LcStatus.EdgeCount2++;
      LcStatus.EdgeCountPos2++;
    }
  }
  /* Update counters if sensor4 transition detected*/
  else if (LcSensor4.Transitions == 1)
  {
    if (LcSensor4.Status == LcSensor3.PreviousStatus)
    {
      LcStatus.EdgeCount2++;
      LcStatus.EdgeCountPos2++;
    }
    else
    {
      LcStatus.EdgeCount2--;
      LcStatus.EdgeCountNeg2++;
    }
  }
#endif /* LC_SENSOR_DEMO 4 */

  /* Refresh PreviousStatus for next transition detection*/
#ifdef SENSOR_1_ENABLED
  LcSensor1.PreviousStatus = LcSensor1.Status;
#endif /* SENSOR_1_ENABLED */
#ifdef SENSOR_2_ENABLED
  LcSensor2.PreviousStatus = LcSensor2.Status;
#endif /* SENSOR_2_ENABLED */
#ifdef SENSOR_3_ENABLED
  LcSensor3.PreviousStatus = LcSensor3.Status;
#endif /* SENSOR_3_ENABLED */
#ifdef SENSOR_4_ENABLED
  LcSensor4.PreviousStatus = LcSensor4.Status;
#endif /* SENSOR_4_ENABLED */

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
}

/**
  * @brief This function handles RTC alarm interrupt through EXTI line 18.
  */
void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&Hrtc);
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  __HAL_TIM_CLEAR_IT(&Htim6, TIM_FLAG_UPDATE);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
