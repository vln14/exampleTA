/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    8-September-2017
  * @brief   This file provides code for the MSP Initialization and de-Initialization codes.
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

/**
  * brief Initializes the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief COMP MSP Initialization
  * @param hcomp: COMP handle pointer
  * @retval None
  */
void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hcomp->Instance == COMP1)
  {
    /**COMP1 GPIO Configuration
    PC5     ------> COMP1_INP
    PB0     ------> COMP1_OUT 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF12_COMP1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

}

/**
  * @brief COMP MSP De-Initialization
  * @param hcomp: COMP handle pointer
  * @retval None
  */
void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{

  if (hcomp->Instance == COMP1)
  {
    /**COMP1 GPIO Configuration
    PC5     ------> COMP1_INP
    PB0     ------> COMP1_OUT 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

  }
}

/**
  * @brief DAC MSP Initialization
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (hdac->Instance == DAC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_DAC1_CLK_ENABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}

/**
  * @brief DAC MSP De-Initialization
  * @param hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{

  if (hdac->Instance == DAC1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_DAC1_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
}

/**
  * @brief LPTIM MSP Initialization
  * @param hlptim: LPTIM handle pointer
  * @retval None
  */
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef* hlptim)
{

  if (hlptim->Instance == LPTIM1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_LPTIM1_CLK_ENABLE();
  }
  else if (hlptim->Instance == LPTIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_LPTIM2_CLK_ENABLE();
  }
}

/**
  * @brief LPTIM MSP De-Initialization
  * @param hlptim: LPTIM handle pointer
  * @retval None
  */
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef* hlptim)
{

  if (hlptim->Instance == LPTIM1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM1_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
  }
  else if (hlptim->Instance == LPTIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_LPTIM2_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(LPTIM2_IRQn);
  }
}

/**
  * @brief RTC MSP Initialization
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{

  if (hrtc->Instance == RTC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();

    /* Peripheral interrupt init */
    /* RTC_WKUP with higher priority */
    HAL_NVIC_SetPriority(TAMP_STAMP_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(TAMP_STAMP_IRQn);
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  }
}

/**
  * @brief RTC MSP De-Initialization
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

  if (hrtc->Instance == RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TAMP_STAMP_IRQn);
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    HAL_NVIC_DisableIRQ(RTC_Alarm_IRQn);

  }
}

/**
  * @brief TIM MSP Initialization
  * @param htim_base: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if (htim_base->Instance == TIM6)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
  else if (htim_base->Instance == TIM7)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
  }

}

/**
  * @brief TIM MSP De-Initialization
  * @param htim_base: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if (htim_base->Instance == TIM6)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
  else if (htim_base->Instance == TIM7)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  }
}

/**
  * @brief TIM MSP Initialization
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    /* Enable the TIMx clock */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}

/**
  * @brief TIM MSP De-Initialization
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    /* Enable the TIMx clock */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
}

/**
  * @brief UART MSP Initialization
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if (huart->Instance == USART2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/**
  * @brief UART MSP De-Initialization
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if (huart->Instance == USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin | USART_RX_Pin);

  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
