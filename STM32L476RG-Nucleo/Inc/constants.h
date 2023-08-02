/**
  ******************************************************************************
  * @file    constants.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    8-September-2017
  * @brief   This file contains the common defines of the application
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#ifdef __cplusplus
extern "C"
{
#endif

  /* Includes ------------------------------------------------------------------*/
  /* Private define ------------------------------------------------------------*/
  /** @defgroup Pins definitions
    * @{
    */
#define USART_TX_Pin GPIO_PIN_2         /*!< USART_TX pin */
#define USART_TX_GPIO_Port GPIOA        /*!< UART_TX port */
#define USART_RX_Pin GPIO_PIN_3         /*!< USART_RX pin */
#define USART_RX_GPIO_Port GPIOA        /*!< UART_RX port */
#define LD2_Pin GPIO_PIN_5              /*!< LED2 pin */
#define LD2_GPIO_Port GPIOA             /*!< LED2 port */
#define TMS_Pin GPIO_PIN_13             /*!< TMS pin */
#define TMS_GPIO_Port GPIOA             /*!< TMS port */
#define TCK_Pin GPIO_PIN_14             /*!< TCK pin */
#define TCK_GPIO_Port GPIOA             /*!< TCK port */
#define SWO_Pin GPIO_PIN_3              /*!< SWO pin */
#define SWO_GPIO_Port GPIOB             /*!< SWO port */
  /**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __MXCONSTANTS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
