/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#define USART_REC_LEN 256
/******************************************************************************/
extern unsigned char USART_RX_BUF[USART_REC_LEN];
extern unsigned short USART_RX_STA;
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
#ifdef IMU_TEST_1
extern uint8_t IMU_RX_End_Flag;
//extern uint8_t IMU_RX_Len;
extern uint8_t IMU_RX_Buffer[IMU_BUFFER_SIZE];
extern uint8_t IMU_RX_Buffer2[IMU_BUFFER_SIZE];
#endif
/* USER CODE END Private defines */

void MX_UART8_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#ifdef IMU_TEST_1

#include "string.h"
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
