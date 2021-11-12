/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void Error_Handler(void);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR3_DIR_Pin GPIO_PIN_13
#define MOTOR3_DIR_GPIO_Port GPIOC
#define MOTOR2_DIR_Pin GPIO_PIN_5
#define MOTOR2_DIR_GPIO_Port GPIOF
#define MOTOR2_FF1_Pin GPIO_PIN_7
#define MOTOR2_FF1_GPIO_Port GPIOF
#define MOTOR2_FF2_Pin GPIO_PIN_8
#define MOTOR2_FF2_GPIO_Port GPIOF
#define MOTOR2_MODE_Pin GPIO_PIN_9
#define MOTOR2_MODE_GPIO_Port GPIOF
#define MOTOR2_RESET_Pin GPIO_PIN_10
#define MOTOR2_RESET_GPIO_Port GPIOF
#define SPI2_MOSI_Pin GPIO_PIN_1
#define SPI2_MOSI_GPIO_Port GPIOC
#define SPI2_MISO_Pin GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOC
#define INFRAIN_Pin GPIO_PIN_3
#define INFRAIN_GPIO_Port GPIOA
#define SPI3_NSS_Pin GPIO_PIN_4
#define SPI3_NSS_GPIO_Port GPIOA
#define Battery_Pin GPIO_PIN_4
#define Battery_GPIO_Port GPIOC
#define INFRA_LED_Pin GPIO_PIN_5
#define INFRA_LED_GPIO_Port GPIOC
#define Boot_Cap_Pin GPIO_PIN_11
#define Boot_Cap_GPIO_Port GPIOF
#define SPI3_IRQ_Pin GPIO_PIN_12
#define SPI3_IRQ_GPIO_Port GPIOF
#define MOTORD_DIR_Pin GPIO_PIN_7
#define MOTORD_DIR_GPIO_Port GPIOE
#define MOTORD_MODE_Pin GPIO_PIN_8
#define MOTORD_MODE_GPIO_Port GPIOE
#define MOTORD_RESET_Pin GPIO_PIN_10
#define MOTORD_RESET_GPIO_Port GPIOE
#define MOTORD_FF1_Pin GPIO_PIN_12
#define MOTORD_FF1_GPIO_Port GPIOE
#define MOTORD_FF2_Pin GPIO_PIN_13
#define MOTORD_FF2_GPIO_Port GPIOE
#define SPI2_CE_Pin GPIO_PIN_15
#define SPI2_CE_GPIO_Port GPIOE
#define SPI2_SCK_Pin GPIO_PIN_10
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI2_IRQ_Pin GPIO_PIN_13
#define SPI2_IRQ_GPIO_Port GPIOB
#define MOTOR1_FF1_Pin GPIO_PIN_10
#define MOTOR1_FF1_GPIO_Port GPIOD
#define MOTOR1_FF2_Pin GPIO_PIN_11
#define MOTOR1_FF2_GPIO_Port GPIOD
#define UART8_RTS_Pin GPIO_PIN_15
#define UART8_RTS_GPIO_Port GPIOD
#define MOTOR1_DIR_Pin GPIO_PIN_2
#define MOTOR1_DIR_GPIO_Port GPIOG
#define MOTOR1_RESET_Pin GPIO_PIN_3
#define MOTOR1_RESET_GPIO_Port GPIOG
#define MOTOR1_MODE_Pin GPIO_PIN_4
#define MOTOR1_MODE_GPIO_Port GPIOG
#define MOTOR4_MODE_Pin GPIO_PIN_5
#define MOTOR4_MODE_GPIO_Port GPIOG
#define MOTOR4_RESET_Pin GPIO_PIN_6
#define MOTOR4_RESET_GPIO_Port GPIOG
#define MOTOR4_FF1_Pin GPIO_PIN_7
#define MOTOR4_FF1_GPIO_Port GPIOG
#define MOTOR4_FF2_Pin GPIO_PIN_8
#define MOTOR4_FF2_GPIO_Port GPIOG
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define MOTOR4_DIR_Pin GPIO_PIN_11
#define MOTOR4_DIR_GPIO_Port GPIOA
#define IMU_NRST_Pin GPIO_PIN_15
#define IMU_NRST_GPIO_Port GPIOA
#define HEART_Pin GPIO_PIN_12
#define HEART_GPIO_Port GPIOC
#define TX_COM_Pin GPIO_PIN_0
#define TX_COM_GPIO_Port GPIOD
#define RX_COM_Pin GPIO_PIN_1
#define RX_COM_GPIO_Port GPIOD
#define BOOT_DONE_Pin GPIO_PIN_2
#define BOOT_DONE_GPIO_Port GPIOD
#define SPI3_CE_Pin GPIO_PIN_3
#define SPI3_CE_GPIO_Port GPIOD
#define PCA9539_RESET_Pin GPIO_PIN_5
#define PCA9539_RESET_GPIO_Port GPIOD
#define SPI1_CE_Pin GPIO_PIN_6
#define SPI1_CE_GPIO_Port GPIOD
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOD
#define SPI1_MISO_Pin GPIO_PIN_9
#define SPI1_MISO_GPIO_Port GPIOG
#define SPI1_NSS_Pin GPIO_PIN_10
#define SPI1_NSS_GPIO_Port GPIOG
#define SPI1_SCK_Pin GPIO_PIN_11
#define SPI1_SCK_GPIO_Port GPIOG
#define SPI1_IRQ_Pin GPIO_PIN_12
#define SPI1_IRQ_GPIO_Port GPIOG
#define F27_RXEN_Pin GPIO_PIN_13
#define F27_RXEN_GPIO_Port GPIOG
#define F27_TXEN_Pin GPIO_PIN_14
#define F27_TXEN_GPIO_Port GPIOG
#define CURRENT_DRDY_Pin GPIO_PIN_15
#define CURRENT_DRDY_GPIO_Port GPIOG
#define MOTOR3_MODE_Pin GPIO_PIN_3
#define MOTOR3_MODE_GPIO_Port GPIOB
#define MOTOR3_FF1_Pin GPIO_PIN_6
#define MOTOR3_FF1_GPIO_Port GPIOB
#define MOTOR3_FF2_Pin GPIO_PIN_7
#define MOTOR3_FF2_GPIO_Port GPIOB
#define MOTOR3_RESET_Pin GPIO_PIN_8
#define MOTOR3_RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ROBOT_50W
// #define ROBOT_70W
#define ENCODER_360              //360Ïß
//#define ENCODER_1000
#define bandwidth_1Mbps
// #define NEW_CAR
#define IMU_TEST_1
#define IMU_BUFFER_SIZE 30
#define AngleTest
//#define bandwidth_250kbps
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
