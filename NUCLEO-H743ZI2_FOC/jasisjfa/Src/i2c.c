/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c3_tx;
DMA_HandleTypeDef hdma_i2c3_rx;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C0ECFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}
/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10C0ECFF;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

}
/* I2C4 init function */
void MX_I2C4_Init(void)
{

  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10C0ECFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    GPIO_InitStruct.Pin = I2C3_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C3_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(I2C3_SCL_GPIO_Port, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  
    /* I2C3 DMA Init */
    /* I2C3_TX Init */
    hdma_i2c3_tx.Instance = DMA1_Stream0;
    hdma_i2c3_tx.Init.Request = DMA_REQUEST_I2C3_TX;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2c3_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2c3_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_i2c3_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c3_tx);

    /* I2C3_RX Init */
    hdma_i2c3_rx.Instance = DMA1_Stream1;
    hdma_i2c3_rx.Init.Request = DMA_REQUEST_I2C3_RX;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2c3_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2c3_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_i2c3_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c3_rx);

  /* USER CODE BEGIN I2C3_MspInit 1 */

  /* USER CODE END I2C3_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspInit 0 */

  /* USER CODE END I2C4_MspInit 0 */
  
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C4 GPIO Configuration    
    PF14     ------> I2C4_SCL
    PF15     ------> I2C4_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C4 clock enable */
    __HAL_RCC_I2C4_CLK_ENABLE();
  /* USER CODE BEGIN I2C4_MspInit 1 */

  /* USER CODE END I2C4_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PC9     ------> I2C3_SDA
    PA8     ------> I2C3_SCL 
    */
    HAL_GPIO_DeInit(I2C3_SDA_GPIO_Port, I2C3_SDA_Pin);

    HAL_GPIO_DeInit(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin);

    /* I2C3 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmatx);
    HAL_DMA_DeInit(i2cHandle->hdmarx);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspDeInit 0 */

  /* USER CODE END I2C4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C4_CLK_DISABLE();
  
    /**I2C4 GPIO Configuration    
    PF14     ------> I2C4_SCL
    PF15     ------> I2C4_SDA 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN I2C4_MspDeInit 1 */

  /* USER CODE END I2C4_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
