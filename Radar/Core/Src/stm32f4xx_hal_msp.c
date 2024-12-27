/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32f4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_adc2;

extern DMA_HandleTypeDef hdma_usart3_tx;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	/* System interrupt init*/
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 1, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 1, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 1, 0);

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hadc->Instance == ADC1) {
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC1 GPIO Configuration
		 PA3     ------> ADC1_IN3
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/**ADC1 GPIO Configuration
		 PA6     ------> ADC1_IN6
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		/* ADC1 Init */
		hdma_adc1.Instance = DMA2_Stream0;
		hdma_adc1.Init.Channel = DMA_CHANNEL_0;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc1.Init.Mode = DMA_NORMAL;
		hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
		hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_adc1.Init.Mode |= DMA_IT_TC;  // Enable Transfer Complete Interrupt

		if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

		/* USER CODE BEGIN ADC1_MspInit 1 */

		/* USER CODE END ADC1_MspInit 1 */
	} else if (hadc->Instance == ADC2) {
		/* USER CODE BEGIN ADC2_MspInit 0 */

		/* USER CODE END ADC2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_ADC2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC2 GPIO Configuration
		 PA5     ------> ADC2_IN5
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC2 DMA Init */
		/* ADC2 Init */
		hdma_adc2.Instance = DMA2_Stream2;
		hdma_adc2.Init.Channel = DMA_CHANNEL_1;
		hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc2.Init.Mode = DMA_NORMAL;
		hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
		hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

		/* USER CODE BEGIN ADC2_MspInit 1 */

		/* USER CODE END ADC2_MspInit 1 */
	}

}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		 PA3     ------> ADC1_IN3
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

		/**ADC1 GPIO Configuration
		 PA6     ------> ADC1_IN6
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(hadc->DMA_Handle);
		/* USER CODE BEGIN ADC1_MspDeInit 1 */

		/* USER CODE END ADC1_MspDeInit 1 */
	} else if (hadc->Instance == ADC2) {
		/* USER CODE BEGIN ADC2_MspDeInit 0 */

		/* USER CODE END ADC2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_ADC2_CLK_DISABLE();

		/**ADC2 GPIO Configuration
		 PA5     ------> ADC2_IN5
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

		/* ADC2 DMA DeInit */
		HAL_DMA_DeInit(hadc->DMA_Handle);
		/* USER CODE BEGIN ADC2_MspDeInit 1 */

		/* USER CODE END ADC2_MspDeInit 1 */
	}

}

/**
 * @brief DAC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hdac->Instance == DAC) {
		/* USER CODE BEGIN DAC_MspInit 0 */

		/* USER CODE END DAC_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_DAC_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**DAC GPIO Configuration
		 PA4     ------> DAC_OUT1
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	}

}

/**
 * @brief DAC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hdac: DAC handle pointer
 * @retval None
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac) {
	if (hdac->Instance == DAC) {
		/* Peripheral clock disable */
		__HAL_RCC_DAC_CLK_DISABLE();

		/**DAC GPIO Configuration
		 PA4     ------> DAC_OUT1
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

	}
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (huart->Instance == USART2) {

		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**USART2 GPIO Configuration
		 PD5     ------> USART2_TX
		 PD6     ------> USART2_RX
		 */

		GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	} else if (huart->Instance == USART3) {

		__HAL_RCC_USART3_CLK_ENABLE();

		__HAL_RCC_GPIOD_CLK_ENABLE();
		/**USART3 GPIO Configuration
		 PD8     ------> USART3_TX
		 PD9     ------> USART3_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* USER CODE BEGIN USART3_MspInit 1 */
		/* USART3 DMA Init */
		/* USART3_TX Init */
		hdma_usart3_tx.Instance = DMA1_Stream3;
		hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_tx.Init.PeriphDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_usart3_tx.Init.Mode = DMA_NORMAL;
		hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

		/* USER CODE BEGIN USART3_MspInit 1 */

		/* USER CODE END USART3_MspInit 1 */
		/* USER CODE END USART3_MspInit 1 */
	}

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
		 PD6     ------> USART2_RX
		 PD5     ------> USART2_TX
		 */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5 | GPIO_PIN_6);
		/* USER CODE BEGIN USART3_MspDeInit 1 */

		/* USER CODE END USART3_MspDeInit 1 */
	} else if (huart->Instance == USART3) {
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration
		 PD8     ------> USART3_TX
		 PD9     ------> USART3_RX
		 */
		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);
		/* USART3 DMA DeInit */
		HAL_DMA_DeInit(huart->hdmatx);
	}
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
	if (htim_base->Instance == TIM1) {
		/* USER CODE BEGIN TIM1_MspInit 0 */

		/* USER CODE END TIM1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM1_CLK_ENABLE();
		/* USER CODE BEGIN TIM1_MspInit 1 */

		/* USER CODE END TIM1_MspInit 1 */
	} else if (htim_base->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	} else if (htim_base->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	} else if (htim_base->Instance == TIM8) {
		/* USER CODE BEGIN TIM8_MspInit 0 */

		/* USER CODE END TIM8_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM8_CLK_ENABLE();
		/* USER CODE BEGIN TIM8_MspInit 1 */

		/* USER CODE END TIM8_MspInit 1 */
	}
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base) {
	if (htim_base->Instance == TIM1) {
		/* USER CODE BEGIN TIM1_MspDeInit 0 */

		/* USER CODE END TIM1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM1_CLK_DISABLE();
		/* USER CODE BEGIN TIM1_MspDeInit 1 */

		/* USER CODE END TIM1_MspDeInit 1 */
	} else if (htim_base->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();
		/* USER CODE BEGIN TIM2_MspDeInit 1 */

		/* USER CODE END TIM2_MspDeInit 1 */
	} else if (htim_base->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspDeInit 0 */

		/* USER CODE END TIM3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
		/* USER CODE BEGIN TIM3_MspDeInit 1 */

		/* USER CODE END TIM3_MspDeInit 1 */
	} else if (htim_base->Instance == TIM8) {
		/* USER CODE BEGIN TIM8_MspDeInit 0 */

		/* USER CODE END TIM8_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM8_CLK_DISABLE();
		/* USER CODE BEGIN TIM8_MspDeInit 1 */

		/* USER CODE END TIM8_MspDeInit 1 */
	}

}
