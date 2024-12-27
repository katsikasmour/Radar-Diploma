/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body for RADAR project
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <complex.h>
#include "utilities.h"
#include "fft.h"
/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define BUFFER_SIZE (SAMPLE_NUM * 2) // Double buffer size for ping-pong

uint16_t firstBuf[BUFFER_SIZE];
uint16_t secondBuf[BUFFER_SIZE];
uint16_t *currentBuffer = firstBuf;  // Points to the current buffer being filled by ADC DMA
uint16_t *processingBuffer= secondBuf; // Points to the buffer being processed and transmitted
uint16_t bufStart[] = { 0xff00, 0x00ff };

int bin;   //position of the max frequency
float sum;
float NF=0;
float speed = 0;
int countVehicles=0;
//buffers for DFT for IQ Channels and magnitude
float complex dft_I[SAMPLE_NUM];
float complex dft_Q[SAMPLE_NUM];
float abs_I[SAMPLE_NUM];
float abs_Q[SAMPLE_NUM];

/* Flags for synchronization */
volatile int adcReady = 0;      // Indicates when ADC DMA has filled a buffer
volatile int uartReady = 1; // Indicates when UART DMA is ready for the next transmission
volatile int activeBuffer = 0; // Indicates the current active buffer (0 or 1)

/* External variables */
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart3;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	GPIO_Init();
	DMA_Init();
	USART3_UART_Init();
	TIM3_Init();
	ADC1_Init();

	/* Initialize buffers */

	/* Start TIMER3 TO TRIGGER ADC1 */
	if (HAL_OK != HAL_TIM_Base_Start(&htim3))
		Error_Handler();
	/* Start ADC DMA in circular mode */
//	HAL_NVIC_DisableIRQ(ADC_IRQn); // Disable ADC interrupt (if not used)
	if (HAL_ADC_Start_DMA(&hadc1, (uint16_t*) currentBuffer, BUFFER_SIZE)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Infinite loop */
	while (1) {
		if (adcReady && uartReady) {
			// Swap buffers atomically
			__disable_irq();
//			HAL_NVIC_DisableIRQ(ADC_IRQn); // Disable ADC interrupt (if not used)
			adcReady = 0;  // Clear ADC ready flag
			uint16_t *temp = processingBuffer;
			processingBuffer = currentBuffer;
			currentBuffer = temp;
			activeBuffer = !activeBuffer; // Toggle active buffer
			__enable_irq();

			// Process and send the buffer
			processBuffer(processingBuffer);
			//FOR DEBUG to send starting bytes for the python script
            HAL_UART_Transmit(&huart3, (uint16_t*) bufStart, 4, 20);
			//If we want to send magnitude of FFT_I  / we want to print +1 item so do delete dc zero and have even number
            HAL_UART_Transmit(&huart3, (uint16_t *)abs_I, SAMPLE_NUM * sizeof(float), HAL_MAX_DELAY);
			sendBuffer(processingBuffer, BUFFER_SIZE);
			HAL_Delay(1);
			if (HAL_ADC_Start_DMA(&hadc1, (uint16_t*) currentBuffer, BUFFER_SIZE)
					!= HAL_OK) {
				Error_Handler();
			}
		}
	}
}

/**
 * @brief Process the buffer: Apply FFT and calculate magnitude.
 * @param buffer Pointer to the buffer with raw ADC data
 */
void processBuffer(uint16_t *buffer) {
	// Apply FFT or other processing logic on the buffer
	rfft(buffer, dft_I, dft_Q, SAMPLE_NUM * 2);
	maxAbs(dft_I, dft_Q, abs_I, abs_Q, &bin, &sum, SAMPLE_NUM);
	NF = sum / SAMPLE_NUM;
	//Detect strongest radar return standing 15 dB above noise floor
	if ((20 * log10(abs_I[bin])) > (20 * log10(NF) + 30)) {
	speed = 3.6 * (bin + 1)* (FREQUENCY_SAMPLE * 1000 / SAMPLE_NUM) / 160.0;
		if (speed > 10) {
		countVehicles++;
		}
	}
}

/**
 * @brief Transmit the buffer using UART DMA.
 * @param buffer Pointer to the buffer to be transmitted
 * @param size Size of the buffer in bytes
 */
void sendBuffer(uint16_t *buffer, size_t size) {
	uartReady = 0; // Clear UART ready flag
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	if (HAL_UART_Transmit_DMA(&huart3, (uint16_t*) buffer, size * 2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC DMA Conversion Complete Callback
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		adcReady = 1; // Mark buffer as ready for processing
	}
}

/**
 * @brief UART DMA Transmission Complete Callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		uartReady = 1; // Mark UART as ready for the next transmission
	}
}

///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
//
///* USER CODE BEGIN PV */
//uint8_t data[2000];
//uint16_t firstBuf[2 * SAMPLE_NUM];
//uint16_t secondBuf[2 * SAMPLE_NUM];
////uint16_t firstBufQ[SAMPLE_NUM];
//uint16_t *ptrBuf = &firstBuf[0];
//uint16_t *ptrUart = &firstBuf[0];
////uint16_t *ptrQ = &secondBuf[0];
//uint16_t bufStart[] = { 0xff00, 0x00ff };
//uint16_t *ptrStart = &bufStart[0];
//
//double complex dft_I[SAMPLE_NUM];
//double complex dft_Q[SAMPLE_NUM];
//double complex dft_Test[SAMPLE_NUM];
//float abs_I[SAMPLE_NUM];
//float abs_Q[SAMPLE_NUM];
//float abs_Test[SAMPLE_NUM];
//float *ptrABS = &abs_I[0];
//
//double sum;
//double NF;
//int j = 0;
//int x = 0;
//int rr = 0;
//int countVehicles = 0;
//float speed = 0;
//unsigned long ulUpdateTimeBegin = 0;
//unsigned long timeStamp_ms = 0;
//uint32_t ulSamplingTime = 1000 / FREQUENCY_SAMPLE; // result in micro seconds
//
//int bin;   //position of the max frequency
//
//uint8_t flagToTrasmit = 0;
///* flags for the flow*/
//int flagADC = 0;
//int flagToChooseBuff = 0;
//int flagUART = 1;
//int uartReady = 0;
//int flagToChoosePrint = 0;
//int flagtoWait = 0;
//int adcReady = 0;
//
//extern TIM_HandleTypeDef htim1;
//extern DMA_HandleTypeDef hdma_adc1;
//extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim8;
//extern DAC_HandleTypeDef hdac;
//extern UART_HandleTypeDef huart3;
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
///* USER CODE END 0 */
//
///**
// * @brief  The application entry point.
// * @retval int
// */
//int main(void) {
//	/* USER CODE BEGIN 1 */
//	/* USER CODE END 1 */
//
//	/* MCU Configuration--------------------------------------------------------*/
//
//	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//
//	/* Configure the system clock */
//
//	SystemClock_Config();
//	GPIO_Init();
//	DMA_Init();
//	HAL_Init();
//	USART3_UART_Init();
//	TIM2_Init();
//	TIM3_Init();
////	TIM8_Init();
//	DAC_Init();
//	ADC1_Init();
////	ADC2_Init();
//	HAL_Delay(5);
////	printf("start fft\n");
//	double complex out[8];
//	memset(out, 0, sizeof(out));
//
//	if (HAL_OK != HAL_TIM_Base_Start(&htim2))
//		Error_Handler();
//	if (HAL_OK != HAL_TIM_Base_Start(&htim3))
//		Error_Handler();
//
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
////	rr = HAL_UART_Transmit_DMA(&huart3, data, sizeof(data));
//	/* USER CODE END 2 */
//
//	///////fill the first buffer in order to be ready to get printed////////////////////////////
//	flagADC = 0;
//	if (HAL_OK != HAL_ADC_Start_DMA(&hadc1, (uint16_t*) ptrBuf,
//	SAMPLE_NUM * 2))
//		Error_Handler();
//	/* Infinite loop */
//	/* USER CODE BEGIN WHILE */
//	while (1) {
//
////		HAL_Delay(1);
//		if (flagADC == 1) {   // START ADC SAMPLING FOR BOTH CHANNELS
//			flagADC = 0;
//			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
//			HAL_Delay(1);
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
////			if (HAL_OK != HAL_ADC_Start_DMA(&hadc1, (uint16_t*) ptrBuf,
////			SAMPLE_NUM * 2))
////				Error_Handler();
//		}
//		if (adcReady == 1) {
//
//			adcReady = 0;
////			flagUART = 1;
//			flagADC = 1;
//			if (flagToChooseBuff == 0) {
//				ptrBuf = &firstBuf[0];
//			} else {
//				ptrBuf = &secondBuf[0];
//			}
//			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
//			if (HAL_OK != HAL_ADC_Start_DMA(&hadc1, (uint16_t*) ptrBuf,
//			SAMPLE_NUM * 2))
//				Error_Handler();
//		}
//		if (flagUART == 1) {
//			HAL_Delay(1);
//			flagUART = 0;
//			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
//			rfft(ptrBuf, dft_I, dft_Q, SAMPLE_NUM * 2);
//			maxAbs(dft_I, dft_Q, abs_I, abs_Q, &bin, &sum, SAMPLE_NUM);
//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//#ifdef DEBUG_SERIAL
//			HAL_UART_Transmit(&huart3, (uint16_t*) ptrStart, 4, 20);
//			HAL_NVIC_EnableIRQ(USART3_IRQn);
//			HAL_UART_Transmit(&huart3, (uint16_t *)abs_I, SAMPLE_NUM * sizeof(float), HAL_MAX_DELAY);
//			HAL_NVIC_EnableIRQ(USART3_IRQn);
//			if (HAL_OK != HAL_UART_Transmit_DMA(&huart3, (uint16_t*) ptrUart,
//			SAMPLE_NUM * 4))
//				Error_Handler(); //print one BYTE-z
//#endif
//
//
//			if (1) {
////				maxAbs(dft_I, dft_Q, abs_I, abs_Q, &bin, &sum, SAMPLE_NUM);
////				printf("\n--\n");
////				printf("max bin now is: %d\n", bin);
//
//				//Calculate noise floor
//				NF = sum / SAMPLE_NUM;
//
//				//Detect strongest radar return standing 15 dB above noise floor
//				if ((20 * log10(abs_I[bin])) > (20 * log10(NF) + 15)) {
//					speed = 3.6 * (bin + 1)
//							* (FREQUENCY_SAMPLE * 1000 / SAMPLE_NUM) / 160.0;
//					if (speed > 4) {
//						countVehicles++;
//					}
////					printf("Detected object with speed: %f km/h\n", speed);
//				} else {
////					printf("ZERO VEHICLE\n");
//				}
//
//			}
//		}
//		if (uartReady == 1) {
//			uartReady = 0;
//			flagUART = 1;
//			if (flagToChoosePrint == 0) {
//				ptrUart = &firstBuf[0];
//			} else {
//				ptrUart = &secondBuf[0];
//			}
//			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
//		}
//		/* USER CODE END 3 */
//	}
//}
//
//// Called when first half of buffer is filled
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
////  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//}
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	adcReady = 1; // Set a flag to indicate data is ready for processing
//	// Toggle the buffer flag for ping-pong operation
//	flagToChooseBuff = !flagToChooseBuff;
//}
//
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//
////	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
//	uartReady = 1;
//	flagToChoosePrint = !flagToChoosePrint;
//}
