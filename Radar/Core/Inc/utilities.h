#ifndef UTILITIES_H_
#define UTILITIES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"


void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_UART_Init(void);
void USART3_UART_Init(void);
void ADC1_Init(void);
void ADC2_Init(void);
void DAC_Init(void);
void TIM1_Init(void);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM8_Init(void);
void Error_Handler(void);
void DMA_Init(void);

int _write(int fd, char* ptr, int len);

#ifdef __cplusplus
}
#endif

#endif
