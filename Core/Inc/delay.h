/*
 * delay.h
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "main.h"
#define DELAY(x, y) DWT_Delay_##y(x)

uint32_t DWT_Delay_Init(void);
__STATIC_INLINE void DWT_Delay_ms(volatile uint32_t au32_milliseconds)
{
	uint32_t au32_initial_ticks = DWT->CYCCNT;
	uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000);
	au32_milliseconds *= au32_ticks;
	while ((DWT->CYCCNT - au32_initial_ticks) < au32_milliseconds);
}
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
	uint32_t au32_initial_ticks = DWT->CYCCNT;
	uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
	au32_microseconds *= au32_ticks;
	while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}

#endif /* INC_DELAY_H_ */
