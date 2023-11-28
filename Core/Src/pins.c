/*
 * pins.c
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */
#include "pins.h"
#include "main.h"

GPIO_InitTypeDef GPIO_InitStructInput = { .Mode = GPIO_MODE_INPUT, .Pull = GPIO_NOPULL };
GPIO_InitTypeDef GPIO_InitStructOutput = { .Mode = GPIO_MODE_OUTPUT_OD };
GPIO_InitTypeDef GPIO_InitStructIT = { .Pin = MELBUS_CLOCK_Pin, .Mode = GPIO_MODE_IT_RISING, .Pull = GPIO_NOPULL };

void SetClockToInt(void) {
	resetBitPosition();
	HAL_GPIO_Init(MELBUS_CLOCK_GPIO_Port, &GPIO_InitStructIT);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void SetPinToInput(uint16_t pin) {
	GPIO_InitStructInput.Pin = pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructInput);
}

void SetPinToOutput(uint16_t pin) {
	GPIO_InitStructOutput.Pin = pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructOutput);
}
