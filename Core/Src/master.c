/*
 * master.c
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */
#include "main.h"
#include "master.h"
#include "pins.h"
#include "delay.h"

static void PrepareMaster(void);
static void ResetMasterToSlave(void);
static void SendByteToMelbus2(byte);

static void PrepareMaster(void) {
	SetPinToOutput(MELBUS_BUSY_Pin);
	// As MD attempting to send in master mode, we need to pull the busy pin down ourselves, unlike SAT
	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_RESET);
	DELAY(400, us); // wait a bit before sending stuff
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToOutput(MELBUS_DATA_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_SET);
	SetPinToOutput(MELBUS_CLOCK_Pin);
}

static void ResetMasterToSlave(void) {
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_DATA_Pin);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	SetClockToInt();
	DELAY(400, us); // busy for 400 more us, internal CD does this too so let's just follow
	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_BUSY_Pin);
}

//This method generates our own clock. Used when in master mode.
static void SendByteToMelbus2(byte byteToSend) {
	DELAY(527, us); // Makes time between bytes sent in master mode very similar to internal CD player
	/*
	 * Send data on rising edge of clock, frequency seems to be ~60 KHz
	 * Logic analyser shows 8us of edge time on the clock line, on both bytes sent by HU and internal CD
	 */
	for (int8_t i = 7; i >= 0; i--) {
		HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_RESET);
		DELAY(8, us);
		HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, (byteToSend & (1 << i)));
		HAL_GPIO_WritePin(GPIOA, MELBUS_CLOCK_Pin, GPIO_PIN_SET);
		DELAY(8, us);
	}
	DELAY(20, us);
}

void SendText(byte header[], size_t size, bool init) {
	PrepareMaster();

	for (uint32_t b = 0; b < size; b++) {
		SendByteToMelbus2(header[b]);
	}

	if (init) {
		goto exit;
	}

	// TODO: send text

	exit:
	ResetMasterToSlave();
}

void reqMaster() {
	// We have to wait 10 ms between melbus_busy going HIGH and requesting master, otherwise it will ignore the request
	DELAY(10, ms);
	SetPinToOutput(MELBUS_DATA_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_RESET);
	DELAY(2, ms); // busy will go LOW after 1ms, then another 1ms of having data LOW will request master mode
	DELAY(200, us); // We add some slack here, melbus seems to accept it just fine. volvo devices are less accurate than we are anyway
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_DATA_Pin);
}
