/*
 * slave.c
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */
#include "main.h"
#include "pins.h"
#include "delay.h"

//Notify HU that we want to trigger the first initiate procedure to add a new device
//(CD-CHGR/SAT etc) by pulling BUSY line low for 1s
void melbusInitReq(void) {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	// Wait until Busy-line de-asserts (not busy, high) before we pull BUSY low to request init
	while (HAL_GPIO_ReadPin(GPIOA, MELBUS_BUSY_Pin) == GPIO_PIN_RESET) {
	}

	SetPinToOutput(MELBUS_BUSY_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_RESET);

	DELAY(1000, ms);

	HAL_GPIO_WritePin(GPIOA, MELBUS_BUSY_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_BUSY_Pin);

	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void SendByteToMelbus(byte byteToSend) {
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);

	SetPinToOutput(MELBUS_DATA_Pin);
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_CLOCK_Pin);
	for (int i = 7; i >= 0; i--) {
		while (HAL_GPIO_ReadPin(GPIOA, MELBUS_CLOCK_Pin) == GPIO_PIN_SET) {
		} //wait for low clock
		HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, byteToSend & (1 << i));
		while (HAL_GPIO_ReadPin(GPIOA, MELBUS_CLOCK_Pin) == GPIO_PIN_RESET) {
		}  //wait for high clock
	}
	//Let the value be read by the HU
	DELAY(5, us);
	HAL_GPIO_WritePin(GPIOA, MELBUS_DATA_Pin, GPIO_PIN_SET);
	SetPinToInput(MELBUS_DATA_Pin);
	SetClockToInt();
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void SendTrackInfo(byte trackInfo[]) {
	for (uint32_t i = 0; i < 9; i++) {
		SendByteToMelbus(trackInfo[i]);
	}
}

void SendCartridgeInfo(byte cartridgeInfo[]) {
	for (uint32_t i = 0; i < 6; i++) {
		SendByteToMelbus(cartridgeInfo[i]);
	}
}

byte fixTrack(byte track) {
	//cut out A-F in each nibble, and skip "00"
	byte hn = track >> 4; // high nibble
	byte ln = track & 0xF; // low nibble
	if (ln == 0xA) {
		ln = 0;
		hn += 1;
	}
	if (ln == 0xF) {
		ln = 9;
	}
	if (hn == 0xA) {
		hn = 0;
		ln = 1;
	}
	if ((hn == 0) && (ln == 0)) {
		ln = 0x9;
		hn = 0x9;
	}
	return ((hn << 4) + ln);
}

void changeCD(byte *disk, byte *track, byte command) {
	switch (command) {
	//0x81 to 0x86 corresponds to cd buttons 1 to 6 on the HU (650)
	case 0x41:  //next cd
		*disk = *disk + 1;
		*track = 1;
		break;
	case 0x01:  //prev cd
		*disk = *disk - 1;
		*track = 1;
		break;
	default:
		*track = 1;
		break;
	}
}
