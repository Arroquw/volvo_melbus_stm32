/*
 * pins.h
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */

#ifndef INC_PINS_H_
#define INC_PINS_H_

#include <stdint.h>

void SetClockToInt(void);
void SetPinToInput(uint16_t pin);
void SetPinToOutput(uint16_t pin);

#endif /* INC_PINS_H_ */
