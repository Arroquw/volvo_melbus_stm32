/*
 * master.h
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */

#ifndef INC_MASTER_H_
#define INC_MASTER_H_

void reqMaster();
void SendText(byte header[], size_t size, bool init);

#endif /* INC_MASTER_H_ */
