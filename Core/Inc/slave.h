/*
 * melbus.h
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */

#ifndef INC_SLAVE_H_
#define INC_SLAVE_H_

void melbusInitReq(void);
void SendByteToMelbus(byte byteToSend);
void prevTrack(void);
void nextTrack(void);
void SendCartridgeInfo(byte cartridgeInfo[]);
void SendTrackInfo(byte trackInfo[]);
byte fixTrack(byte track);
void changeCD(byte *disk, byte *track, byte command);
#endif /* INC_SLAVE_H_ */
