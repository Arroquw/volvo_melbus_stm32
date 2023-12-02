/*
 * master.h
 *
 *  Created on: Nov 28, 2023
 *      Author: justin
 */

#ifndef INC_MASTER_H_
#define INC_MASTER_H_

/* MRB2 from IMIV:
 *
 * Start of track:
 * 0 1E EC 87 FF DF DF FB D8 FA 0 2 1 3 2 0 80 99 54 68 61 74 20 4F 6C 64 20 50 61 69 72 20 4F 66
 * That Old Pair Of
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * (mem button was pressed)
 * D8 1E F9 1 1 3 1 0 1
 *
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 99 46 61 74 62 6F 79 20 53 6C 69 6D 20 2D 20 54 68
 * Fatboy Slim - Th
 * D8 1E F9 2 2 3 1 0 1
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 99 65 20 47 72 65 61 74 65 73 74 20 48 69 74 73 3A
 * e Greatest Hits:
 *
 * 0 1E EC 87 FF DF DF FB D8 FA 0 1 1 3 2 0 80 99 53 70 69 72 69 74 73 0 0 0 0 0 0 0 0 0
 * spirits
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 0 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20
 *
 * (mem button was pressed)
 * D8 1E F9 1 1 3 1 0 1
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 1 3 1 0 80 99 4A 75 6E 6B 69 65 20 58 6C 20 2D 20 52 61 64 69
 * Junkie Xl - Radi
 * D8 1E F9 2 2 3 1 0 1
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 2 3 1 0 80 99 6F 20 4A 78 6C 3A 20 61 20 42 72 6F 61 64 63 61
 * o Jxl: a Broadca
 * D8 1E F9 3 3 3 1 0 1
 * 0 1E EC 87 FF DF DF FB D8 FA 0 3 3 3 1 0 80 99 73 74 20 66 72 6F 6D 20 74 68 65 20 43 6F 6D 70
 * st from the Comp
 * sniff was cut off here sadly
 *

-> 1 x x x 0 80 -> title in rows, 16 bytes per row
-> 2 x x x 0 80 ->
-> 3 x x x 0 80 -> title in rows, 16 bytes per row, but not followed by 0x20 0x20 0x20
 */

#define TEXTINIT_SIZE 11
#define TEXTCMD_SIZE 27
union text_cmd {
	struct __PACKED {
		byte header[4];
		byte row;
		byte row2;
		byte row3;
		byte row4;
		byte footer[2];
		byte track;
		byte payload[16];
	}text_cmd_st;
	byte raw[TEXTCMD_SIZE];
};

void reqMaster();
void SendText(union text_cmd, bool init);

#endif /* INC_MASTER_H_ */
