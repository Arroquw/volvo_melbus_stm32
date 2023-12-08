# volvo_melbus_stm32
Based on https://github.com/visualapproach/Volvo-melbus

I removed SAT code as there is no way for me to test it (don't have an HU-x50, only x03). Should be easy to readd to main.c though, just needs some changing of the SendText and SendByteToMelbus2 functions in master.c to accomodate for the fact that SAT does not pull the BUSY pin low (apparently/allegedly).

All the logic for the bus itself is done in Core/main.c, all the other files contain functions that are for supporting the base protocol bitbanging. Adding a new command is as simple as adding the definition for it to main.h, adding it to the commands array in main.c and expanding the enum in main.c with your entry.

# Sending text
Sending text is done in the MRB/MRB_2 cases in the main switch in main.c, and adding text is done in the MD_RTR_x case. MD_RTR_1 requests row 1, MD_RTR_2 requests row 2 and MD_RTR_3 requests row 3. Each row can contain up to 16 text characters, which are NOT null-terminated. Strings shorter than 16 bytes should be padded with spaces (0x20).

# Arduino
Backporting to arduino in progress in arduino_code branch

# Bluetooth
Adding bluetooth work in progress, starting out with simple bluetooth module BK8000L with AT commands support for next/prev/play support through serial, later moving to microchip BM62.
