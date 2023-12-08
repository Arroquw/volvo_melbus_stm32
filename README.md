# volvo_melbus_stm32
Based on https://github.com/visualapproach/Volvo-melbus

Implements CD changer and MD changer on the melbus. CD changer does not support displaying text on the screen, MD changer does.

On MD changer mode, press the "Mem" button on the radio to display text, and press it again to go back to displaying "MDxx xx".

Next, prev, text are all implemented in non practical ways, as I don't have a bluetooth module connected to it yet (see bluetooth section).

# Internal logic
I removed SAT code from the original arduino code as there is no way for me to test it (don't have an HU-x50, only x03). Should be easy to re-add to main.c though, just needs some changing of the SendText and SendByteToMelbus2 functions in master.c to accomodate for the fact that SAT does not pull the BUSY pin low (apparently/allegedly).

All the logic for the bus itself is done in Core/main.c, all the other files contain functions that are for supporting the base protocol bitbanging. Adding a new command is as simple as adding the definition for it to main.h, adding it to the commands array in main.c and expanding the enum in main.c with your entry.

# Sending text
Sending text is done in the MRB/MRB_2 cases in the main switch in main.c, and adding text is done in the MD_RTR_x case. MD_RTR_1 requests row 1, MD_RTR_2 requests row 2 and MD_RTR_3 requests row 3. Each row can contain up to 16 text characters, which are NOT null-terminated. Strings shorter than 16 bytes should be padded with spaces (0x20).

# Arduino
Backporting to arduino in progress in arduino_code branch

# Bluetooth
Adding bluetooth work in progress, starting out with simple bluetooth module BK8000L with AT commands support for next/prev/play support through serial, later moving to microchip BM62.

# Running and hardware

 -- microcontroller
    I used an STM32F411CEU6, on a WeAct studio blackpill module.

I connected the 8 pin DIN on the back of the head unit to a circuit board, with a buck converter module connected to external 12V which I routed from my fusebox next to the driver seat, and ground connected to the DIN plug shell which goes into the HU.

 -- pins
    CLK - Pin A2 with 100 ohms resistor
    DATA - Pin A3 with 100 ohms resistor
    BUSY - Pin A4 with 100 ohms resistor
    3v3 connected to buck converter with a diode for protection
    GND connected to DIN plug shell
    Pin B3 routed out for SWD debug prints
    
# Data sniffing
I also made a simple program that uses an external stm32's USART to print out all the data it receives on the melbus, code can be found here: https://github.com/Arroquw/melbus_sniffer
I found this to be really helpful to decode the master mode that the internal CD player that my HU has uses, since it uses master mode the exact same way as an MD chgr.

