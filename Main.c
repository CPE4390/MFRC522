/* 
 * File:   Main.c
 * Author: bmcgarvey
 *
 * Created on January 16, 2015, 8:29 AM
 */

#include <xc.h>
#include "LCD.h"
#include "MFRC522PCD.h"

UID uid = {4, {0x34, 0x4b, 0x80, 0x2e}, 0};
MifareKey keyA = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
MifareKey keyB = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char blockData[16];
AccessBits ab = {BLOCK_TRANSPORT, BLOCK_TRANSPORT, BLOCK_TRANSPORT, ACCESS_BITS_TRANSPORT};
char testBuffer[64];

void InitSystem(void);

void main(void) {
    char r1, r2;
    char sector = 1;
    char block = 0;

    InitSystem();
    LCDInit();
    lprintf(0, "RFID");
    PCDInit();
    while (1) {
        do {
            r1 = PICCFirstId(&uid);
            if (r1 == STATUS_OK) {
                lprintf(1, "%02x %02x %02x %02x", uid.bytes[0], uid.bytes[1], uid.bytes[2], uid.bytes[3]);
            } else {
                LCDClearLine(1);
                lprintf(1, "%02x", r1);
            }
        } while (r1 != STATUS_OK);

        r1 = MFAuthenticate(sector, &uid, &keyA, KEY_A);
        r2 = MFReadBlock(sector, block, blockData);
        lprintf(0, "%02x%02x%02x%02x%02x", blockData[0], blockData[1], blockData[2], blockData[3], blockData[5]);
        while (PORTBbits.RB0);
        __delay_ms(10);
        while (!PORTBbits.RB0);
        __delay_ms(10);
        PICCHalt();
        MFStopCrypto1();
    }
}

void InitSystem(void) {
    OSCTUNEbits.PLLEN = 1;
    TRISBbits.RB0 = 1;
}

void __interrupt(high_priority) HighISR(void) {

}

