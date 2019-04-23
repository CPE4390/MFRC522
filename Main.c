/* 
 * File:   Main.c
 * Author: bmcgarvey
 *
 * Created on January 16, 2015, 8:29 AM
 */

#include <xc.h>
#include "LCD.h"
#include "MFRC522PCD.h"
#include <stdio.h>

char lcd[20];

UID uid = {4, {0x34, 0x4b, 0x80, 0x2e}, 0};
MifareKey keyA = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
MifareKey keyB = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char blockData[16];
AccessBits ab = {BLOCK_TRANSPORT, BLOCK_TRANSPORT, BLOCK_TRANSPORT, ACCESS_BITS_TRANSPORT};
char testBuffer[64];

void InitSystem(void);

void main(void) {
    char r1, r2;
    long i;

    char sector = 1;
    char block = 0;

    InitSystem();
    LCDInit();
    LCDWriteLine("RFID", 0);
    PCDInit();
    while (1) {
        do {
            r1 = PICCFirstId(&uid);
            //r1 = PICCSetUID(&uid);
            if (r1 == STATUS_OK) {
                sprintf(lcd, "%02x %02x %02x %02x", uid.bytes[0], uid.bytes[1], uid.bytes[2], uid.bytes[3]);
                LCDWriteLine(lcd, 1);
            } else {
                LCDClearLine(1);
                sprintf(lcd, "%02x", r1);
                LCDWriteLine(lcd, 1);
            }
        } while (r1 != STATUS_OK);

        r1 = MFAuthenticate(sector, &uid, &keyA, KEY_A);
        r2 = MFReadBlock(sector, block, blockData);
        sprintf(lcd, "%02x%02x%02x%02x%02x", blockData[0], blockData[1], blockData[2], blockData[3], blockData[5]);
        LCDWriteLine(lcd, 0);
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

void interrupt HighISR(void) {

}

