/* 
 * File:   MFRC522PCD.h
 * Author: Brad McGarvey
 *
 * Created on February 3, 2015, 8:07 AM
 */

#ifndef MFRC522PCD_H
#define	MFRC522PCD_H

//uController pin assignments
/*
 Pin assignments
    RB3 = IRQ
    RD3 = CS (SDA)
    RD4 = SDO connect to MOSI  for MSSP2
    RD5 = SDI connect to MISO  for MSSP2
    RD6 = SCK for MSSP2
    RD7 = RESET
 */

//Hardware dependent defines
#define _XTAL_FREQ      32000000L

#define PCD_RESET       LATDbits.LATD7
#define PCD_CS          LATDbits.LATD3
//Set tris bits for the above pins only - SPI pins are handled separately
#define PCD_TRIS()      TRISD &= 0b01110111; //Set tris bits for the above pins

#define MSSPx           2   //1 or 2 to select MSSP1 or MSSP2 for SPI

//PCD Gain settings
#define GAIN_18DB           0b000
#define GAIN_23DB           0b001
#define GAIN_33DB           0b100
#define GAIN_38DB           0b101
#define GAIN_43DB           0b110
#define GAIN_48DB           0b111
#define GAIN_MIN            0b000
#define GAIN_MID            0b100
#define GAIN_MAX            0b111

//PCD Baud rate
#define BAUD_106            0b000
#define BAUD_212            0b001
#define BAUD_424            0b010
#define BAUD_848            0b011

//PICC SAK Types
#define PICC_UNKNOWN        0x00
#define PICC_ISO_14443_4    0x01
#define PICC_MIFARE_MINI    0x02
#define PICC_MIFARE_1K      0x03
#define PICC_MIFARE_4K      0x04
#define PICC_MIFARE_UL      0x05
#define PICC_MIFARE_PLUS    0x06
#define PICC_ISO_18092      0x07
#define PICC_NOT_COMPLETE   0x80

//Mifare access bit constants
#define ACCESS_BITS_TRANSPORT       0b001
#define ACCESS_BITS_KEYB            0b011
#define BLOCK_TRANSPORT             0b000
#define BLOCK_READ_AB               0b010
#define BLOCK_READ_AB_WRITE_B       0b100
#define BLOCK_READ_B_WRITE_B        0b011
#define BLOCK_READ_B                0b101
#define BLOCK_NONE                  0b111
#define VALUE_DEC_AB_INC_B          0b110
#define VALUE_DEC_AB_NO_W_OR_INC    0b001

//Error Codes
#define STATUS_OK           0
#define STATUS_ERROR        1
#define STATUS_CRC_ERROR    2
#define STATUS_COLLISION    3
#define STATUS_TIMEOUT      4
#define STATUS_NAK          5
#define STATUS_ACK          6
#define STATUS_BUFFER_SIZE  7

//Mifare classic key A or B
#define KEY_A   0
#define KEY_B   1

//Structure for UID's up to 10 bytes - sak is returned at select
typedef struct {
    char size;
    char bytes[10];
    char sak;
} UID;

//Mifare classic key
typedef struct {
    char bytes[6];
} MifareKey;

//Mifare Ultralight Key
typedef struct {
    char bytes[4];
} MifareULKey;

//Access bits for Mifare classic sector trailer
typedef struct {
    char B0;
    char B1;
    char B2;
    char B3;
} AccessBits;

//PCD Functions
void PCDInit(void);
void PCDReset(void);
inline void PCDAntennaOn(void);
inline void PCDAntennaOff(void);
void PCDSetGain(char gain);
char PCDGetGain(void);

//PICC functions
char PICCFirstId(UID *id);
char PICCNextId(UID *uid);
char PICCHalt(void);
char PICCGetType(UID *uid);
char PICCSelectId(UID *uid);

//Mifare classic functions
char MFReadBlock(char sector, char block, char *data);
char MFWriteBlock(char sector, char block, char *data);
char MFWriteValue(char sector, char block, long value);
char MFReadValue(char sector, char block, long *value);
char MFIncrement(char sector, char block, long offset);
char MFDecrement(char sector, char block, long offset);
char MFWriteSectorTrailer(char sector, MifareKey *keyA, MifareKey *keyB, AccessBits *accessBits);
char MFReadSectorTrailer(char sector, MifareKey *keyB, AccessBits *accesBits);
char MFAuthenticate(char sector, UID *uid, MifareKey *key, char keyType);
inline void MFStopCrypto1(void);

//Mifare Ultralight and NTAG203 functions
char MFULReadBlock(char block, char *data);
char MFULWriteBlock(char block, char *data);
char MFULReadCounter(unsigned int *value);
char MFULWriteCounter(unsigned int value);
char MFULWriteOTP(char *data);
char MFULSetLockBytes(char b1, char b2, char b3, char b4);
char MFULGetLockBytes(char *b1, char *b2, char *b3, char *b4);

//Mifare Ultralight functions
char MFULSetKey(MifareULKey *key);
char MFULAuthenticate(MifareULKey *key);

//Misc
char PCDSelfTest(char *buffer);

#endif	/* MFRC522PCD_H */

