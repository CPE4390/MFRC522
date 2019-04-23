/*
 * File:   MFRC522PCD.c
 * Author: Brad McGarvey
 *
 * Created on February 3, 2015, 8:07 AM
 */

#include <xc.h>
#include "MFRC522PCD.h"
#include <string.h>


//These functions are processor specific
//The might need to be modified if using another uController or MSSP1

#if MSSPx == 1
#define SSPxCON1bits    SSPCON1bits
#define SSPxSTATbits    SSPSTATbits
#define SSPxBUF         SSPBUF
#define SPI_TRIS()      TRISCbits.TRISC3=0;TRISCbits.TRISC4=1;TRISCbits.TRISC5=0
//SDO=RC5, SDI=RC4, SCL=RC3
#define SSPxIF          SSPIF
#elif MSSPx == 2
#define SSPxCON1bits    SSP2CON1bits
#define SSPxSTATbits    SSP2STATbits
#define SSPxBUF         SSP2BUF
#define SPI_TRIS()      TRISDbits.TRISD4=0;TRISDbits.TRISD5=1;TRISDbits.TRISD6=0
//SDO=RD4, SDI=RD5, SCL=RD6
#define SSPxIF          SSP2IF
#else
#error Invalid MSSPx selection
#endif

//MRFC522 Registers
#define CommandReg      0x01
#define ComIEnReg       0x02
#define DivIEnReg       0x03
#define ComIrqReg       0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define Status1Reg      0x07
#define Status2Reg      0x08
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0a
#define WaterLevelReg   0x0b
#define ControlReg      0x0c
#define BitFramingReg   0x0d
#define CollReg         0x0e
#define ModeReg         0x11
#define TxModeReg       0x12
#define RxModeReg       0x13
#define TxControlReg    0x14
#define TxASKReg        0x15
#define TxSelReg        0x16
#define RxSelReg        0x17
#define RxThresholdReg  0x18
#define DemodReg        0x19
#define MfTxReg         0x1c
#define MfRxReg         0x1d
#define SerialSpeedReg  0x1f
#define CRCResultRegH   0x21
#define CRCResultRegL   0x22
#define ModWidthReg     0x24
#define RFCfgReg        0x26
#define GsNReg          0x27
#define CWGsPReg        0x28
#define ModGsPReg       0x29
#define TModeReg        0x2a
#define TPrescaleReg    0x2b
#define TReloadRegH     0x2c
#define TReloadRegL     0x2d
#define TCounterValReg  0x2e
#define TestSel1Reg     0x31
#define TestSel2Reg     0x32
#define TestPinEnReg    0x33
#define TestPinValueReg 0x34
#define TestBusReg      0x35
#define AutoTestReg     0x36
#define VersionReg      0x37
#define AnalogTestReg   0x38
#define TestDAC1Reg     0x39
#define TestDAC2Reg     0x3a
#define TestADCReg      0x3b

//MRFC522 Commands
#define CommandIdle         0b0000
#define CommandMem          0b0001
#define CommandRand         0b0010
#define CommandCalcCRC      0b0011
#define CommandTransmit     0b0100
#define CommandNoChange     0b0111
#define CommandReceive      0b1000
#define CommandTransceive   0b1100
#define CommandMFAuthent    0b1110
#define CommandSoftReset    0b1111

//PICC Commands
#define PICC_REQA           0x26
#define PICC_WUPA           0x52
#define PICC_CT             0x88
#define PICC_CL1            0x93
#define PICC_CL2            0x95
#define PICC_CL3            0x97
#define PICC_HLTA           0x50
#define PICC_MF_AUTH_A      0x60
#define PICC_MF_AUTH_B      0x61
#define PICC_MF_READ        0x30
#define PICC_MF_WRITE       0xa0
#define PICC_MF_DECREMENT   0xc0
#define PICC_MF_INCREMENT   0xc1
#define PICC_MF_RESTORE     0xc2
#define PICC_MF_TRANSFER    0xb0
#define PICC_MFUL_WRITE     0xa2

//Misc
#define MF_ACK              0x0a

//Interrupt flags
#define TxIRq       0x40
#define RxIRq       0x20
#define IdleIRq     0x10
#define HiAlertIRq  0x08
#define LoAlertIrq  0x04
#define ErrIRq      0x02
#define TimerIRq    0x01
#define MfinActIRq  0x10
#define CRCIRq      0x04

//Error flags
#define WrErr       0x80
#define TempErr     0x40
#define BufferOvfl  0x10
#define CollErr     0x08
#define CRCErr      0x04
#define ParityErr   0x02
#define ProtocolErr 0x01

//CRC types for doCRC
#define CRC_NONE            0x00
#define CRC_RX              0x01
#define CRC_TX              0x02

//Private prototypes
inline char SPIrw(char b);
char PCDReadRegister(char address);
void PCDReadFIFO(char *buff, char count, char align);
void PCDWriteRegister(char address, char data);
void PCDWriteFIFO(char *buff, char count);
void PCDSetRegisterMask(char address, char mask);
void PCDClearRegisterMask(char address, char mask);
char PCDCommand(char command, char waitIrqs, char *data, char dataLen,
        char *back, char *backLen, char rxAlign, char *validBits, char doCRC);
char PCDCalculateCRC(char *data, char len, char *result);
char PCDGetFIFOLevel(void);
inline void PCDFlushFIFO(void);
inline void PCDAbortCommand(void);
inline void PCDClearIRqs(void);
inline char PCDGetIRqs(void);
char PCDTransmitShortFrame(char command);
void PCDSetTimeout(unsigned int ms);
char PICCSelect(UID *uid);
char PICCSendWUPA(void);
char PICCSendREQA(void);
char MFWrite(char address, char *data);
char MFGetAccessBits(AccessBits *accessBits, char *buffer);
void MFSetAccessBits(AccessBits *accessBits, char *buffer);
char MFValidateAccessBits(char *buffer);
char MFTransfer(char sector, char block);
char MFRestore(char sector, char block);
char MFULWrite(char block, char *data);

inline char SPIrw(char b) {
    SSPxIF = 0;
    SSPxBUF = b;
    while (SSPxIF == 0);
    return SSPxBUF;
}

void PCDInitPins(void) {
    PCD_RESET = 0;
    PCD_CS = 1;
    PCD_TRIS();
}

void InitSPI(void) {
    SPI_TRIS();
    SSPxSTATbits.CKE = 1;
    SSPxCON1bits.CKP = 0;
    SSPxSTATbits.SMP = 0;
    SSPxCON1bits.SSPM = 0b0001;
    SSPxCON1bits.SSPEN = 1;
}

char PCDReadRegister(char address) {
    char r;
    address <<= 1;
    address |= 0b10000000;
    PCD_CS = 0;
    SPIrw(address);
    r = SPIrw(0);
    PCD_CS = 1;
    return r;
}

void PCDWriteRegister(char address, char data) {
    address <<= 1;
    address &= 0b01111111;
    PCD_CS = 0;
    SPIrw(address);
    SPIrw(data);
    PCD_CS = 1;
}

void PCDWriteFIFO(char *buff, char count) {
    char address = FIFODataReg;
    address <<= 1;
    address &= 0b01111111;
    PCD_CS = 0;
    SPIrw(address);
    while (count > 0) {
        SPIrw(*buff);
        --count;
        ++buff;
    }
    PCD_CS = 1;
}

void PCDReadFIFO(char *buff, char count, char align) {
    char address = FIFODataReg;
    char mask;
    char i;
    char value;

    address <<= 1;
    address |= 0b10000000;
    PCD_CS = 0;
    SPIrw(address);
    if (align) {
        mask = 0;
        for (i = align; i <= 7; ++i) {
            mask |= (1 << i);
        }
        value = SPIrw(address);
        *buff &= ~mask;
        *buff |= (value & mask);
        ++buff;
        --count;
    }
    while (count > 1) {
        *buff = SPIrw(address);
        ++buff;
        --count;
    }
    *buff = SPIrw(0);
    PCD_CS = 1;
}

void PCDSetRegisterMask(char address, char mask) {
    char reg;
    reg = PCDReadRegister(address);
    reg |= mask;
    PCDWriteRegister(address, reg);
}

void PCDClearRegisterMask(char address, char mask) {
    char reg;
    reg = PCDReadRegister(address);
    reg &= ~mask;
    PCDWriteRegister(address, reg);
}

void PCDInit(void) {
    PCDInitPins();
    InitSPI();
    PCD_RESET = 1;
    __delay_ms(10);
    //100% ASK
    PCDWriteRegister(TxASKReg, 0x40);
    //Setup timer
    PCDWriteRegister(TModeReg, 0b10000000);
    PCDWriteRegister(TPrescaleReg, 169);
    PCDSetTimeout(15);
    //CRC preset value 0x6363
    PCDWriteRegister(ModeReg, 0x3d);
    //Set threshold levels
    PCDWriteRegister(RxThresholdReg, 0xa2);
    //Antenna on
    PCDAntennaOn();
    PCDSetGain(GAIN_MAX);
}

void PCDReset(void) {
    PCDWriteRegister(CommandReg, CommandSoftReset);
    __delay_ms(10);
    while (PCDReadRegister(CommandReg) & 0b00010000);
}

inline void PCDAntennaOn(void) {
    PCDSetRegisterMask(TxControlReg, 0b00000011);
}

inline void PCDAntennaOff(void) {
    PCDClearRegisterMask(TxControlReg, 0b00000011);
}

void PCDSetGain(char gain) {
    PCDClearRegisterMask(RFCfgReg, 0b01110000);
    PCDSetRegisterMask(RFCfgReg, gain << 4);
}

char PCDGetGain(void) {
    char gain;
    gain = PCDReadRegister(RFCfgReg);
    return (gain >> 4) & 0b00000111;
}

char PCDCommand(char command, char waitIrqs, char *data, char dataLen,
        char *back, char *backLen, char rxAlign, char *validBits, char doCRC) {

    char irqs;
    char bitFrame;
    char txLastBits = 0;
    char rxLastBits;
    char n;
    char status;
    char crcBuffer[4];

    if (doCRC & CRC_TX) {
        status = PCDCalculateCRC(data, dataLen, crcBuffer);
        if (status) {
            return status;
        }
    }
    if (validBits) {
        txLastBits = *validBits;
    }
    bitFrame = (rxAlign << 4) + txLastBits;
    PCDAbortCommand();
    PCDFlushFIFO();
    PCDClearIRqs();
    PCDWriteFIFO(data, dataLen);
    if (doCRC & CRC_TX) {
        PCDWriteFIFO(crcBuffer, 2);
    }
    PCDWriteRegister(BitFramingReg, bitFrame);
    PCDWriteRegister(CommandReg, command);
    if (command == CommandTransceive) {
        PCDSetRegisterMask(BitFramingReg, 0b10000000);
    }
    do {
        irqs = PCDGetIRqs();
        if (irqs & TimerIRq) {
            return STATUS_TIMEOUT;
        }
    } while (!(irqs & waitIrqs));
    status = PCDReadRegister(ErrorReg);
    if (status & 0b11010111) {
        return STATUS_ERROR;
    }
    if (back && backLen) {
        n = PCDGetFIFOLevel();
        if (doCRC & CRC_RX) {
            if (n - 2 > *backLen) {
                return STATUS_BUFFER_SIZE;
            }
            *backLen = n - 2;
        } else {
            if (n > *backLen) {
                return STATUS_BUFFER_SIZE;
            }
            *backLen = n;
        }
        PCDReadFIFO(back, *backLen, rxAlign);
        rxLastBits = PCDReadRegister(ControlReg) & 0b00000111;
        if (validBits) {
            *validBits = rxLastBits;
        }
    }
    if (status & CollErr) {
        return STATUS_COLLISION;
    }
    if (back && backLen) {
        if (n == 1 && rxLastBits == 4) {
            if ((back[0] & 0x0f) == MF_ACK) {
                return STATUS_ACK;
            } else {
                return STATUS_NAK;
            }
        }
        if (doCRC & CRC_RX) {
            if (n < 2 || rxLastBits != 0) {
                return STATUS_CRC_ERROR;
            }
            PCDReadFIFO(crcBuffer, 2, 0);
            status = PCDCalculateCRC(back, n - 2, &crcBuffer[2]);
            if (status != STATUS_OK) {
                return status;
            }
            if (crcBuffer[2] != crcBuffer[0] || crcBuffer[3] != crcBuffer[1]) {
                return STATUS_CRC_ERROR;
            }
        }
    }

    return STATUS_OK;
}

char PCDCalculateCRC(char *data, char len, char *result) {
    char i;
    PCDAbortCommand();
    PCDClearRegisterMask(DivIrqReg, CRCIRq);
    PCDFlushFIFO();
    PCDWriteFIFO(data, len);
    PCDWriteRegister(CommandReg, CommandCalcCRC);
    do {
        i = PCDReadRegister(DivIrqReg);
    } while (!(i & CRCIRq));
    PCDAbortCommand();
    result[0] = PCDReadRegister(CRCResultRegL);
    result[1] = PCDReadRegister(CRCResultRegH);
    return STATUS_OK;
}

char PCDGetFIFOLevel(void) {
    char level;
    level = PCDReadRegister(FIFOLevelReg);
    level &= 0b01111111;
    return level;
}

inline void PCDFlushFIFO(void) {
    PCDWriteRegister(FIFOLevelReg, 0x80);
}

inline void PCDAbortCommand(void) {
    PCDWriteRegister(CommandReg, CommandIdle);
}

inline void PCDClearIRqs(void) {
    PCDWriteRegister(ComIrqReg, 0b01111111);
}

inline char PCDGetIRqs(void) {
    return PCDReadRegister(ComIrqReg);
}

void PCDSetTimeout(unsigned int ms) {
    unsigned int reload;

    reload = ms * 40 - 1;
    //Prescale is 169 - set during init
    //each count is 25us
    PCDWriteRegister(TReloadRegH, reload >> 8);
    PCDWriteRegister(TReloadRegL, reload);
}

char PCDTransmitShortFrame(char command) {
    char bits = 7;
    char size = 2;
    char status;
    char atqa[2];

    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, &command, 1, atqa,
            &size, 0, &bits, CRC_NONE);
    return status;
}

//TODO Test this with long ID's
char PICCSelect(UID *uid) {
    char buffer[9];
    char status;
    char currentBits;
    char cascadeLevel;
    char index;
    char uidIndex;
    char uidComplete;
    char *back;
    char backLen;
    char selected;
    char partialBits;
    char bytes;
    char txBytes;
    char i;

    //Setup for Anticollision loop
    cascadeLevel = 1;
    currentBits = 0;
    uidIndex = 0;
    uidComplete = 0;
    while (!uidComplete) {
        PCDClearRegisterMask(CollReg, 0x80);
        buffer[0] = PICC_CL1 + (cascadeLevel - 1) * 2;
        //Do the anticollision loop
        selected = 0;
        bytes = 0;
        index = 2;
        partialBits = 0;
        while (!selected) {
            if (currentBits == 32) {
                //This is select as all bits are known
                buffer[1] = 0x70;
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                status = PCDCalculateCRC(buffer, 7, &buffer[7]);
                if (status) {
                    return status;
                }
                back = &buffer[6];
                backLen = 3;
                index = 9;
                partialBits = 0;
            } else {
                buffer[1] = (2 + bytes) << 4;
                buffer[1] += partialBits;
                back = &buffer[index];
                backLen = 9 - index;
            }
            txBytes = index;
            if (partialBits) {
                ++txBytes;
            }
            status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer,
                    txBytes, back, &backLen, partialBits, &partialBits, 0);
            if (status == STATUS_COLLISION) {
                i = PCDReadRegister(CollReg);
                if (i & 0x20) {
                    return STATUS_ERROR;
                }
                i &= 0b00011111;
                if (i == 0) {
                    i = 32;
                }
                if (i <= currentBits) {
                    return STATUS_ERROR; //shouldn't happen
                }
                currentBits = i;
                bytes = currentBits / 8;
                partialBits = currentBits % 8;
                index = bytes + 2;
                if (partialBits) {
                    buffer[index] |= (1 << (partialBits - 1));
                } else {
                    buffer[index - 1] |= 0b10000000;
                }
            } else {
                if (status) {
                    return status;
                }
                if (currentBits == 32) {
                    selected = 1;
                } else {
                    currentBits = 32;
                }
            }
        }
        if (backLen != 3 || partialBits != 0) {
            return STATUS_ERROR;
        }
        status = PCDCalculateCRC(back, 1, buffer);
        if (status) {
            return status;
        }
        if (buffer[0] != back[1] || buffer[1] != back[2]) {
            return STATUS_CRC_ERROR;
        }
        i = 0;
        if (buffer[2] == PICC_CT) {
            i = 1;
        }
        for (; i < 4; ++i) {
            uid->bytes[uidIndex] = buffer[2 + i];
            ++uidIndex;
        }
        if (back[0] & 0b00000100) {
            ++cascadeLevel;
            if (cascadeLevel > 3) {
                return STATUS_ERROR;
            }
        } else {
            uid->sak = back[0];
            uidComplete = 1;
        }
    }
    uid->size = 3 * cascadeLevel + 1;
    return STATUS_OK;
}

char PICCFirstId(UID *uid) {
    char status;

    status = PICCSendWUPA();
    if (status) {
        return status;
    }
    return PICCSelect(uid);
}

char PICCNextID(UID *uid) {
    char status;

    status = PICCSendREQA();
    if (status) {
        return status;
    }
    return PICCSelect(uid);
}

//TODO Test this with long ID's
char PICCSelectId(UID *uid) {
    char buffer[9];
    char status;
    char cascadeLevel;
    char index;
    char uidIndex;
    char uidComplete;
    char *back;
    char backLen;
    char bytes;

    status = PICCSendWUPA();
    if (status) {
        return status;
    }
    cascadeLevel = 1;
    uidIndex = 0;
    uidComplete = 0;
    while (!uidComplete) {
        buffer[0] = PICC_CL1 + (cascadeLevel - 1) * 2;
        buffer[1] = 0x70;
        bytes = 4;
        index = 2;
        if ((uid->size > 4 && cascadeLevel == 1) || (uid->size > 7 && cascadeLevel == 2)) {
            buffer[2] = PICC_CT;
            ++index;
            bytes = 3;
        }
        while (bytes) {
            buffer[index] = uid->bytes[uidIndex];
            ++index;
            ++uidIndex;
            --bytes;
        }
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        back = &buffer[6];
        status = PCDCalculateCRC(buffer, 7, &buffer[7]);
        if (status) {
            return status;
        }
        back = &buffer[6];
        backLen = 3;
        status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer,
                9, back, &backLen, 0, 0, CRC_NONE);
        if (status) {
            return status;
        }
        if (backLen != 3) {
            return STATUS_ERROR;
        }
        status = PCDCalculateCRC(back, 1, buffer);
        if (status) {
            return status;
        }
        if (buffer[0] != back[1] || buffer[1] != back[2]) {
            return STATUS_CRC_ERROR;
        }
        if (back[0] & 0b00000100) {
            ++cascadeLevel;
            if (cascadeLevel > 3) {
                return STATUS_ERROR;
            }
        } else {
            uid->sak = back[0];
            uidComplete = 1;
        }
    }
    return STATUS_OK;
}

char PICCHalt(void) {
    char status;
    char buffer[2];
    char len = 1;

    buffer[0] = PICC_HLTA;
    buffer[1] = 0;
    PCDSetTimeout(2);
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer, &len, 0,
            0, CRC_TX);
    PCDSetTimeout(15);
    if (status == STATUS_TIMEOUT) {
        return STATUS_OK; //successful halt means NO communication within 1ms
    } else {
        return status;
    }
}

char PICCSendWUPA(void) {
    char status;

    PCDClearRegisterMask(CollReg, 0x80);
    status = PCDTransmitShortFrame(PICC_WUPA);
    return status;
}

char PICCSendREQA(void) {
    char status;

    PCDClearRegisterMask(CollReg, 0x80);
    status = PCDTransmitShortFrame(PICC_REQA);
    return status;
}

char PICCGetType(UID *uid) {
    if (uid->sak & 0b00000100) {
        return PICC_NOT_COMPLETE;
    }
    switch (uid->sak) {
        case 0x09: return PICC_MIFARE_MINI;
        case 0x08: return PICC_MIFARE_1K;
        case 0x18: return PICC_MIFARE_4K;
        case 0x00: return PICC_MIFARE_UL;
        case 0x10:
        case 0x11:return PICC_MIFARE_PLUS;
    }
    if (uid->sak & 0b00100000) {
        return PICC_ISO_14443_4;
    }
    if (uid->sak & 0b01000000) {
        return PICC_ISO_18092;
    }
    return PICC_UNKNOWN;
}

char MFAuthenticate(char sector, UID *uid, MifareKey *key, char keyType) {
    char buffer[12];

    if (keyType == KEY_A) {
        buffer[0] = PICC_MF_AUTH_A;
    } else if (keyType == KEY_B) {
        buffer[0] = PICC_MF_AUTH_B;
    } else {
        return STATUS_ERROR;
    }
    buffer[1] = sector * 4 + 3;
    memcpy(&buffer[2], key->bytes, 6);
    memcpy(&buffer[8], uid->bytes, 4);
    return PCDCommand(CommandMFAuthent, IdleIRq, buffer, 12, 0, 0, 0, 0, 0);
}

inline void MFStopCrypto1(void) {
    PCDClearRegisterMask(Status2Reg, 0b00001000);
}

char MFReadBlock(char sector, char block, char *data) {
    char len = 16;
    char status;

    data[0] = PICC_MF_READ;
    data[1] = sector * 4 + block;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, data, 2, data,
            &len, 0, 0, CRC_TX | CRC_RX);
    return status;
}

char MFWrite(char address, char *data) {
    char buffer[2];
    char len = 1;
    char status;

    buffer[0] = PICC_MF_WRITE;
    buffer[1] = address;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    len = 1;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, data, 16, buffer,
            &len, 0, 0, CRC_TX);
    if (status == STATUS_ACK) {
        status = STATUS_OK;
    }
    return status;
}

char MFWriteBlock(char sector, char block, char *data) {
    char address;

    address = sector * 4 + block;
    if (address % 4 == 3) {
        return STATUS_ERROR; //This is a sector trailer!
    }
    return MFWrite(address, data);
}

void MFSetAccessBits(AccessBits *accessBits, char *buffer) {
    char c1, c2, c3;

    c1 = ((accessBits->B0 & 4) << 2) | ((accessBits->B1 & 4) << 3) |
            ((accessBits->B2 & 4) << 4) | ((accessBits->B3 & 4) << 5);
    c2 = ((accessBits->B0 & 2) >> 1) | (accessBits->B1 & 2) |
            ((accessBits->B2 & 2) << 1) | ((accessBits->B3 & 2) << 2);
    c3 = ((accessBits->B0 & 1) << 4) | ((accessBits->B1 & 1) << 5) |
            ((accessBits->B2 & 1) << 6) | ((accessBits->B3 & 1) << 7);
    buffer[0] = ((~c2 & 0x0f) << 4) | ((~c1 & 0xf0) >> 4);
    buffer[1] = c1 | ((~c3 & 0xf0) >> 4);
    buffer[2] = (c3 & 0xf0) | (c2 & 0x0f);
}

char MFGetAccessBits(AccessBits *accessBits, char *buffer) {
    //First check format
    if (MFValidateAccessBits(buffer)) {
        return STATUS_ERROR;
    }
    accessBits->B0 = ((buffer[1] & 0b00010000) >> 2)
            | ((buffer[2] & 0b00000001) << 1) | ((buffer[2] & 0b00010000) >> 4);
    accessBits->B1 = ((buffer[1] & 0b00100000) >> 3)
            | (buffer[2] & 0b00000010) | ((buffer[2] & 0b00100000) >> 5);
    accessBits->B2 = ((buffer[1] & 0b01000000) >> 4)
            | ((buffer[2] & 0b00000100) >> 1) | ((buffer[2] & 0b01000000) >> 6);
    accessBits->B3 = ((buffer[1] & 0b10000000) >> 5)
            | ((buffer[2] & 0b00001000) >> 2) | ((buffer[2] & 0b10000000) >> 7);
    return STATUS_OK;
}

char MFValidateAccessBits(char *buffer) {
    if ((buffer[1] & 0x0f) & (buffer[2] >> 4)) {
        return STATUS_ERROR;
    }
    if ((buffer[0] >> 4) & (buffer[2] & 0x0f)) {
        return STATUS_ERROR;
    }
    if ((buffer[0] & 0x0f) & (buffer[1] >> 4)) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

char MFWriteSectorTrailer(char sector, MifareKey *keyA, MifareKey *keyB,
        AccessBits *accessBits) {
    char buffer[16];
    char address;

    address = sector * 4 + 3;
    memcpy(buffer, keyA->bytes, 6);
    MFSetAccessBits(accessBits, &buffer[6]);
    if (MFValidateAccessBits(&buffer[6]) != STATUS_OK) {
        return STATUS_ERROR;
    }
    buffer[9] = 0;
    memcpy(&buffer[10], keyB->bytes, 6);
    return MFWrite(address, buffer);
}

char MFReadSectorTrailer(char sector, MifareKey *keyB, AccessBits *accessBits) {
    char buffer[16];
    char status;

    status = MFReadBlock(sector, 3, buffer);
    if (status) {
        return status;
    }
    if (MFGetAccessBits(accessBits, &buffer[6]) != STATUS_OK) {
        return STATUS_ERROR;
    }
    memcpy(keyB->bytes, &buffer[10], 6);
    return STATUS_OK;
}

char MFWriteValue(char sector, char block, long value) {
    char buffer[16];
    char address;

    address = sector * 4 + block;
    *((long *) buffer) = value;
    *((long *) (&buffer[8])) = value;
    *((long *) (&buffer[4])) = ~value;
    buffer[12] = buffer[14] = address;
    buffer[13] = buffer[15] = ~address;
    return MFWriteBlock(sector, block, buffer);
}

char MFReadValue(char sector, char block, long *value) {
    char buffer[16];
    char status;

    status = MFReadBlock(sector, block, buffer);
    if (status) {
        return status;
    }
    if (*((long *) buffer) != *((long *) (&buffer[8]))) {
        return STATUS_ERROR;
    }
    if (*((long *) buffer) != ~(*((long *) (&buffer[4])))) {
        return STATUS_ERROR;
    }
    *value = *((long *) buffer);
    return STATUS_OK;
}

char MFIncrement(char sector, char block, long offset) {
    char buffer[2];
    char status;
    char address;
    char len = 1;

    address = sector * 4 + block;
    if (address % 4 == 3) {
        return STATUS_ERROR;
    }
    buffer[0] = PICC_MF_INCREMENT;
    buffer[1] = address;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    len = 1;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, (char *) &offset,
            4, buffer, &len, 0, 0, CRC_TX);
    if (status != STATUS_TIMEOUT) { //doesn't ACK so Timeout is success
        return status;
    } else {
        return MFTransfer(sector, block);
    }
}

char MFDecrement(char sector, char block, long offset) {
    char buffer[2];
    char status;
    char address;
    char len = 1;

    address = sector * 4 + block;
    if (address % 4 == 3) {
        return STATUS_ERROR;
    }
    buffer[0] = PICC_MF_DECREMENT;
    buffer[1] = address;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    len = 1;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, (char *) &offset,
            4, buffer, &len, 0, 0, CRC_TX);
    if (status != STATUS_TIMEOUT) {//Decrement doesn't ACK so Timeout is success
        return status;
    } else {
        return MFTransfer(sector, block);
    }
}

char MFTransfer(char sector, char block) {
    char buffer[2];
    char status;
    char address;
    char len = 1;

    address = sector * 4 + block;
    if (address % 4 == 3) {
        return STATUS_ERROR;
    }
    buffer[0] = PICC_MF_TRANSFER;
    buffer[1] = address;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    return STATUS_OK;
}

char MFRestore(char sector, char block) {
    char buffer[2];
    char status;
    char address;
    char len = 1;

    address = sector * 4 + block;
    buffer[0] = PICC_MF_RESTORE;
    buffer[1] = address;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    len = 1;
    buffer[0] = 0; //Restore needs some data so send 0 ???
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 1, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_TIMEOUT) { //doesn't ACK so Timeout is success
        return status;
    } else {
        return STATUS_OK;
    }
}

char MFULReadBlock(char block, char *data) {
    char buffer[16];
    char len = 16;
    char status;

    buffer[0] = PICC_MF_READ;
    buffer[1] = block;
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 2, buffer,
            &len, 0, 0, CRC_TX | CRC_RX);
    if (status) {
        return status;
    }
    memcpy(data, buffer, 4);
    return STATUS_OK;
}

char MFULWrite(char block, char *data) {
    char buffer[6];
    char len = 1;
    char status;

    buffer[0] = PICC_MFUL_WRITE;
    buffer[1] = block;
    memcpy(&buffer[2], data, 4);
    status = PCDCommand(CommandTransceive, RxIRq | IdleIRq, buffer, 6, buffer,
            &len, 0, 0, CRC_TX);
    if (status != STATUS_ACK) {
        return status;
    }
    return STATUS_OK;
}

char MFULWriteBlock(char block, char *data) {
    if (block < 4 || block > 0x27) {
        return STATUS_ERROR; //Not user memory
    }
    return MFULWrite(block, data);
}

char MFULReadCounter(unsigned int *value) {
    char buffer[4];
    char status;

    status = MFULReadBlock(0x29, buffer);
    if (status) {
        return status;
    }
    *value = *((unsigned int *) buffer);
    return STATUS_OK;
}

char MFULWriteCounter(unsigned int value) {
    char buffer[4];

    *((unsigned int *) buffer) = value;
    buffer[2] = buffer[3] = 0;
    return MFULWrite(0x29, buffer);
}

char MFULWriteOTP(char *data) {
    return MFULWrite(0x03, data);
}

char MFULSetLockBytes(char b1, char b2, char b3, char b4) {
    char buffer[4];
    char status;

    status = MFULReadBlock(2, buffer);
    if (status) {
        return status;
    }
    buffer[2] = b1;
    buffer[3] = b2;
    status = MFULWrite(2, buffer);
    if (status) {
        return status;
    }
    buffer[0] = b3;
    buffer[1] = b4;
    buffer[2] = 0;
    buffer[3] = 0;
    status = MFULWrite(0x28, buffer);
    if (status) {
        return status;
    }
    return STATUS_OK;
}

char MFULGetLockBytes(char *b1, char *b2, char *b3, char *b4) {
    char buffer[4];
    char status;

    status = MFULReadBlock(2, buffer);
    if (status) {
        return status;
    }
    *b1 = buffer[2];
    *b2 = buffer[3];
    status = MFULReadBlock(0x28, buffer);
    if (status) {
        return status;
    }
    *b3 = buffer[0];
    *b4 = buffer[1];
    return STATUS_OK;
}

//TODO Finish MFULAuthenticate and MFULSetkey

char PCDSelfTest(char *buffer) {
    char status;

    memset(buffer, 0, 64);
    PCDReset();
    PCDWriteFIFO(buffer, 25);
    PCDWriteRegister(CommandReg, CommandMem);
    PCDWriteRegister(AutoTestReg, 0x09);
    PCDClearIRqs();
    PCDWriteFIFO(buffer, 1);
    PCDWriteRegister(CommandReg, CommandCalcCRC);
    while ((PCDGetIRqs() & IdleIRq) == 0);
    status = PCDReadRegister(ErrorReg);
    PCDReadFIFO(buffer, 64, 0);
    return status;
}
