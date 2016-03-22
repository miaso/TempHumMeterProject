/* 
 * File:   spiDriver.h
 * Author: Dan
 *
 * Created on 29 ianuarie 2015, 15:13
 */

#ifndef SPIDRIVER_H
#define	SPIDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif
/******************************************************************************/
// defines
/******************************************************************************/
#define EEPROM_SIZE                                     262144     // bytes -> 256KBytes
#define EEPROM_PAGE_SIZE                                256        // bytes
#define EEPROM_PAGE_COUNT                               1024
#define EEPROM_TOP_ADDRESS                              EEPROM_SIZE - 1

#define EEPROM_WRITE_IN_PROGRESS                        0x01
#define EEPROM_WRITE_ENABLED                            0x02

#define EEPROM_WREN                                     0x06
#define EEPROM_WRDI                                     0x04
#define EEPROM_RDSR                                     0x05
#define EEPROM_WRSR                                     0x01
#define EEPROM_READ                                     0x03
#define EEPROM_WRITE                                    0x02
#define EEPROM_RDID                                     0x83
#define EEPROM_WRID                                     0x82
#define EEPROM_RDLS                                     0x83
#define EEPROM_LID                                      0x82

/******************************************************************************/
// variables
/******************************************************************************/

/******************************************************************************/
// functions
/******************************************************************************/
uint8_t spiTransferByte(uint8_t data);
uint8_t spi2TransferByte(uint8_t data);

void eepromWriteEnable(void);
void eepromWriteDisable(void);
uint8_t eepromReadStatusRegister(void);
void eepromWriteStatusRegister(uint8_t data);
void eepromRead(uint32_v address, uint16_t bytesToRead);
void eepromWrite(uint32_v address, uint16_t bytesToWrite);
void eepromSaveConfig(void);
void eepromLoadConfig(void);
void eepromSaveRegister(uint32_v address, uint32_v value);
void eepromErase(uint16_t value);
void eepromClearHistoricalData(void);
uint16_t eepromTest(void);
/******************************************************************************/
#ifdef	__cplusplus
}
#endif

#endif	/* SPIDRIVER_H */

