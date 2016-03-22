#include "extralDefines.h"
#include "spiDriver.h"
#include "uart1.h"
/******************************************************************************/
// global variables
/******************************************************************************/
uint8_t  eepromReadBuffer[EEPROM_BUFFER_SIZE];
uint8_t  eepromWriteBuffer[EEPROM_BUFFER_SIZE];
uint16_t eepromReadBufferCount;
uint16_t eepromWriteBufferCount;


uint8_t k ;

/******************************************************************************/
// local variables
/******************************************************************************/
volatile uint16_t eepromIndex;

uint16_t writeAddressCheck;

uint16_t bytesToWrite0;
uint16_t bytesToWrite1;

/******************************************************************************/
// transfer one byte over SPI
/******************************************************************************/
uint8_t spiTransferByte(uint8_t data)
{
    uint8_t receivedByte;

    while(SPI1_TX_BUFFER_FULL);
    SPI1_INTERRUPT_CLEAR;
    SPI1_FAULT_CLEAR;
    SPI1_OVERFLOW_CLEAR;
    SPI1BUF = data; 
    while(!SPI1_RX_BUFFER_FULL);
    receivedByte = SPI1BUF;

    return receivedByte;
}
/******************************************************************************/
// transfer one byte over SPI2
/******************************************************************************/
uint8_t spi2TransferByte(uint8_t data)
{
    uint8_t receivedByte;

    while(SPI2_TX_BUFFER_FULL);
    SPI2_INTERRUPT_CLEAR;
    SPI2_FAULT_CLEAR;
    SPI2_OVERFLOW_CLEAR;
    SPI2BUF = data; 
    while(!SPI2_RX_BUFFER_FULL);
    receivedByte = SPI2BUF;

    return receivedByte;
}
/******************************************************************************/
// eeprom write enable
/******************************************************************************/
void eepromWriteEnable(void)
{
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WREN);
    EEPROM_CS_HIGH;
}
/******************************************************************************/
// eeprom write disable
/******************************************************************************/
void eepromWriteDisable(void)
{
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WRDI);
    EEPROM_CS_HIGH;
}
/******************************************************************************/
//
/******************************************************************************/
uint8_t eepromReadStatusRegister(void)
{
    uint8_t receivedData;
    
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_RDSR);
    receivedData = spi2TransferByte(0xFF);
    EEPROM_CS_HIGH;
    
    return receivedData;
}
/******************************************************************************/
// write status register: disable protection and write status register bit
/******************************************************************************/
void eepromWriteStatusRegister(uint8_t data)
{
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    eepromWriteEnable();
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WRSR);
    spi2TransferByte(data);
    EEPROM_CS_HIGH;
}
/******************************************************************************/
//
/******************************************************************************/
void eepromRead(uint32_v address, uint16_t bytesToRead)
{
    if(address.dw > EEPROM_TOP_ADDRESS || !bytesToRead)
    {
        return;
    }
    // wait for eeprom to finish any writing tasks
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    EEPROM_CS_LOW;
    // send read command
    spi2TransferByte(EEPROM_READ);
    // send start address 
    spi2TransferByte(address.b[2]);
    spi2TransferByte(address.b[1]);
    spi2TransferByte(address.b[0]);
    // read the data bytes
    eepromReadBufferCount = 0;
    while(eepromReadBufferCount < bytesToRead)
    {
        eepromReadBuffer[eepromReadBufferCount++] = spi2TransferByte(0xFF);
    }
    EEPROM_CS_HIGH;
}
/******************************************************************************/
// to do: check address + bytes to write
// must not exceed page size
/******************************************************************************/
void eepromWrite(uint32_v address, uint16_t bytesToWrite)
{
    if((address.dw > EEPROM_TOP_ADDRESS) || (!bytesToWrite))
    {
        return;
    }
    // check if address + bytesToWrite exceed 255 (write page size)
    writeAddressCheck = address.b[0];
    writeAddressCheck += bytesToWrite;
    if(writeAddressCheck >= EEPROM_PAGE_SIZE)
    {
        bytesToWrite1 = writeAddressCheck - EEPROM_PAGE_SIZE;
        bytesToWrite0 = bytesToWrite - bytesToWrite1;
    }
    else
    {
        bytesToWrite0 = bytesToWrite;
        bytesToWrite1 = 0;
    }
    // wait for the eeprom to finish any ongoing writes
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    // enable eeprom writes
    eepromWriteEnable();
    // send write instruction + 24bit address
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WRITE);
    spi2TransferByte(address.b[2]);
    spi2TransferByte(address.b[1]);
    spi2TransferByte(address.b[0]);
    // send bytes to write
    for(eepromIndex = 0; eepromIndex < bytesToWrite0; eepromIndex++)
    {
        spi2TransferByte(eepromWriteBuffer[eepromIndex]);
    }
    EEPROM_CS_HIGH;
    // leftover bytes that exceed the current address space
    if(bytesToWrite1)
    {
        while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
        eepromWriteEnable();
        address.dw += bytesToWrite0;
        EEPROM_CS_LOW;
        spi2TransferByte(EEPROM_WRITE);
        spi2TransferByte(address.b[2]);
        spi2TransferByte(address.b[1]);
        spi2TransferByte(address.b[0]);
        for(eepromIndex = bytesToWrite0; eepromIndex < bytesToWrite; eepromIndex++)
        {
            spi2TransferByte(eepromWriteBuffer[eepromIndex]);
        }
        EEPROM_CS_HIGH;
    }
}
/******************************************************************************/
//
/******************************************************************************/
void eepromSaveRegister(uint32_v address, uint32_v value)
{
    // wait for the eeprom to finish any ongoing writes
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    // enable eeprom writes
    eepromWriteEnable();
    // send write instruction + 24bit address
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WRITE);
    spi2TransferByte(address.b[2]);
    spi2TransferByte(address.b[1]);
    spi2TransferByte(address.b[0]);
    // send bytes to write
    spi2TransferByte(value.b[0]);
    spi2TransferByte(value.b[1]);
    spi2TransferByte(value.b[2]);
    spi2TransferByte(value.b[3]);
    EEPROM_CS_HIGH;    
}
/******************************************************************************/
//
/******************************************************************************/
void eepromSaveConfig(void)
{
    // wait for the eeprom to finish any ongoing writes
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    // enable eeprom writes
    eepromWriteEnable();
    // send write instruction + 24bit address
    EEPROM_CS_LOW;
    spi2TransferByte(EEPROM_WRITE);
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    // send bytes to write
    for(eepromIndex = 0; eepromIndex < EEADDR_CONFIG_END; eepromIndex++)
    {
//        spi2TransferByte(cfgParamsP.b[eepromIndex]);
    }
    EEPROM_CS_HIGH;
}
/******************************************************************************/
//
/******************************************************************************/
void eepromLoadConfig(void)
{
    // wait for eeprom to finish any writing tasks
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    EEPROM_CS_LOW;
    // send read command
    spi2TransferByte(EEPROM_READ);
    // send start address 
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    spi2TransferByte(EEADDR_SENSOR_TYPE);
    // read the data bytes
    eepromIndex = EEADDR_SENSOR_TYPE;
    while(eepromIndex < EEADDR_CONFIG_END)
    {
//        cfgParamsP.b[eepromIndex++] = spi2TransferByte(0xFF);
    }
    EEPROM_CS_HIGH;    
}
/******************************************************************************/
//
/******************************************************************************/
void eepromClearHistoricalData(void)
{
    uint32_v address;

    for(address.dw = EEPROM_DATA_START; address.dw < EEPROM_SIZE; address.dw += EEPROM_PAGE_SIZE)
    {
        while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
        eepromWriteEnable();
        EEPROM_CS_LOW;
        spi2TransferByte(EEPROM_WRITE);
        spi2TransferByte(address.b[2]);
        spi2TransferByte(address.b[1]);
        spi2TransferByte(address.b[0]);
        // erase one page at a time
        for(eepromIndex = 0; eepromIndex < EEPROM_PAGE_SIZE; eepromIndex++)
        {
            spi2TransferByte(0x00);
        }
        EEPROM_CS_HIGH;
    }
}
/******************************************************************************/
//
/******************************************************************************/
void eepromErase(uint16_t value)
{
    uint32_v address;
    
    for(address.dw = 0; address.dw < EEPROM_SIZE; address.dw += EEPROM_PAGE_SIZE)
    {
        while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
        eepromWriteEnable();
        EEPROM_CS_LOW;
        spi2TransferByte(EEPROM_WRITE);
        spi2TransferByte(address.b[2]);
        spi2TransferByte(address.b[1]);
        spi2TransferByte(address.b[0]);
        // erase one page at a time
        for(eepromIndex = 0; eepromIndex < EEPROM_PAGE_SIZE; eepromIndex++)
        {
            spi2TransferByte(value);
        }
        EEPROM_CS_HIGH;
    }
}
/******************************************************************************/
//
/******************************************************************************/
uint16_t eepromTest(void)
{
    volatile uint32_t idx;
    uint16_t val;
    /*************************************************/
    // Step 1: write 0x00 and read back entire EEPROM 
    /*************************************************/
    eepromErase(0x00);
    // read back entire EEPROM
    idx = 0;
    // wait for EEPROM to finish any writing tasks
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    EEPROM_CS_LOW;
    // send read command
    spi2TransferByte(EEPROM_READ);
    // send start address 
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    // read the data bytes
     UART1PutStr("dsa");
    while(idx < EEPROM_SIZE)
    {
        val = spi2TransferByte(0xFF);
        if(0x00 != val)
        {
            UART1PutStr("zhopa");
            EEPROM_CS_HIGH;
            return false;
        }
        idx++;
    }
    
    EEPROM_CS_HIGH;
    UART1PutStr(":)");
    /*************************************************/
    // Step 2: write 0xFF and read back entire EEPROM 
    /*************************************************/
    eepromErase(0xFF);
    // read back entire EEPROM
    idx = 0;
    // wait for EEPROM to finish any writing tasks
    while(eepromReadStatusRegister() & EEPROM_WRITE_IN_PROGRESS);
    EEPROM_CS_LOW;
    // send read command
    spi2TransferByte(EEPROM_READ);
    // send start address 
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    spi2TransferByte(0x00);
    // read the data bytes
    while(idx < EEPROM_SIZE)
    {
        k= spi2TransferByte(0xFF);
        if(0xFF != k)
        {
            EEPROM_CS_HIGH;
              UART1PutStr(":(");
            return false;
            
          
        }
        idx++;
    }
    EEPROM_CS_HIGH;
    UART1PutStr(":)");
    return true;
}