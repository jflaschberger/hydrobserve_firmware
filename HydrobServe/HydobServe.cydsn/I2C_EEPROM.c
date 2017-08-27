/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "I2C_EEPROM.h"


uint8 readOneByteFromEEPROM(uint8 regAdr)
{
   
    CyDelay(2);
    
    //first do a dummy write to set current address to regAdr
    uint8 value;
    
    //clear master status
    //uint8 status = I2CM_I2CMasterClearStatus();  
    //debugI2CMasterStatus(status);
    uint8 indicator = I2CM_I2CMasterSendStart(EEPROM_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(regAdr);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (EEPROM, read), read aborted"); SW_TX_PutCRLF(); 
        
        }
        
    //now send another (re)start condition to initiate reading process
    indicator = I2CM_I2CMasterSendRestart(EEPROM_ADDRESS, I2CM_I2C_READ_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    value = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
    I2CM_I2CMasterSendStop();
    
    return value;
}

float32 readFloat32FromEEPROM(uint8 regAdr)
{
    CyDelay(5);
    
     //first do a dummy write to set current address to regAdr
    uint8 byte1, byte2, byte3, byte4;
    
    //clear master status
    uint8 status = I2CM_I2CMasterClearStatus();  
    //debugI2CMasterStatus(status);
    uint8 indicator = I2CM_I2CMasterSendStart(EEPROM_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(regAdr);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (EEPROM, read), read aborted"); SW_TX_PutCRLF();    
    }
        
    //now send another (re)start condition to initiate reading process
    indicator = I2CM_I2CMasterSendRestart(EEPROM_ADDRESS, I2CM_I2C_READ_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    byte1 = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
    byte2 = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
    byte3 = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
    byte4 = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
    I2CM_I2CMasterSendStop();
    
    //sprintf(debugStr, "float bytes: %i, %i, %i, %i", byte1, byte2, byte3, byte4);
    //SW_TX_PutString(debugStr);
    
    //build float from four bytes
    uint32_t iHelper = byte4<<24 | byte3<<16 | byte2<<8 | byte1;
    float32 f = *((float32*)&iHelper);
    
    return f;
}

    

uint16 readUint16FromEEPROM(uint8 regAdr)
{
    CyDelay(5);
    
     //first do a dummy write to set current address to regAdr
    uint8 byte1, byte2;
    
    //clear master status
    uint8 status = I2CM_I2CMasterClearStatus();  
    //debugI2CMasterStatus(status);
    uint8 indicator = I2CM_I2CMasterSendStart(EEPROM_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(regAdr);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (EEPROM, read), read aborted"); SW_TX_PutCRLF(); 
        
        }
        
    //now send another (re)start condition to initiate reading process
    indicator = I2CM_I2CMasterSendRestart(EEPROM_ADDRESS, I2CM_I2C_READ_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    byte1 = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
    byte2 = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
    I2CM_I2CMasterSendStop();
    
    //build uint16 from two bytes
    uint16 output = byte2<<8 | byte1;
    
    
    return output;
}

/*uint16 readint16FromEEPROM(uint8 regAdr)
{
    CyDelay(5);
    
     //first do a dummy write to set current address to regAdr
    uint8 byte1, byte2;
    
    //clear master status
    uint8 status = I2CM_I2CMasterClearStatus();  
    //debugI2CMasterStatus(status);
    uint8 indicator = I2CM_I2CMasterSendStart(EEPROM_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(regAdr);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (EEPROM, read), read aborted"); SW_TX_PutCRLF();   
    }
        
    //now send another (re)start condition to initiate reading process
    indicator = I2CM_I2CMasterSendRestart(EEPROM_ADDRESS, I2CM_I2C_READ_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    byte1 = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
    byte2 = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
    I2CM_I2CMasterSendStop();
    
    //build uint16 from two bytes
    int16 output = byte2<<8 | byte1;
    
    
    return output;
}*/


void writeOneByteToEEPROM(uint8 regAdr, uint8 value)    
{    
    CyDelay(5);
    
    //clear master status
    uint8 status = I2CM_I2CMasterClearStatus();
    //debugI2CMasterStatus(status);
    
    uint8 indicator = I2CM_I2CMasterSendStart(EEPROM_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    CyDelay(2);
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(regAdr);
        //debugI2CTransfer(indicator);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (EEPROM), write aborted"); SW_TX_PutCRLF(); 
    }
    
    I2CM_I2CMasterWriteByte(value);
    I2CM_I2CMasterSendStop();
    
}

void writeUint16ToEEPROM(uint8 regAdr, uint16 value)
{    
    //first split uint16 into both bytes
    uint8 helperTwoBytes[sizeof(uint16) ];
    memcpy( helperTwoBytes, & value, sizeof(uint16) );
    
    //now write both bytes to EEPROM. This is done by single byte write operations to prevent page rollover (see datasheet)
    writeOneByteToEEPROM(regAdr, helperTwoBytes[0]); 
    writeOneByteToEEPROM(regAdr + 1, helperTwoBytes[1]);
}

void writeFloat32ToEEPROM(uint8 regAdr, float32 value)
{
    //first split float32 into four bytes
    uint8 helperTwoBytes[sizeof(float32) ];
    memcpy( helperTwoBytes, & value, sizeof(float32) );
    
    //now write both bytes to EEPROM. This is done by single byte write operations to prevent page rollover (see datasheet)
    writeOneByteToEEPROM(regAdr, helperTwoBytes[0]); 
    writeOneByteToEEPROM(regAdr + 1, helperTwoBytes[1]);
    writeOneByteToEEPROM(regAdr + 2, helperTwoBytes[2]);
    writeOneByteToEEPROM(regAdr + 3, helperTwoBytes[3]);
}

void debugI2CMasterStatus(uint32 status)
{
    switch(status)
    {        
        case I2CM_I2C_MSTAT_CLEAR:
            SW_TX_PutString("master status is cleared"); SW_TX_PutCRLF(); 
        case I2CM_I2C_MSTAT_RD_CMPLT:
            SW_TX_PutString("Read transfer complete."); SW_TX_PutCRLF(); 
            SW_TX_PutString("The error condition status bits must be checked to ensure that read transfer was completed successfully."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_WR_CMPLT:
            SW_TX_PutString("Write transfer complete."); SW_TX_PutCRLF();
            SW_TX_PutString("The error condition status bits must be checked to ensure that write transfer was completed successfully."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_XFER_INP:
            SW_TX_PutString("Transfer in progress."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_XFER_HALT:
            SW_TX_PutString("Transfer has been halted. The I2C bus is waiting for ReStart or Stop condition generation."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_SHORT_XFER:
            SW_TX_PutString("Error condition: Write transfer completed before all bytes were transferred."); SW_TX_PutCRLF(); 
            SW_TX_PutString("The slave NAKed the byte which was expected to be ACKed."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_ADDR_NAK:
            SW_TX_PutString("Error condition: Slave did not acknowledge address."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_ARB_LOST:
            SW_TX_PutString("Error condition: Master lost arbitration during communications with slave."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_BUS_ERROR:
            SW_TX_PutString("Error condition: bus error occurred during master transfer due to misplaced Start or Stop condition on the bus."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_ABORT_XFER:
            SW_TX_PutString("Error condition: Slave was addressed by another master while master performed the start condition generation. As a result, master has automatically switched to slave mode and is responding. The master transaction has not taken place"); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTAT_ERR_XFER:
            SW_TX_PutString("Error condition: This is the ORed value of all error conditions provided above."); SW_TX_PutCRLF(); break;
        default: SW_TX_PutString("unhandled 12c master status"); SW_TX_PutCRLF();
    }//end switch status
}


void debugI2CTransfer(uint32 inidicator)
{
    switch(inidicator)
    {
        case I2CM_I2C_MSTR_NO_ERROR:
            SW_TX_PutString("Function complete without error."); SW_TX_PutCRLF(); break;
        case I2CM_I2C_MSTR_BUS_BUSY:
            SW_TX_PutString("Bus is busy. Nothing was sent"); SW_TX_PutCRLF();break;
        case I2CM_I2C_MSTR_NOT_READY:
            SW_TX_PutString("Master is not ready for to start transfer."); SW_TX_PutCRLF();break;
        case I2CM_I2C_MSTR_ERR_LB_NAK:
            SW_TX_PutString("Error condition: Last byte was NAKed."); SW_TX_PutCRLF();break;
        case I2CM_I2C_MSTR_ERR_ARB_LOST:
            SW_TX_PutString("Error condition: Master lost arbitration."); SW_TX_PutCRLF();break;
        case I2CM_I2C_MSTR_ERR_BUS_ERR:
            SW_TX_PutString("Error condition: Master encountered a bus error. Bus error is misplaced start or stop detection."); SW_TX_PutCRLF();break;
        case I2CM_I2C_MSTR_ERR_ABORT_START:
            SW_TX_PutString("Error condition: The start condition generation was aborted due to beginning of Slave operation. This error condition is only applicable for Multi-Master-Slave mode."); SW_TX_PutCRLF();break;
        default: SW_TX_PutString("unhandles I2C error condition"); SW_TX_PutCRLF();
    }
}

/* [] END OF FILE */
