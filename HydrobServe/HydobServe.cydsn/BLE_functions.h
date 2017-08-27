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
#include "common.h"

CYBLE_CONN_HANDLE_T  connectionHandle;
CYBLE_API_RESULT_T apiResult;
CYBLE_LP_MODE_T lpMode;


void StackEventHandler( uint32 eventCode, void *eventParam );

/**********************************************************
    reading from BLE stack
 **********************************************************/
uint16 readUint16FromBLEdb(CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);

uint8 readUint8FromBLEdb(CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);



/**********************************************************
    writing to BLE stack
 **********************************************************/
//writes one uint8 value to ble stack
void writeUint8ToBLEdb(uint8 Value, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);

//writes one uint16 value to ble stack
void writeUint16ToBLEdb(uint16 firstValue, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);

//writes float32 value to ble stack 
void writeFloat32ToBLEdb(float32 value, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);

//writes 22 character array to ble stack 
void write22CharsToBLEdb(uint8* ptrFirstCell, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle);

/**********************************************************
    send notifications
 **********************************************************/
/*DUMMY
void notifySomething();*/

/* [] END OF FILE */
