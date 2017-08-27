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
#include "BLE_functions.h"
#include "I2C_EEPROM.h"


/**********************************************************
    reading from BLE stack
 **********************************************************/
float32 floatFromBytes(uint8 byte1, uint8 byte2, uint8 byte3, uint8 byte4)
{
    //build float from four bytes
    uint32_t iHelper = byte4<<24 | byte3<<16 | byte2<<8 | byte1;
    float32 f = *((float32*)&iHelper);
    return f;
}

uint8 readUint8FromBLEdb(CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    uint8 returnVal;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T  readHandle;     //handle to interface with BLE database
    
    // first get relevant Data from BLE database and convert it to suiting data format
    readHandle.attrHandle = characteristicHandle;  // Attribute Handle of the characteristic in GATT Database. Taken from BLE_custom.h
    
    readHandle.value.val = returnVal;                                       
    readHandle.value.len = sizeof(1);         
    
    //read out DB: uint16 value ia put into helperArray
    CyBle_GattsReadAttributeValue(&readHandle, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
    
    return returnVal;
}

uint16 readUint16FromBLEdb(CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    char helperArray[2];                            //stores the 2 bytes of an uint16 value
    uint16 returnVal;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T  readHandle;     //handle to interface with BLE database
    
    // first get relevant Data from BLE database and convert it to suiting data format
    readHandle.attrHandle = characteristicHandle;  // Attribute Handle of the characteristic in GATT Database. Taken from BLE_custom.h
    
    readHandle.value.val = helperArray;                                       
    readHandle.value.len = sizeof(2);         
    
    //read out DB: uint16 value ia put into helperArray
    CyBle_GattsReadAttributeValue(&readHandle, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
    
    //get content out of helperArray and convert to uint16
    returnVal = (helperArray[0] << 8) | helperArray[1];
    return returnVal;
}


float32 readFloat32FromBLEdb(CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    char helperArray[4];                            //stores the 2 bytes of an uint16 value
    float32 returnVal;
    CYBLE_GATT_HANDLE_VALUE_PAIR_T  readHandle;     //handle to interface with BLE database
    
    // first get relevant Data from BLE database and convert it to suiting data format
    readHandle.attrHandle = characteristicHandle;  // Attribute Handle of the characteristic in GATT Database. Taken from BLE_custom.h
    
    readHandle.value.val = helperArray;                                       
    readHandle.value.len = sizeof(4);         
    
    //read out DB: uint16 value ia put into helperArray
    CyBle_GattsReadAttributeValue(&readHandle, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
    
    //get content out of helperArray and convert to uint16
    returnVal = floatFromBytes(helperArray[0],helperArray[1],helperArray[2],helperArray[3]);
    return returnVal;
}

/*
//read string. so far nothing is done with it. Easiest idea: store into global variable. But no Idea if used in the end
void readStringFromBLEdb()
{
    CYBLE_GATT_HANDLE_VALUE_PAIR_T  readHandle;     //handle to interface with BLE database
    char userName[22];  
    
    // first get relevant Data from BLE database and convert it to suiting data format
    readHandle.attrHandle = CYBLE_USER_DATA_FIRST_NAME_OF_USER_CHAR_HANDLE;       Attribute Handle of the characteristic in GATT Database. Taken from BLE_custom.h
    readHandle.value.val = userName;                                       
    readHandle.value.len = sizeof(userName);         
    
    //read out DB:  values ia put into helperArray
    CyBle_GattsReadAttributeValue(&readHandle, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);  
}*/



/**********************************************************
    writing to BLE stack
 **********************************************************/

//writes 22 character array to ble stack 
void write22CharsToBLEdb(uint8* ptrFirstCell, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
    dataHandle.attrHandle = characteristicHandle;
    dataHandle.value.val = ptrFirstCell;
    dataHandle.value.len = 22;
    
    // Update the attribute value. This will allow Client device to read the existing  values via the characteristic 
    CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
}

void writeUint16ToBLEdb(uint16 firstValue, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    uint8 helperByteArray[2];
    uint8 helperTwoBytes[sizeof(uint16) ];
    
    //copy storage of first uint16 value as bytes and pack them into helper array
    memcpy( helperTwoBytes, & firstValue, sizeof(uint16) );
    helperByteArray[0] = helperTwoBytes[0];
    helperByteArray[1] = helperTwoBytes[1];
    
    uint8* data = &helperByteArray[0];
    
    //update characteristics.  find handles as defines in "BLE_custom.h"
    CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
    dataHandle.attrHandle = characteristicHandle;
    dataHandle.value.val = data;
    dataHandle.value.len = 2;
    
    // Update the attribute value. This will allow Client device to read the existing  values via the characteristic 
    CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
}


void writeUint8ToBLEdb(uint8 Value, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    /*
    uint8 helperByteArray[2];
    uint8 helperTwoBytes[sizeof(uint16) ];
    
    //copy storage of first uint16 value as bytes and pack them into helper array
    memcpy( helperTwoBytes, & Value, sizeof(uint16) );
    helperByteArray[0] = helperTwoBytes[0];
    helperByteArray[1] = helperTwoBytes[1];
    */
    uint8* data = &Value;
    
    //update characteristics.  find handles as defines in "BLE_custom.h"
    CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
    dataHandle.attrHandle = characteristicHandle;
    dataHandle.value.val = data;
    dataHandle.value.len = 1;
    
    // Update the attribute value. This will allow Client device to read the existing  values via the characteristic 
    CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
}

//writes float32 value to ble stack 
void writeFloat32ToBLEdb(float32 value, CYBLE_GATT_DB_ATTR_HANDLE_T  characteristicHandle)
{
    uint8 helperByteArray[4];
    uint8 helperTwoBytes[sizeof(float32) ];
    
    //copy storage of first uint16 value as bytes and pack them into helper array
    memcpy( helperTwoBytes, & value, sizeof(float32) );
    helperByteArray[0] = helperTwoBytes[0];
    helperByteArray[1] = helperTwoBytes[1];
    helperByteArray[2] = helperTwoBytes[2];
    helperByteArray[3] = helperTwoBytes[3];
   
    //set pointer to first element
    uint8* data = &helperByteArray[0];
    
    //update characteristics.  find handles as defines in "BLE_custom.h"
    CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
    dataHandle.attrHandle = characteristicHandle;
    dataHandle.value.val = data;
    dataHandle.value.len = 4;
    
    // Update the attribute value. This will allow Client device to read the existing  values via the characteristic  
    CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
}



/**********************************************************
    send notifications
 **********************************************************/
/* DUMMY
void notifySomething()
{
    //prepare data to be sent
    uint8_t msg = 0;
    
    // 'notificationHandle' is handle to store notification data parameters  
    CYBLE_GATTS_HANDLE_VALUE_NTF_T notificationHandle; 
    
    // Update Notification handle with new data
    notificationHandle.attrHandle = CYBLE_DEVICE_INFORMATION_CUP_TYPE_CHAR_HANDLE;
    
    notificationHandle.value.val = &msg;
    notificationHandle.value.len = sizeof(msg); 
    
    // Report data to BLE component for sending data by notifications 
    CyBle_GattsNotification(cyBle_connHandle, &notificationHandle); 
} */




void wakeUpBLE()
{
    #if BLE_DEBUG
        UART_PutString("trying to wake up BLE"); UART_PutCRLF();
    #endif    
    lpMode = CyBle_ExitLPM();
    #if BLE_DEBUG
        if(lpMode == CYBLE_BLESS_ACTIVE)
        {
            UART_PutString("BLE successfully woken up"); UART_PutCRLF();
        }
    #endif
}


  
void handleIndications()
{
    /*question like
    if((rscIndicationState == ENABLED) && (YES == rscIndicationPending))
            {
                HandleRscIndications();
            }
    */
    
    //...do something like
    /*
    */
}



/***************************************************************************
* BLE Event handler
***************************************************************************/

void StackEventHandler( uint32 eventCode, void *eventParam )
{
    /* Structure to store data written by Client */ 
    CYBLE_GATTS_WRITE_REQ_PARAM_T *wrReqParam; 
    
    switch( eventCode )
    {
        /* Generic events */

        case CYBLE_EVT_HOST_INVALID:
        break;

        case CYBLE_EVT_STACK_ON:
            /* CyBle_GappStartAdvertisement( CYBLE_ADVERTISING_FAST ); */
        break;

        case CYBLE_EVT_TIMEOUT:
        break;

        case CYBLE_EVT_HARDWARE_ERROR:
        break;

        case CYBLE_EVT_HCI_STATUS:
        break;

        case CYBLE_EVT_STACK_BUSY_STATUS:
        break;

        case CYBLE_EVT_PENDING_FLASH_WRITE:
        break;


        /* GAP events */

        case CYBLE_EVT_GAP_AUTH_REQ:
        break;

        case CYBLE_EVT_GAP_PASSKEY_ENTRY_REQUEST:
        break;

        case CYBLE_EVT_GAP_PASSKEY_DISPLAY_REQUEST:
        break;

        case CYBLE_EVT_GAP_AUTH_COMPLETE:
        break;

        case CYBLE_EVT_GAP_AUTH_FAILED:
        break;

        case CYBLE_EVT_GAP_DEVICE_CONNECTED:
        break;

        case CYBLE_EVT_GAP_DEVICE_DISCONNECTED:
            /* CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST); */
        break;

        case CYBLE_EVT_GAP_ENCRYPT_CHANGE:
        break;

        case CYBLE_EVT_GAP_CONNECTION_UPDATE_COMPLETE:
        break;

        case CYBLE_EVT_GAP_KEYINFO_EXCHNGE_CMPLT:
        break;


        /* GAP Peripheral events */

        case CYBLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            //if(CyBle_GetState() != CYBLE_STATE_CONNECTED && CyBle_GetState() != CYBLE_STATE_ADVERTISING)
            //CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
        break;


        /* GAP Central events */

        case CYBLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
        break;

        case CYBLE_EVT_GAPC_SCAN_START_STOP:
        break;


        /* GATT events */

        case CYBLE_EVT_GATT_CONNECT_IND:
        break;

        case CYBLE_EVT_GATT_DISCONNECT_IND:
        break;


        /* GATT Client events (CYBLE_EVENT_T) */

        case CYBLE_EVT_GATTC_ERROR_RSP:
        break;

        case CYBLE_EVT_GATTC_XCHNG_MTU_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BY_GROUP_TYPE_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BY_TYPE_RSP:
        break;

        case CYBLE_EVT_GATTC_FIND_INFO_RSP:
        break;

        case CYBLE_EVT_GATTC_FIND_BY_TYPE_VALUE_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_BLOB_RSP:
        break;

        case CYBLE_EVT_GATTC_READ_MULTI_RSP:
        break;

        case CYBLE_EVT_GATTC_WRITE_RSP:
        break;

        case CYBLE_EVT_GATTC_EXEC_WRITE_RSP:
        break;

        case CYBLE_EVT_GATTC_HANDLE_VALUE_NTF:
        break;

        case CYBLE_EVT_GATTC_HANDLE_VALUE_IND:
        break;


        /* GATT Client events (CYBLE_EVT_T) */

        case CYBLE_EVT_GATTC_INDICATION:
        break;

        case CYBLE_EVT_GATTC_SRVC_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_INCL_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_CHAR_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_DESCR_DISCOVERY_FAILED:
        break;

        case CYBLE_EVT_GATTC_SRVC_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_CHAR_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_DESCR_DUPLICATION:
        break;

        case CYBLE_EVT_GATTC_SRVC_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_INCL_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_CHAR_DISCOVERY_COMPLETE:
        break;

        case CYBLE_EVT_GATTC_DISCOVERY_COMPLETE:
        break;


        /* GATT Server events (CYBLE_EVENT_T) */

        case CYBLE_EVT_GATTS_XCNHG_MTU_REQ:
        break;

        case CYBLE_EVT_GATTS_WRITE_REQ:
        /* This event is generated when the connected Central device sends a Write request. */ 
            /* The parameter ‘eventParam’ contains the data written */
            
            /* Extract the Write data sent by Client */ 
            wrReqParam = (CYBLE_GATTS_WRITE_REQ_PARAM_T *) eventParam;
            
            // Store data locally  
            uint8* data = wrReqParam->handleValPair.value.val;
            
            //update characteristics.
            CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
            dataHandle.attrHandle = wrReqParam->handleValPair.attrHandle;
            dataHandle.value.val = data;
            //dataHandle.value.len = sizeof(*data);
            dataHandle.value.len = wrReqParam->handleValPair.value.len;
                        
                        
            /* Update the attribute value. This will allow Client device to read the existing  values via the characteristic */ 
            CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);
            
            uint8 i = 0;
            uint8 helperArray[45];               //holds input data
            
            //fill data bytes to array
            uint8 length = wrReqParam->handleValPair.value.len;
            for(i=0;i<length;i++)
            {
                helperArray[i] = *(data +i);
            }
            
            
            // If the attribute handle of the characteristic written to 
            // is equal to that of the aimed characteristic, then do individual things
            
            switch(wrReqParam->handleValPair.attrHandle)
            {
                /*
                case CYBLE_NETWORK_DATA_WIFI_NAME_CHAR_HANDLE:     //wifi name
                {
                    //build send string
                    char helpStr[length + 6+1]; //dont forget cell for terminating character
                    helpStr[0] = 'W';
                    helpStr[1] = 'N';
                    helpStr[2] = 'a';
                    helpStr[3] = 'm';
                    helpStr[4] = 'e';
                    helpStr[5] = '~';
                    
                    for(i=6; i<6+length; i++)     
                    {
                        helpStr[i] = helperArray[i-6];
                    }
                    //terminate String
                    helpStr[6+length] = 0;
                    //SW_TX_PutString("now sending to wifi: "); SW_TX_PutString(helpStr); 
                    //pipe data to wifi module
                    writeMessageToWIFI(helpStr);
                    
                    
                    break;
                }*/
                
                case CYBLE_VOLUME_DATA_DRUNK_VOLUME_CHAR_HANDLE:     
                {
                    //set to zero directly, no reading of written data!
                    overall_volume = 0;
                    overall_pulses = 0;
                    num_drink_events = 0;
                    SW_TX_PutString("Volume, pulses and evt cnt set to zero"); SW_TX_PutCRLF();
                    break;
                }
                
                case CYBLE_VOLUME_DATA_PULSE_COUNT_CHAR_HANDLE:   
                {
                    //set to zero directly, no reading of written data!
                    overall_volume = 0;
                    overall_pulses = 0;
                    num_drink_events = 0;
                    SW_TX_PutString("Volume, pulses and evt cnt set to zero"); SW_TX_PutCRLF();
                    break;
                }
                
                case CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_CHAR_HANDLE:
                {
                    //I2CM_Start(); CyDelay(5);
                    //pack new value into global variable
                    hall_multiplier = readFloat32FromBLEdb(CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_CHAR_HANDLE);
                    
                    //save it to EEPROM
                    writeFloat32ToEEPROM(EEPROM_MULTIPLIER, hall_multiplier);
                    //I2CM_Stop();
                    break;                    
                }
                
                case CYBLE_USER_DATA_GENER_CHAR_HANDLE:
                {
                    //I2CM_Start(); CyDelay(5);
                    //pack new value into variable
                    gender = *data;
                    //save to EEPROM
                    writeOneByteToEEPROM(EEPROM_GENDER, gender);
                    //I2CM_Stop();
                    break;
                }
                
                case CYBLE_USER_DATA_USER_ID_CHAR_HANDLE:
                {
                    //I2CM_Start(); CyDelay(5);
                    //SW_TX_PutString("Got user data id via BLE:"); SW_TX_PutCRLF();
                    for(i=0;i<length;i++)
                    {
                        userID[i] = helperArray[i];                         //pack it into variable ...
                        writeOneByteToEEPROM(EEPROM_ID+i, helperArray[i]);  //...ynd into EEPROM
                        
                        //SW_TX_PutString(userID[i]); SW_TX_PutCRLF();
                       // sprintf(debugStr, " %c=%i ", helperArray[i], helperArray[i]);
                        //SW_TX_PutString(debugStr);
                    }
                    //I2CM_Stop();
                    break;
                }
                case CYBLE_USER_DATA_FIRST_NAME_CHAR_HANDLE:
                {          
                    //I2CM_Start(); CyDelay(5);
                    for(i=0;i<length;i++)
                    {
                        firstName[i] = helperArray[i];                         //pack it into variable ...
                        writeOneByteToEEPROM(EEPROM_FIRSTNAME+i, helperArray[i]);  //...ynd into EEPROM
                    }
                    break;
                    //I2CM_Stop();
                }
                case CYBLE_USER_DATA_LAST_NAME_CHAR_HANDLE:
                {
                    //I2CM_Start(); CyDelay(5);
                    for(i=0;i<length;i++)
                    {
                        lastName[i] = helperArray[i];                         //pack it into variable ...
                        writeOneByteToEEPROM(EEPROM_LASTNAME+i, helperArray[i]);  //...and into EEPROM
                    }
                    break;
                    //I2CM_Stop();
                }
                
                
                //other cases from bleCustom.h
                default: SW_TX_PutString("BLE write request to unhandled write handle");SW_TX_PutCRLF();
            }//end switch write request
            
            /* Send the response to the write request received. */ 
            CyBle_GattsWriteRsp(cyBle_connHandle);
        break;

        case CYBLE_EVT_GATTS_WRITE_CMD_REQ:
        break;

        case CYBLE_EVT_GATTS_PREP_WRITE_REQ:
        break;

        case CYBLE_EVT_GATTS_EXEC_WRITE_REQ:
        break;

        case CYBLE_EVT_GATTS_HANDLE_VALUE_CNF:
        break;

        case CYBLE_EVT_GATTS_DATA_SIGNED_CMD_REQ:
        break;


        /* GATT Server events (CYBLE_EVT_T) */

        case CYBLE_EVT_GATTS_INDICATION_ENABLED:
        break;

        case CYBLE_EVT_GATTS_INDICATION_DISABLED:
        break;


        /* L2CAP events */

        case CYBLE_EVT_L2CAP_CONN_PARAM_UPDATE_REQ:
        break;

        case CYBLE_EVT_L2CAP_CONN_PARAM_UPDATE_RSP:
        break;

        case CYBLE_EVT_L2CAP_COMMAND_REJ:
        break;

        case CYBLE_EVT_L2CAP_CBFC_CONN_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_CONN_CNF:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DISCONN_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DISCONN_CNF:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DATA_READ:
        break;

        case CYBLE_EVT_L2CAP_CBFC_RX_CREDIT_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_TX_CREDIT_IND:
        break;

        case CYBLE_EVT_L2CAP_CBFC_DATA_WRITE_IND:
        break;


        /* default catch-all case */

        default:
        break;
    }
}


/* [] END OF FILE */
