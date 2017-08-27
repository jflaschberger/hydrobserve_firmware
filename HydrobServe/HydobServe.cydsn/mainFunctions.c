/* ========================================
 * Author:  Jannai Flaschberger, 2017
 * Copyright (C)

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ========================================
*/

#include "mainFunctions.h"
#include "I2C_EEPROM.h"
#include "BLE_functions.h"
#include "common.h"
#include "I2C_LIS3DH.h"



/*************************************************************
    *       Variables
    ************************************************************/
//global vars to be defined
uint16  ctrIdleTime = 0;
uint32  new_hall_pulses = 0;
uint16  num_drink_events = 0;
uint8   oldBatLevel = 0;
float32 hall_multiplier;
uint8   gender;
uint16  overall_volume;
uint16  overall_pulses;

//global strings (uint8 because BLE subsystem likes it that way)
uint8 userID[22];
uint8 firstName[22];
uint8 lastName[22];


//local vars
bool    uncountedPulses = false;
uint16  counted_hall_pulses = 0;
float32 hall_multiplier;


enum sysState {drinking, idle, energySaving, sleeping};
enum BLEState {powerDown, advertising, connected, goingToSleep, disconnected};

//enum systemstate SysState = idle;
enum sysState SystemState = idle;
enum BLEState bleState = disconnected;


/*****************************************************************
*secondly routine, triggered by timer isr
*****************************************************************/
CY_ISR(DoThisEverySecond)
{
    Timer_1_STATUS;     //reset counter
     
    // !!! BLE advertising is completely done by button press / wakeup routine!
    //don't activate handle_BLE_Statemachine();
    
    //SW_TX_PutString("tick");SW_TX_PutCRLF();
    //sprintf(debugStr, "Battery: %i" ,MeasureBattery());
    //SW_TX_PutString(debugStr);SW_TX_PutCRLF();
    
    //how many seconds are we going to stay awake from now on?
    sprintf(debugStr, "%i" ,IDLE_TIME_TILL_SLEEP - ctrIdleTime);
    SW_TX_PutString(debugStr);SW_TX_PutCRLF();
        
    
    //check if counter got some pulses
    if(new_hall_pulses)
    {
        counted_hall_pulses = counted_hall_pulses + new_hall_pulses;
        new_hall_pulses = 0;
        
        //set system state to dringking and reset timer
        if(SystemState != drinking)
        {
            SystemState = drinking;
            SW_TX_PutString("------------------------   SystemState = drinking------------------------");SW_TX_PutCRLF();
        }
        ctrIdleTime = 0;
    }
    
    
    
    //3 seconds without hall pulses. Drinking event is finished!
    if(ctrIdleTime > 2 && SystemState == drinking)
    {
        SystemState = idle;
        SW_TX_PutString("------------------------  SystemState = idle------------------------");SW_TX_PutCRLF();
        
        //calculate drunk volume
        uint16 addedVolume = (uint16) ((float) counted_hall_pulses * hall_multiplier);
        overall_volume = overall_volume + addedVolume;
        overall_pulses = overall_pulses + counted_hall_pulses;
        
        //add to drink events
        num_drink_events++;        
        
        //write to ble
        writeUint16ToBLEdb(overall_volume, CYBLE_VOLUME_DATA_DRUNK_VOLUME_CHAR_HANDLE);
        writeUint16ToBLEdb(overall_pulses, CYBLE_VOLUME_DATA_PULSE_COUNT_CHAR_HANDLE);
        writeUint16ToBLEdb(num_drink_events, CYBLE_VOLUME_DATA_NUMBER_DRINK_EVTS_CHAR_HANDLE);
        
        sprintf(debugStr, "added Pulses | Volume: %i | %i ml", counted_hall_pulses, addedVolume);
        SW_TX_PutString(debugStr);SW_TX_PutCRLF();
        sprintf(debugStr, "overall Pulses | Volume: %i | %i ml", overall_pulses, overall_volume);
        SW_TX_PutString(debugStr);SW_TX_PutCRLF();
        
        //reset counting variables
        addedVolume = 0;
        counted_hall_pulses = 0;
        
    }
       
    if(CyBle_GetState() == CYBLE_STATE_CONNECTED)
    {
        ctrIdleTime = 0;
    }
    
    if(ctrIdleTime == IDLE_TIME_TILL_SLEEP)
    {
       GoToHibernate();
    }    
    
    //increase timer
    ctrIdleTime++;
    
    acc_print_xyz();
}


/*****************************************************************
*bool BLE_powerDown();
*****************************************************************/
bool BLE_powerDown();


/*****************************************************************
*bool BLE_powerUp();
*****************************************************************/
bool BLE_powerUp();


/*****************************************************************
*bool system_PowerUp();
*****************************************************************/
bool system_PowerUp();


/*****************************************************************
*bool system_PowerDown();
*****************************************************************/
bool system_PowerDown();


/*****************************************************************
*system_PowerSave();
*****************************************************************/
bool system_PowerSave();


/*****************************************************************
*ISR firing by every pulse from hall sensor
*****************************************************************/
CY_ISR(ISR_hall_pulse)
{
    ISR_hall_ClearPending(); 
    uint8 InterruptState = Pin_hall_sig_ClearInterrupt();
    //count pulse
    new_hall_pulses++;
    //sprintf(debugStr," %lu ",new_hall_pulses);
    //SW_TX_PutString(debugStr);
    
    uncountedPulses = true;
}


/*****************************************************************
*button got pressed. Initialize system if we have been sleeping before
*****************************************************************/
CY_ISR(ISR_button_press)
{
    ISR_btn_ClearPending();
    ctrIdleTime = 0;
    
    uint8 InterruptState = Pin_button_isr_ClearInterrupt();
    
    //yey, we woke back up. Restart BLE component, initialize, ...
    SystemState = idle;
    SW_TX_PutString("button pressed ISR fired");SW_TX_PutCRLF();
    
    switch(CyBle_GetState())
    {
        case CYBLE_STATE_CONNECTED:
            {SW_TX_PutString("advertising not possible: already connected!"); SW_TX_PutCRLF();
            break;}
        case CYBLE_STATE_ADVERTISING:
            {SW_TX_PutString("advertising not possible: already advertising right now"); SW_TX_PutCRLF();
            break;}
        case CYBLE_STATE_DISCONNECTED:
            {SW_TX_PutString("begin to advertise now"); SW_TX_PutCRLF();
            CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);
            break;}
        case CYBLE_STATE_INITIALIZING:
            {SW_TX_PutString("advertising not possible: still in init phase"); SW_TX_PutCRLF();
            break;}
        case CYBLE_STATE_STOPPED:
            {SW_TX_PutString("advertising not possible: BLE device stopped"); SW_TX_PutCRLF();
            break;}
        default:
            {SW_TX_PutString("unknown BLE State (advertising switch default)"); SW_TX_PutCRLF();}
    }
    
}


/*****************************************************************
*The accellerometer eportet an movement. Do check for pulses and presence of water
*****************************************************************/
CY_ISR(ISR_acc_movement)
{
    //SystemState = moved;
    SW_TX_PutString("movement ISR fired");SW_TX_PutCRLF();
}




/*****************************************************************
*initialize the accellerometer
*****************************************************************/
bool initAcc()
{
    bool success = acc_begin();
    if(success)
    {
        SW_TX_PutString("accelerometer successfully initialized");SW_TX_PutCRLF();
    }
    else
    {
        SW_TX_PutString("accelerometer COULD NOT be initialized");SW_TX_PutCRLF();
    }
    return true;
}



static void LowPowerImplementation(void)
{
    CYBLE_LP_MODE_T bleMode;
    uint8 interruptStatus;
    
    /* For advertising and connected states, implement deep sleep 
     * functionality to achieve low power in the system. For more details
     * on the low power implementation, refer to the Low Power Application 
     * Note.
     */
    if((CyBle_GetState() == CYBLE_STATE_ADVERTISING) || 
       (CyBle_GetState() == CYBLE_STATE_CONNECTED))
    {
        SW_TX_PutString("tyring to shut down BLE");SW_TX_PutCRLF();
        /* Request BLE subsystem to enter into Deep-Sleep mode between connection and advertising intervals */
        bleMode = CyBle_EnterLPM(CYBLE_BLESS_DEEPSLEEP);
        /* Disable global interrupts */
        interruptStatus = CyEnterCriticalSection();
        /* When BLE subsystem has been put into Deep-Sleep mode */
        if(bleMode == CYBLE_BLESS_DEEPSLEEP)
        {
            /* And it is still there or ECO is on */
            if((CyBle_GetBleSsState() == CYBLE_BLESS_STATE_ECO_ON) || 
               (CyBle_GetBleSsState() == CYBLE_BLESS_STATE_DEEPSLEEP))
            {
                CySysPmDeepSleep();
            }
        }
        else /* When BLE subsystem has been put into Sleep mode or is active */
        {
            /* And hardware doesn't finish Tx/Rx opeation - put the CPU into Sleep mode */
            if(CyBle_GetBleSsState() != CYBLE_BLESS_STATE_EVENT_CLOSE)
            {
                CySysPmSleep();
            }
        }
        /* Enable global interrupt */
        CyExitCriticalSection(interruptStatus);
    }
}



void GoToHibernate()
{   
    //Global signal starten!
    //Alle anderen Komponenten noch ausschalten!
    //!!!!!!!!!!!Hall Sensor aussschalte
        
    CyDelay(35);
    //switch off hall sensor
    VCC_hall_Write(OFF);
    SW_TX_PutString("Gehe Schlafen\n\r");
    I2CM_Stop();
    CyDelay(35);
    
    CySysPmHibernate(); 
}



uint8 MeasureBattery(void)
{
	int16 adcResult;
    int32 mvolts;
	uint32 sarControlReg;
    uint8 batteryLevel;

    
    /* Set the reference to VBG and enable reference bypass */
	sarControlReg = ADC_SAR_CTRL_REG & ~ADC_VREF_MASK;
	ADC_SAR_CTRL_REG = sarControlReg | ADC_VREF_INTERNAL1024BYPASSED;
	
	/* 25 ms delay for reference capacitor to charge */
	CyDelay(25);             
	
	/* Set the reference to VDD and disable reference bypass */
	sarControlReg = ADC_SAR_CTRL_REG & ~ADC_VREF_MASK;
	ADC_SAR_CTRL_REG = sarControlReg | ADC_VREF_VDDA;

	/* Perform a measurement. Store this value in Vref. */
	CyDelay(1);
	ADC_StartConvert();
	ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);

    adcResult = ADC_GetResult16(0);
	/* Calculate input voltage by using ratio of ADC counts from reference
	*  and ADC Full Scale counts. 
    */
	mvolts = (1024 * 2048) / adcResult;
    
    /* Convert battery level voltage to percentage using linear approximation
    *  divided into two sections according to typical performance of 
    *  CR2033 battery specification:
    *  3V - 100%
    *  2.8V - 29%
    *  2.5V - 0%
    */
    if(mvolts < MEASURE_BATTERY_MIN)
    {
        batteryLevel = 0;
    }
    else if(mvolts < MEASURE_BATTERY_MID)
    {
        batteryLevel = (mvolts - MEASURE_BATTERY_MIN) * MEASURE_BATTERY_MID_PERCENT / 
                       (MEASURE_BATTERY_MID - MEASURE_BATTERY_MIN); 
    }
    else if(mvolts < MEASURE_BATTERY_MAX)
    {
        batteryLevel = MEASURE_BATTERY_MID_PERCENT +
                       (mvolts - MEASURE_BATTERY_MID) * (100 - MEASURE_BATTERY_MID_PERCENT) / 
                       (MEASURE_BATTERY_MAX - MEASURE_BATTERY_MID); 
    }
    else
    {
        batteryLevel = 100;
    }
    
   return batteryLevel; 
}


/*****************************************************************
*void initBLEcharacteristics();
loads current state of variables (not resetted after hybernate) into
BLE characteristics (resetted after hybernate)
*****************************************************************/
void initBLEcharacteristics()
{    
    //set up pointer for char arrays
    uint8* ptr;
    uint8 i;
    
    //now read content from EEPROM
    hall_multiplier = readFloat32FromEEPROM(EEPROM_MULTIPLIER);
    gender = readOneByteFromEEPROM(EEPROM_GENDER);
    
    //read out all the strings
    for(i=0; i<22;i++)
    {
        firstName[i] = readOneByteFromEEPROM(EEPROM_FIRSTNAME + i);
        lastName[i] = readOneByteFromEEPROM(EEPROM_LASTNAME + i);
        userID[i] = readOneByteFromEEPROM(EEPROM_ID + i);
    }
    
    /*
    //print our what we got
    SW_TX_PutString("----------read the following from EEPROM"); SW_TX_PutCRLF();
    
    //hall multiplier
    sprintf(debugStr, "hall_multiplier (float) =  %f", hall_multiplier);
    SW_TX_PutString(debugStr); 
    
    SW_TX_PutCRLF();
    
    //gender
    sprintf(debugStr, "gender =  %i", gender);
    SW_TX_PutString(debugStr); SW_TX_PutCRLF();
    
    
    //First Name
    SW_TX_PutString("First Name String: ");
    for(i=0;i<22;i++)
    {
        sprintf(debugStr, " %c=%i ", firstName[i], firstName[i]);
        SW_TX_PutString(debugStr);
    }
    SW_TX_PutCRLF();
    
    //Last Name
    SW_TX_PutString("Last Name String: ");
    for(i=0;i<22;i++)
    {
        sprintf(debugStr, " %c=%i ", lastName[i], lastName[i]);
        SW_TX_PutString(debugStr);
    }
    SW_TX_PutCRLF();
    
    //Uder ID
    SW_TX_PutString("User ID String: ");
    for(i=0;i<22;i++)
    {
        sprintf(debugStr, " %c=%i ", userID[i], userID[i]);
        SW_TX_PutString(debugStr);
    }
    SW_TX_PutCRLF();
    */
    
    
    
    //pulse multiplier
    writeFloat32ToBLEdb(hall_multiplier, CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_CHAR_HANDLE);
    //drunk volume
    writeUint16ToBLEdb(overall_volume, CYBLE_VOLUME_DATA_DRUNK_VOLUME_CHAR_HANDLE);
    //number of drink events
    writeUint16ToBLEdb(num_drink_events, CYBLE_VOLUME_DATA_NUMBER_DRINK_EVTS_CHAR_HANDLE);
    //pulse count
    writeUint16ToBLEdb(overall_pulses, CYBLE_VOLUME_DATA_PULSE_COUNT_CHAR_HANDLE);
    //gender
    writeUint8ToBLEdb(gender, CYBLE_USER_DATA_GENER_CHAR_HANDLE);
    /*
    //first name
    ptr = &firstName[0];
    write22CharsToBLEdb(ptr, CYBLE_USER_DATA_FIRST_NAME_CHAR_HANDLE);
    
    //last name
    ptr = &lastName[0];
    write22CharsToBLEdb(ptr, CYBLE_USER_DATA_LAST_NAME_CHAR_HANDLE);
    
    //id
    ptr = &userID[0];
    write22CharsToBLEdb(ptr, CYBLE_USER_DATA_USER_ID_CHAR_HANDLE);
    
    
    */
    
    CYBLE_GATT_HANDLE_VALUE_PAIR_T dataHandle;
    dataHandle.attrHandle = CYBLE_USER_DATA_FIRST_NAME_CHAR_HANDLE;
    dataHandle.value.val = firstName;
    dataHandle.value.len = 22;
    CYBLE_GATT_ERR_CODE_T error = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);   
    
    dataHandle.attrHandle = CYBLE_USER_DATA_LAST_NAME_CHAR_HANDLE;
    dataHandle.value.val = lastName;
    dataHandle.value.len = 22;
    CYBLE_GATT_ERR_CODE_T error2 = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);   
    
    dataHandle.attrHandle = CYBLE_USER_DATA_USER_ID_CHAR_HANDLE;
    dataHandle.value.val = userID;
    dataHandle.value.len = 22;
    CYBLE_GATT_ERR_CODE_T error3 = CyBle_GattsWriteAttributeValue(&dataHandle, 0, &cyBle_connHandle, CYBLE_GATT_DB_LOCALLY_INITIATED);   
}


/* [] END OF FILE */
