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
#include <project.h>
#include "BLE_functions.h"
#include "common.h"
#include "mainFunctions.h"

#define WDT_INTERVAL_2S             2000u                       /* millisecond */
#define ILO_FREQ                    32000                       /* Hz */


/***************************************
*        function Dummies
***************************************/
void handle_BLE_Statemachine();
         

//ISR dummies
CY_ISR(DoThisEverySecond);
CY_ISR(ISR_hall_pulse);
CY_ISR(ISR_button_press);
CY_ISR(ISR_acc_movement);
CY_ISR(ISR_anyInterrupt);
CY_ISR(isrCombined);


#define Pin_button_isr_IntrTrigger      ((uint32)((uint32)1u << Pin_button_isr__PORT))
#define Pin_NoDPWakeup_IntrTrigger    ((uint32)((uint32)1u << Pin_NoDPWakeup__PORT))
#define Pin_CombinedIntr_IntrTrigger  ((uint32)((uint32)1u << Pin_CombinedIntr__PORT))
//delete later


CY_ISR(isrCombined)
{
     if(Pin_button_isr_INTR_CAUSE & Pin_button_isr_IntrTrigger)
    {
    }
    
        Pin_CombinedIntr_ClearInterrupt();
}


int main()
{
    //start SW UART TX for debug
    SW_TX_Start();
    SW_TX_PutCRLF();
    SW_TX_PutCRLF();
    SW_TX_PutString("----------------UART debug started------------------------");   
    
    //Start Analog Digital Converter
    ADC_Start();
    
    // DO NOT CHANGE 
    // Change SCL and SDA pins drive mode to Resistive Pull Up --> no external components necessary
    I2CM_scl_SetDriveMode(I2CM_scl_DM_RES_UP);
    I2CM_sda_SetDriveMode(I2CM_sda_DM_RES_UP);
     
    
    /*************************************************************
    *       Start Interrupts
    ************************************************************/
    
    //Start secondly timer running the main routine
    Timer_1_Start();
    
    // Enable the Interrupt component connected to Timer interrupt 
    ISR_everySecond_StartEx(DoThisEverySecond);
    // Enable the Interrupt component connected to hall sensor
    ISR_hall_StartEx(ISR_hall_pulse);
    //ISR_hall_SetVector(ISR_hall_pulse);
    // Enable the Interrupt component connected to accellerometer
    ISR_acc_StartEx(ISR_acc_movement);
    // Enable the Interrupt component connected to pushbutton
    ISR_btn_StartEx(ISR_button_press);
    //ISR_btn_SetVector(ISR_button_press);
    
    //ISR_wakeup_StartEx(ISR_anyInterrupt);
    isr_CombinedPort_StartEx(isrCombined);
    
    /* zaehler_pos is not resettet. Do manual reset if Reset Reason is not Hibernate*/
    /* Values are safed if device wakes up from hibernate but not if it resetts of another reason*/
    if( CySysPmGetResetReason() != CY_PM_RESET_REASON_WAKEUP_HIB )
      {
        overall_volume = 0;
        overall_pulses = 0;
      }
    
    uint32 reason = CySysPmGetResetReason();
    SW_TX_PutString("Wakeup Reason = ");  
    switch(reason)
        {
        /* Power up, XRES */
        case CY_PM_PWR_STOP_TOKEN_XRES:
            SW_TX_PutString("Reset");  
            break;

        /* Wakeup from Hibernate */
        case CY_PM_PWR_STOP_TOKEN_HIB:
            SW_TX_PutString("wakeup from hibernate");  
            break;

        /* Wakeup from Stop (through WAKEUP pin assert) */
        case CY_PM_PWR_STOP_TOKEN_STOP:
            SW_TX_PutString("wakeup from stop");  
            break;

        /* Unknown reason */
        default:
            SW_TX_PutString("Unknown reason");  
            break;
        }
        SW_TX_PutCRLF();
    
    //start I2C
    //I2CM_Start();
    
       
    
    //start BLE component and wait for it to initialize
    CyBle_Start(StackEventHandler);
    while (CyBle_GetState() == CYBLE_STATE_INITIALIZING)
    {
        CyBle_ProcessEvents();
        CyDelay(10);
    }
    
    // Enable global interrupts 
    CyGlobalIntEnable;
    
    
    //switch on hall sensor
    VCC_hall_Write(ON);
    
    //check if bat level has changed and if so, pack new value into ble database
    uint8 newBatLevel = MeasureBattery();
    if(oldBatLevel != newBatLevel)
    {
        oldBatLevel = newBatLevel;
        writeUint8ToBLEdb(oldBatLevel, CYBLE_BATTERY_BATTERY_LEVEL_CHAR_HANDLE);
    }
    
    //start I2C communication with eeprom and let it wake up
    I2CM_Start(); CyDelay(5);
    //now init BLE characteristics with saved variables
    initBLEcharacteristics();
    initAcc();
    //I2CM_Stop();
    
    SW_TX_PutString("---------------------INIT DONE. NOW LOOPING-------------------");SW_TX_PutCRLF();
    

    /*************************************************************
    *       LOOP ROUTINE
    ************************************************************/
    for(;;)
    {
        //regularly update BLE Stack
        CyBle_ProcessEvents();    
        //SW_TX_PutString(".");
        //CyDelay(10);
        
            
    }
}//end main





/* ========================================
 *
 * To Do:
 * don'tgo to sleep when a BLE connectio is established. Stay awake till a disconnect event
 * 24 Mhz CPU frequency is an overkill for this use case. Probably lower it and save power?
 * Think of power saving modes while in awake mode
 * ========================================
*/



