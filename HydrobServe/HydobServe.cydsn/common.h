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
#include <stdint.h>
#include <stdbool.h>
#include <project.h>
#include <stdio.h>



char debugStr[50];


/**********************************************************
                global variables
 **********************************************************/
//global variables holding the times for MiCS cycles. Defined in mainFunctions.c
//extern uint32 new_hall_pulses;
//extern uint16 ctrIdleTime;
extern CY_NOINIT uint16 overall_volume;
extern CY_NOINIT uint16 overall_pulses;
extern CY_NOINIT uint16 num_drink_events;

extern CY_NOINIT float32 hall_multiplier;
extern CY_NOINIT uint8 oldBatLevel;
extern CY_NOINIT uint8 gender;

//these hold the name strings. content is char, but for BLE subsystem everything needs to be bytes anyways
extern CY_NOINIT  uint8 firstName[22];
extern CY_NOINIT  uint8 lastName[22];
extern CY_NOINIT  uint8 userID[22];


//enum systemstate {drinking, idle, inBLEconnection, moved};


/**********************************************************
                global constants
 **********************************************************/

#define ON                          (1u)
#define OFF                         (0u)



#define BATTERY_TIMEOUT             (3000u)       /* Counts depend on connection parameters */    
#define MEASURE_BATTERY_MAX         (3000)      /* Use 3V as battery voltage starting */
#define MEASURE_BATTERY_MID         (2800)      /* Use 2.8V as a knee point of discharge curve @ 29% */
#define MEASURE_BATTERY_MID_PERCENT (29)        
#define MEASURE_BATTERY_MIN         (2500)      /* Use 2.5V as a cut-off of battery life */
#define LOW_BATTERY_LIMIT           (10)        /* Low level limit in percent */
    
#define ADC_VREF_MASK               (0x000000F0Lu)

#define IDLE_TIME_TILL_SLEEP        (30u)       // time after which in idle system goes to sleep

//EEPROM 
#define EEPROM_ADDRESS              (0x50u)      //Adress of the EEPROM
#define EEPROM_MULTIPLIER           (00u)     //Register Adress of pulse multiplier (4 bytes)
#define EEPROM_FIRSTNAME            (04u)     //Register Adress of first name (22 byte long)
#define EEPROM_LASTNAME             (26u)     //Register Adress of first name (22 byte long)
#define EEPROM_ID                   (50u)     //Register Adress of first name (22 byte long)
#define EEPROM_evts                 (75u)
#define EEPROM_GENDER               (80u)

/* [] END OF FILE */
