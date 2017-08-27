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
#include "common.h"





bool initAcc();

static void LowPowerImplementation(void);

/*****************************************************************
*bool BLE_powerDown();
*****************************************************************/
bool BLE_powerDown();

/*****************************************************************
*void initBLEcharacteristics();
*****************************************************************/
void initBLEcharacteristics();

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
*Go to hibernate node();
*****************************************************************/
void GoToHibernate();

/*****************************************************************
*MeasureBattery;
*****************************************************************/
uint8 MeasureBattery(void);

void testEEPROM();

/* [] END OF FILE */
