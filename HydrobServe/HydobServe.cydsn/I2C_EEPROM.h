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

#if !defined(I2C_EEPROM_H)
#define I2C_EEPROM_H

#include "common.h"

#define EEPROM_ADDRESS (0x50u)
    
   
void writeOneByteToEEPROM(uint8 regAdr, uint8 value);
void writeUint16ToEEPROM(uint8 regAdr, uint16 value);
void writeFloat32ToEEPROM(uint8 regAdr, float32 value);

uint8 readOneByteFromEEPROM(uint8 regAdr);

uint16 readUint16FromEEPROM(uint8 regAdr);

float32 readFloat32FromEEPROM(uint8 regAdr);


void debugI2CTransfer(uint32 inidicator);
void debugI2CMasterStatus(uint32 status);

#endif
/* [] END OF FILE */
