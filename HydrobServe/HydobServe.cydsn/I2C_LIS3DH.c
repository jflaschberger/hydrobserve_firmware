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

#include "I2C_LIS3DH.h"

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool acc_begin() 
{
 
  /* Check connection */
  uint8 deviceid = acc_readRegister8(LIS3DH_REG_WHOAMI);
  if (deviceid != 0x33)
  {
    /* No LIS3DH detected ... return false */
    //Serial.println(deviceid, HEX);
    return false;
  }

  // enable all axes, normal mode
  acc_writeRegister8(LIS3DH_REG_CTRL1, 0x07);
  // 400Hz rate
  //setDataRate(LIS3DH_DATARATE_400_HZ);
    //setDataRate(LIS3DH_DATARATE_100_HZ);

  // High res & BDU enabled
  acc_writeRegister8(LIS3DH_REG_CTRL4, 0x88);

  // DRDY on INT1
  acc_writeRegister8(LIS3DH_REG_CTRL3, 0x10);

  // Turn on orientation config
  //writeRegister8(LIS3DH_REG_PL_CFG, 0x40);

  // enable adcs
  //acc_writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);

  return true;
}

void acc_print_xyz()
{
    //read data
    acc_read();
    
    //and now print it!
    acc_read();
    sprintf(debugStr, "x = %f,\t y = %f\t, z = %f", x_g, y_g, z_g);
    SW_TX_PutString(debugStr);SW_TX_PutCRLF();    
}


void acc_read(void) 
{
  // read x y z at once
/*
    Wire.beginTransmission(LIS3DH_DEFAULT_ADDRESS);
    Wire.write(LIS3DH_REG_OUT_X_L | 0x80); // 0x80 for autoincrement
    Wire.endTransmission();

    Wire.requestFrom(LIS3DH_DEFAULT_ADDRESS, 6);
    x = Wire.read(); x |= ((uint16_t)Wire.read()) << 8;
    y = Wire.read(); y |= ((uint16_t)Wire.read()) << 8;
    z = Wire.read(); z |= ((uint16_t)Wire.read()) << 8;
 */
    uint8 data[6];
        
	//clear master status
    uint8 status = I2CM_I2CMasterClearStatus();
    //debugI2CMasterStatus(status);
    uint8 indicator = I2CM_I2CMasterSendStart(LIS3DH_DEFAULT_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(LIS3DH_REG_OUT_X_L | 0x80);
        //debugI2CTransfer(indicator);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (acc), write aborted"); SW_TX_PutCRLF(); 
    }
    I2CM_I2CMasterSendRestart(LIS3DH_DEFAULT_ADDRESS, I2CM_I2C_READ_XFER_MODE);

	
    uint8 i =0;
	for (i = 0; i < 6; i++)
    {
        if(i < 5)
        {
            //I2CM_I2CMasterSendRestart(MMA8452Q_ADDRESS, I2CM_I2C_READ_XFER_MODE);
            data[i] = I2CM_I2CMasterReadByte(I2CM_I2C_ACK_DATA);
        }
        else//last one with NAK
        {
            data[i] = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
        }
        //sprintf(debugStr," %i ", data[i]); SW_TX_PutString(debugStr);
    }
    
    //build values from byte array
    x = data[0] | (uint16) data[1] << 8;
    y = data[2] | (uint16) data[3] << 8;
    z = data[4] | (uint16) data[5] << 8;

    
  uint8 range = acc_getRange();
  uint16 divider = 1;
  if (range == LIS3DH_RANGE_16_G) divider = 1365; // different sensitivity at 16g
  if (range == LIS3DH_RANGE_8_G) divider = 4096;
  if (range == LIS3DH_RANGE_4_G) divider = 8190;
  if (range == LIS3DH_RANGE_2_G) divider = 16380;

  x_g = (float)x / divider;
  y_g = (float)y / divider;
  z_g = (float)z / divider;

}

/**************************************************************************/
/*!
    @brief  Read the auxilary ADC
*/
/**************************************************************************/
/*
int16_t acc_readADC(uint8_t adc) {
  if ((adc < 1) || (adc > 3)) return 0;
  uint16_t value;

  adc--;

  uint8_t reg = LIS3DH_REG_OUTADC1_L + adc*2;

    Wire.beginTransmission(LIS3DH_DEFAULT_ADDRESS);
    Wire.write(reg | 0x80);   // 0x80 for autoincrement
    Wire.endTransmission();
    Wire.requestFrom(LIS3DH_DEFAULT_ADDRESS, 2);
    value = Wire.read();  value |= ((uint16_t)Wire.read()) << 8;
  

  return value;
}*/


/**************************************************************************/
/*!
    @brief  Set INT to output for single or double click
*/
/**************************************************************************/

void acc_setClick(uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow) {
  if (!c) {
    //disable int
    uint8_t r = acc_readRegister8(LIS3DH_REG_CTRL3);
    r &= ~(0x80); // turn off I1_CLICK
    acc_writeRegister8(LIS3DH_REG_CTRL3, r);
    acc_writeRegister8(LIS3DH_REG_CLICKCFG, 0);
    return;
  }
  // else...

  acc_writeRegister8(LIS3DH_REG_CTRL3, 0x80); // turn on int1 click
  acc_writeRegister8(LIS3DH_REG_CTRL5, 0x08); // latch interrupt on int1

  if (c == 1)
    acc_writeRegister8(LIS3DH_REG_CLICKCFG, 0x15); // turn on all axes & singletap
  if (c == 2)
    acc_writeRegister8(LIS3DH_REG_CLICKCFG, 0x2A); // turn on all axes & doubletap


  acc_writeRegister8(LIS3DH_REG_CLICKTHS, clickthresh); // arbitrary
  acc_writeRegister8(LIS3DH_REG_TIMELIMIT, timelimit); // arbitrary
  acc_writeRegister8(LIS3DH_REG_TIMELATENCY, timelatency); // arbitrary
  acc_writeRegister8(LIS3DH_REG_TIMEWINDOW, timewindow); // arbitrary
}

uint8_t acc_getClick(void) {
  return acc_readRegister8(LIS3DH_REG_CLICKSRC);
}


/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void acc_setRange(lis3dh_range_t range)
{
  uint8_t r = acc_readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  acc_writeRegister8(LIS3DH_REG_CTRL4, r);
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
lis3dh_range_t acc_getRange(void)
{
  /* Read the data format register to preserve bits */
  return (lis3dh_range_t)((acc_readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
void acc_setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = acc_readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  acc_writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the LIS3DH (controls power consumption)
*/
/**************************************************************************/
lis3dh_dataRate_t acc_getDataRate(void)
{
  return (lis3dh_dataRate_t)((acc_readRegister8(LIS3DH_REG_CTRL1) >> 4)& 0x0F);
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
/*
bool acc_getEvent(sensors_event_t *event) {
   Clear the event 
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = 0;

  read();

  event->acceleration.x = x_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y_g * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z_g * SENSORS_GRAVITY_STANDARD;
}*/

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
/*
void acc_getSensor(sensor_t *sensor) {
   Clear the sensor_t object 
  memset(sensor, 0, sizeof(sensor_t));

   //Insert the sensor name in the fixed length char array 
  strncpy (sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = 0;
  sensor->min_value   = 0;
  sensor->resolution  = 0;
}*/


/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void acc_writeRegister8(uint8_t reg, uint8_t value) 
{
    /*
    Wire.beginTransmission((uint8_t)LIS3DH_DEFAULT_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
    */
    
    I2CM_I2CMasterClearStatus();
    I2CM_I2CMasterSendStart(LIS3DH_DEFAULT_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    I2CM_I2CMasterWriteByte(reg);
    I2CM_I2CMasterWriteByte(value);
    I2CM_I2CMasterSendStop();  
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t acc_readRegister8(uint8_t reg) 
{
/*
    Wire.beginTransmission(LIS3DH_DEFAULT_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();

    Wire.requestFrom(LIS3DH_DEFAULT_ADDRESS, 1);
    value = Wire.read();
    */
    
    //clear master status
    uint8 status = I2CM_I2CMasterClearStatus();
    //debugI2CMasterStatus(status);
    
    uint8 indicator = I2CM_I2CMasterSendStart(LIS3DH_DEFAULT_ADDRESS, I2CM_I2C_WRITE_XFER_MODE);
    //debugI2CTransfer(indicator);
    
    //do transfer if ready
    if(indicator == I2CM_I2C_MSTR_NO_ERROR)
    {
        I2CM_I2CMasterWriteByte(reg);
        //debugI2CTransfer(indicator);
    }
    else
    {
        SW_TX_PutString("Could not beginn I2C transmittion (acc), write aborted"); SW_TX_PutCRLF(); 
    }
    
    I2CM_I2CMasterSendRestart(LIS3DH_DEFAULT_ADDRESS, I2CM_I2C_READ_XFER_MODE);
    uint8 data = I2CM_I2CMasterReadByte(I2CM_I2C_NAK_DATA);
    I2CM_I2CMasterSendStop();
    return data;
}



/* [] END OF FILE */
