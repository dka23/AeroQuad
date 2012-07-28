/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_ACCELEROMETER_LSM303DLM_H_
#define _AEROQUAD_ACCELEROMETER_LSM303DLM_H_

#include <Accelerometer.h>
#include <SensorsStatus.h>

#define ACCEL_ADDRESS (0x30 >> 1) // for SA0 = Low

// Registers
#define LSM303_CTRL_REG1_A       0x20

void initializeAccel() {
  updateRegisterI2C(ACCEL_ADDRESS, LSM303_CTRL_REG1_A, 0x27); // Normal power mode, all axes enabled

    sendByteI2C(ACCEL_ADDRESS, LSM303_CTRL_REG1_A);
    if (readByteI2C(ACCEL_ADDRESS) ==  0x27) {
        vehicleState |= ACCEL_DETECTED;
    }

    
   delay(10); 
}
  
void measureAccel() {
    accelScaleFactor[XAXIS] = 19.62 / 32767.0;
    accelScaleFactor[YAXIS] = -19.62 / 32767.0;
    accelScaleFactor[ZAXIS] = -19.62 / 32767.0;
    
  sendByteI2C(ACCEL_ADDRESS, 0xA8);
  Wire.requestFrom(ACCEL_ADDRESS, 6);

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = readReverseShortI2C() * accelScaleFactor[axis] + runTimeAccelBias[axis];
  }
}

void measureAccelSum() {

  sendByteI2C(ACCEL_ADDRESS, 0xA8);
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    accelSample[axis] += readReverseShortI2C();
  }
  accelSampleCount++;
}

void evaluateMetersPerSec() {
    accelScaleFactor[XAXIS] = 19.62 / 32767.0;
    accelScaleFactor[YAXIS] = -19.62 / 32767.0;
    accelScaleFactor[ZAXIS] = -19.62 / 32767.0;

    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
    accelScaleFactor[XAXIS] = 19.62 / 32767.0;
    accelScaleFactor[YAXIS] = -19.62 / 32767.0;
    accelScaleFactor[ZAXIS] = -19.62 / 32767.0;

    
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}



#endif
