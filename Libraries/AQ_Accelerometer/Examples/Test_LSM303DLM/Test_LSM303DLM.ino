/*
  AeroQuad v3.0 - March 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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



#include <Wire.h>             // Arduino IDE bug, needed because that the ITG3200 use Wire!
#include <Device_I2C.h>       // Arduino IDE bug, needed because that the ITG3200 use Wire!
//#include <Platform_CHR6DM.h>  // Arduino IDE bug, needed because that the CHR6DM use Wire!

#include <GlobalDefined.h>
#include <APM_ADC.h>
#include <AQMath.h>
#include <Accelerometer_LSM303DLM.h>

unsigned long timer;

void setup() {
  
  Wire.begin();
  
  #ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

 #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif  
  
  Serial.begin(115200);
  Serial.println("Accelerometer library test (LSM303DLM)");
  
  initializeAccel();
  computeAccelBias();
}

void loop() {
  
  if((millis() - timer) > 100) // 10Hz
  {
    timer = millis();
    //accel.measure();
    measureAccel();
    
    Serial.print("Roll: ");
    Serial.print(meterPerSecSec[XAXIS]);
    Serial.print(" Pitch: ");
    Serial.print(meterPerSecSec[YAXIS]);
    Serial.print(" Yaw: ");
    Serial.print(meterPerSecSec[ZAXIS]);
    Serial.println();
  }
}
