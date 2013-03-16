//
//  LED_Stripes.h
//  
//
//  Created by Daniel Kleine-Albers on 05.01.13.
//
//

#ifndef _LED_Stripes_h
#define _LED_Stripes_h

#include <Arduino.h>
#include "pins_arduino.h"

#define STRIPE_B_W 8
#define STRIPE_B_R 9
#define STRIPE_F_W 10
#define STRIPE_FR_G 11
#define STRIPE_FL_R 12

#define BASE_BRIGHTNESS_WHITE 50
#define BASE_BRIGHTNESS_RED 50
#define BASE_BRIGHTNESS_GREEN 50

#define START_MS        700
#define WHITE_MS        60
#define RED_MS          180
#define PAUSE_MS        170
#define LONG_PAUSE_MS   260

unsigned long ledStripesStateChangeTime = 0;
unsigned int ledStripesState = 0;


void staticLights() {
    analogWrite(STRIPE_F_W, 0);
    analogWrite(STRIPE_FL_R, BASE_BRIGHTNESS_RED);
    analogWrite(STRIPE_FR_G, BASE_BRIGHTNESS_GREEN);
    analogWrite(STRIPE_B_W, BASE_BRIGHTNESS_WHITE);
    analogWrite(STRIPE_B_R, BASE_BRIGHTNESS_WHITE);
}

void whiteFlash() {
    analogWrite(STRIPE_F_W, 255);
    analogWrite(STRIPE_FL_R, 255);
    analogWrite(STRIPE_FR_G, 255);
}

void redFlash() {
    analogWrite(STRIPE_B_W, 0);
    analogWrite(STRIPE_B_R, 255);
}

void initializeLEDStripes() {
    pinMode(STRIPE_F_W, OUTPUT);
    pinMode(STRIPE_FL_R, OUTPUT);
    pinMode(STRIPE_FR_G, OUTPUT);
    pinMode(STRIPE_B_W, OUTPUT);
    pinMode(STRIPE_B_R, OUTPUT);
    
    staticLights();
}

void processLEDStripes() {
    unsigned long millisSinceStateChange = millis() - ledStripesStateChangeTime;
    
    if (motorArmed == ON) {
        if (ledStripesState == 0 && millisSinceStateChange >= START_MS) {
            whiteFlash();
            ledStripesState = 1;
            ledStripesStateChangeTime = millis();
        } else if (ledStripesState == 1 && millisSinceStateChange >= WHITE_MS) {
            staticLights();
            ledStripesState = 2;
            ledStripesStateChangeTime = millis();
        } else if (ledStripesState == 2 && millisSinceStateChange >= PAUSE_MS) {
            whiteFlash();
            ledStripesState = 3;
            ledStripesStateChangeTime = millis();
        } else if (ledStripesState == 3 && millisSinceStateChange >= WHITE_MS) {
            staticLights();
            ledStripesState = 4;
            ledStripesStateChangeTime = millis();
        } else if (ledStripesState == 4 && millisSinceStateChange >= LONG_PAUSE_MS) {
            redFlash();
            ledStripesState = 5;
            ledStripesStateChangeTime = millis();
        } else if (ledStripesState == 5 && millisSinceStateChange >= RED_MS) {
            staticLights();
            ledStripesState = 0;
            ledStripesStateChangeTime = millis();
        }
    } else {
        staticLights();
    }
}
#endif
