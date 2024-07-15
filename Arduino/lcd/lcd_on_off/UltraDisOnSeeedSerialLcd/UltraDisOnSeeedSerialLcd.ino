/*
 * UltraDisOnSeeedSerialLcd.ino
 * Example sketch for ultrasonic ranger
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : LG, FrankieChu
 * Create Time: Jan 17,2013
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/***************************************************************************/
//  Function: Measure the distance to obstacles in front and display the
//			  result on seeedstudio serialLcd. Make sure you installed the
//			  serialLCD, SofewareSerial and Ultrasonic library.
//	Hardware: Grove - Ultrasonic Ranger, Grove - Serial LCD
//	Arduino IDE: Arduino-1.0
/*****************************************************************************/


#include <SoftwareSerial.h>
#include "Ultrasonic.h"
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;
Ultrasonic ultrasonic(3);


int people = false;
int lsc_on = false;
unsigned long int passtime = 0;
unsigned long int deltatime = 60000;

void setup()
{
    Serial.begin(9600);

    lcd.begin(16, 2);
    
    // Print a message to the LCD.
    lcd.print("hello, world!");
}
void loop()
{
	long RangeInCentimeters;
	RangeInCentimeters = ultrasonic.MeasureInCentimeters();
	delay(150);

    if(RangeInCentimeters<100)
    {
        if(!people)
        {
            passtime=millis();
            lcd.setCursor(0, 1);
            Serial.println(RangeInCentimeters);
            Serial.println("People In");
            lcd.display();
            people = true;
            lcd.print("Bonjour ");
            lcd.print(RangeInCentimeters);
            
        }
        passtime=millis();
    }
    else
    {
        people=false;
        if(millis()-passtime<deltatime)
        {
            lcd.setCursor(14,0);
            lcd.print((deltatime-(millis()-passtime))/1000);
            //Serial.print((deltatime-(millis()-passtime))/1000);
        }
        else
        {

            lcd.noDisplay();
        }
    }
}

