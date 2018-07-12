/*------------------------------------------------------------------------------
  LIDARLite Arduino Library
  LIDARLite.cpp

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2016 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "LIDARLitev3.h"

/*------------------------------------------------------------------------------
  Constructor

  Use LIDARLite::begin to initialize.
------------------------------------------------------------------------------*/
LIDARLite::LIDARLite() {}

/*------------------------------------------------------------------------------
  Begin

  Starts the sensor and I2C.

  Parameters
  ------------------------------------------------------------------------------
  configuration: Default 0. Selects one of several preset configurations.
  fasti2c: Default 100 kHz. I2C base frequency.
    If true I2C frequency is set to 400kHz.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::begin(int configuration, bool fasti2c, char lidarliteAddress)
{
    Wire.begin(); //-- Start I2C
    if(fasti2c) {
        #if ARDUINO >= 157
            Wire.setClock(400000UL);    //-- Set I2C frequency to 400kHz, for Arduino Due
        #else
            TWBR = ((F_CPU / 400000UL) - 16) / 2;   //-- Set I2C frequency to 400kHz
        #endif
    }
    configure(configuration, lidarliteAddress);     //-- Configuration settings

    if(Serial) Serial.println("* Supplying power to the unit(POWER_ENABLE_PIN=D3)");
    pinMode(this->power_enable_pin, OUTPUT);        //-- set power enable pin
    digitalWrite(this->power_enable_pin, HIGH);     //-- set it high for supplying power
    //-- if power_enable_pin is low, power to the unit stops
  
} /* LIDARLite::begin */

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::configure(int configuration, char lidarliteAddress)
{
    switch (configuration) {
    case 0: //-- Default mode, balanced performance
        write(0x02,0x80,lidarliteAddress); //-- Default
        write(0x04,0x08,lidarliteAddress); //-- Default
        write(0x1c,0x00,lidarliteAddress); //-- Default
        break;

    case 1: //-- Short range, high speed
        write(0x02,0x1d,lidarliteAddress);
        write(0x04,0x08,lidarliteAddress); //-- Default
        write(0x1c,0x00,lidarliteAddress); //-- Default
        break;

    case 2: //-- Default range, higher speed short range
        write(0x02,0x80,lidarliteAddress); //-- Default
        write(0x04,0x00,lidarliteAddress);
        write(0x1c,0x00,lidarliteAddress); //-- Default
        break;

    case 3: //-- Maximum range
        write(0x02,0xff,lidarliteAddress);
        write(0x04,0x08,lidarliteAddress); //-- Default
        write(0x1c,0x00,lidarliteAddress); //-- Default
        break;

    case 4: //-- High sensitivity detection, high erroneous measurements
        write(0x02,0x80,lidarliteAddress); //-- Default
        write(0x04,0x08,lidarliteAddress); //-- Default
        write(0x1c,0x80,lidarliteAddress);
        break;

    case 5: //-- Low sensitivity detection, low erroneous measurements
        write(0x02,0x80,lidarliteAddress); //-- Default
        write(0x04,0x08,lidarliteAddress); //-- Default
        write(0x1c,0xb0,lidarliteAddress);
        break;
    }
} /* LIDARLite::configure */

/*------------------------------------------------------------------------------
  Reset

  Reset device. The device reloads default register settings, including the
  default I2C address. Re-initialization takes approximately 22ms.

  Parameters
  ------------------------------------------------------------------------------
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::reset(char lidarliteAddress)
{
    write(0x00,0x00,lidarliteAddress);
} /* LIDARLite::reset */

/*------------------------------------------------------------------------------
  Distance

  Take a distance measurement and read the result.

  Process
  ------------------------------------------------------------------------------
  1.  Write 0x04 or 0x03 to register 0x00 to initiate an aquisition.
  2.  Read register 0x01 (this is handled in the read() command)
      - if the first bit is "1" then the sensor is busy, loop until the first
        bit is "0"
      - if the first bit is "0" then the sensor is ready
  3.  Read two bytes from register 0x8f and save
  4.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
      The result is the measured distance in centimeters.

  Parameters
  ------------------------------------------------------------------------------
  biasCorrection: Default true. Take aquisition with receiver bias
    correction. If set to false measurements will be faster. Receiver bias
    correction must be performed periodically. (e.g. 1 out of every 100
    readings).
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
int LIDARLite::getdistance(bool biasCorrection, char lidarliteAddress)
{
    if(biasCorrection) {
        //-- Take acquisition & correlation processing with receiver bias correction
        write(0x00,0x04,lidarliteAddress);
    } else {
        //-- Take acquisition & correlation processing without receiver bias correction
        write(0x00,0x03,lidarliteAddress);
    }
    //-- Array to store high and low bytes of distance
    byte distanceArray[2];
    //-- Read two bytes from register 0x8f (autoincrement for reading 0x0f and 0x10)
    read(0x8f,2,distanceArray,true,lidarliteAddress);
    //-- Shift high byte and add to low byte
    int distance = (distanceArray[0] << 8) + distanceArray[1];
    return(distance);
} /* LIDARLite::distance */

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to write to.
  myValue: value to write.
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::write(char myAddress, char myValue, char lidarliteAddress)
{
    Wire.beginTransmission((int)lidarliteAddress);
    Wire.write((int)myAddress);     //-- Set register for write
    Wire.write((int)myValue);       //-- Write myValue to register

    //-- A nack means the device is not responding, report the error over serial
    int nackCatcher = Wire.endTransmission();
    if(nackCatcher != 0) {
        Serial.println("> nack");
    }

    delay(1); //-- 1 ms delay for robustness with successive reads and writes
} /* LIDARLite::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device. Will detect an unresponsive device and report
  the error over serial. The optional busy flag monitoring
  can be used to read registers that are updated at the end of a distance
  measurement to obtain the new data.

  Parameters
  ------------------------------------------------------------------------------
  myAddress: register address to read from.
  numOfBytes: numbers of bytes to read. Can be 1 or 2.
  arrayToSave: an array to store the read values.
  monitorBusyFlag: if true, the routine will repeatedly read the status
    register until the busy flag (LSB) is 0.
------------------------------------------------------------------------------*/
void LIDARLite::read(char myAddress, int numOfBytes, byte arrayToSave[2], bool monitorBusyFlag, char lidarliteAddress)
{
    int busyFlag = 0; //-- busyFlag monitors when the device is done with a measurement
    
    if(monitorBusyFlag) {
        busyFlag = 1; //-- Begin read immediately if not monitoring busy flag
    }
    
    int busyCounter = 0; //-- busyCounter counts number of times busy flag is checked, for timeout

    while(busyFlag != 0) { //-- Loop until device is not busy
        //-- Read status register to check busy flag
        Wire.beginTransmission((int)lidarliteAddress);
        Wire.write(0x01); //-- Set the status register to be read

        //-- A nack means the device is not responding, report the error over serial
        int nackCatcher = Wire.endTransmission();
        if(nackCatcher != 0) {
            Serial.println("> nack");
        }

        Wire.requestFrom((int)lidarliteAddress,1);  //-- Read register 0x01
        busyFlag = bitRead(Wire.read(),0);  //-- Assign the LSB of the status register to busyFlag

        busyCounter++;  //-- Increment busyCounter for timeout

        //-- Handle timeout condition, exit while loop and goto bailout
        if(busyCounter > 9999) {
            goto bailout;
        }
    }

    //-- Device is not busy, begin read
    if(busyFlag == 0) {
        Wire.beginTransmission((int)lidarliteAddress);
        Wire.write((int)myAddress); //-- Set the register to be read

        //-- A nack means the device is not responding, report the error over serial
        int nackCatcher = Wire.endTransmission();
        if(nackCatcher != 0) {
            Serial.println("> nack");
        }

        //-- Perform read of 1 or 2 bytes, save in arrayToSave
        Wire.requestFrom((int)lidarliteAddress, numOfBytes);
        int i = 0;
        if(numOfBytes <= Wire.available()) {
            while(i < numOfBytes) {
                arrayToSave[i] = Wire.read();
                i++;
            }
        }
    }

    //-- bailout reports error over serial
    if(busyCounter > 9999) {
        bailout:
            busyCounter = 0;
            Serial.println("> read failed");
    }
} /* LIDARLite::read */

/*------------------------------------------------------------------------------
  Correlation Record To Serial

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  Select memory bank by writing 0xc0 to register 0x5d
  3.  Set test mode select by writing 0x07 to register 0x40
  4.  For as many readings as you want to take (max is 1024)
      1.  Read two bytes from 0xd2
      2.  The Low byte is the value from the record
      3.  The high byte is the sign from the record

  Parameters
  ------------------------------------------------------------------------------
  separator: the separator between serial data words
  numberOfReadings: Default: 256. Maximum of 1024
  lidarliteAddress: Default 0x62. Fill in new address here if changed. See
    operating manual for instructions.
------------------------------------------------------------------------------*/
void LIDARLite::correlationRecordToSerial(char separator, int numberOfReadings, char lidarliteAddress)
{
    byte correlationArray[2];   //-- Array to store read values
    int correlationValue = 0;   //-- Var to store value of correlation record

    write(0x5d,0xc0,lidarliteAddress);  //-- Selects memory bank
    write(0x40, 0x07,lidarliteAddress); //-- Test mode enable
  
    for(int i = 0; i<numberOfReadings; i++) {
        //-- Select single byte
        read(0xd2,2,correlationArray,false,lidarliteAddress);
        //--  Low byte is the value of the correlation record
        correlationValue = correlationArray[0];
        //-- if upper byte lsb is set, the value is negative
        if((int)correlationArray[1] == 1) {
            correlationValue |= 0xff00;
        }
        Serial.print((int)correlationValue);
        Serial.print(separator);
    }
    //-- test mode disable
    write(0x40,0x00,lidarliteAddress);
} /* LIDARLite::correlationRecordToSerial */

//===============================================================================
//== new addition by Kyu ========================================================
//===============================================================================
int LIDARLite::getvelocity(char lidarliteAddress)
{
    byte velocitybuf[2], c;
    int velocityValue;
 
    read(0x09, 1, velocitybuf, true, lidarliteAddress);  //-- Read one byte from register 0x09
    
    if(velocitybuf[0] & 0x80) { //-- -128 ~ 127 in two's complement
        velocityValue=((int)((byte)(~velocitybuf[0]+1))*(-1)); // do not change this line
    } else {
        velocityValue = (int) velocitybuf[0];
    } 
    
    return velocityValue;
} /* LIDARLite::getvelocity */


//-- status register bit flag --------------------------------------------------
#define FLAG_PROCESS_ERROR  0x40    //-- 0: no error, 1: system error 
#define FLAG_HEALTH         0x20    //-- 0: error, 1: reference and receiver bias are operational
#define FLAG_SECONDARY      0x10    //-- 0: No secondary return detected, 1: secondary return detected in correlation record
#define FLAG_INVALID_SIGNAL 0x08    //-- 0: Peak detected, 1: not, measurement is invalid
#define FLAG_SIGNAL_OVERFLOW 0x04   //-- 0: no overflow, 1: 
#define FLAG_REFERENCE_OVERFLOW 0x02 //-- 0: no overflow
#define FLAG_BUSY           0x01    //-- 0: ready, 1: busy (measurement)

byte LIDARLite::getdevstatus(char lidarliteAddress)
{
    byte stsbuf[2];
    
    read(0x01, 1, stsbuf, true, lidarliteAddress);

    Serial.println("* Unit status:");
    if(stsbuf[0] & FLAG_PROCESS_ERROR) {
        Serial.println("\t* System error detected during measurement");
    }
    if(stsbuf[0] & FLAG_HEALTH) {
        Serial.println("\t* Reference and receiver bias are operational");
    }
    if(stsbuf[0] & FLAG_SECONDARY) {
        Serial.println("\t* Secondary return detected in correlation record");
    }
    if(stsbuf[0] & FLAG_INVALID_SIGNAL) {
        Serial.println("\t* Peak not detected in correlation record, measurment is invalid");
    }
    if(stsbuf[0] & FLAG_SIGNAL_OVERFLOW) {
        Serial.println("\t* Signal data in correlation record has reached the maximum value before overflow. This occurs with a strong received signal strength");
    }
    if(stsbuf[0] & FLAG_REFERENCE_OVERFLOW) {
        Serial.println("\t* Reference data in correlation record has reached the maximum value before overflow. This occurs periodically");        
    }
    if(stsbuf[0] & FLAG_BUSY) {
        Serial.println("\t* Device is busy taking a measurement");
    }
    return stsbuf[0];
} /* LIDARLite::getdevstatus */


int LIDARLite::getserialno(char lidarliteAddress)
{
    byte senobuf[2];

    read(0x16, 2, senobuf, true, lidarliteAddress);
    this->serialno = (senobuf[0] << 8) + senobuf[1];
    Serial.print("* Unit Serial number : "); Serial.println(this->serialno);
    
    return this->serialno;
} /* LIDARLite::getserialno */

int LIDARLite::getsignalstrength(char lidarliteAddress)
{
    byte strengthbuf[2];

    //-- get received signal strength (SIGNAL_STRENGTH)
    read(0x0E, 1, strengthbuf, true, lidarliteAddress);
    this->signalstrength = (int) strengthbuf[0];

    return this->signalstrength;
}   /* LIDARLite::getsignalstrength() */

// Read distance fast. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int LIDARLite::getdistancefast(bool biasCorrection, char lidarliteAddress)
{
    byte isBusy = 1;
    int distance;
    int loopCount;

    //-- Polling busy bit in status register until device is idle
    while(isBusy)  {
        //-- Read status register
        Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
        Wire.write(0x01);
        Wire.endTransmission();
        Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
        isBusy = Wire.read();
        isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

        loopCount++; // Increment loop counter
        // Stop status register polling if stuck in loop
        if(loopCount > 9999) {
            break;
        }
    }

    //-- Send measurement command
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0X00); // Prepare write to register 0x00
    if(biasCorrection == true) {
        Wire.write(0X04); // Perform measurement with receiver bias correction
    } else {
        Wire.write(0X03); // Perform measurement without receiver bias correction
    }
    Wire.endTransmission();

    // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
    // The I2C transaction finishes before new distance measurement data is acquired.
    // Prepare 2 byte read from registers 0x0f and 0x10
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x8f);
    Wire.endTransmission();

    // Perform the read and repack the 2 bytes into 16-bit word
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
    distance = Wire.read();
    distance <<= 8;
    distance |= Wire.read();

    // Return the measured distance
    return distance;
}   /* LIDARLite::getdistancefast() */

//-- bit arrangement for ACQ_CONFIG_REG ------------------------------------
#define REFERENCE_PROCESS                   0x40    //-- 6th bit
#define DEFAULT_DELAY                       0x20    //-- 5th bit     
#define REFERENCE_FILTER                    0x10    //-- 4th bit
#define QUICK_TERMINATION                   0x08    //-- 3rd bit
#define REFERENCE_ACQUISITION_COUNT         0x04    //-- 2nd bit

#define DEFAULT_PWM_MODE                    0x00
#define STATUS_OUTPUT_MODE                  0x01
#define FIXED_DELAY_PWM_MODE                0x02
#define OSCILLATOR_OUTPUT_MODE              0x03

void LIDARLite::getconfigvalues(char lidarliteAddress)
{
    byte cfgbuf[2];

    read(0x02, 1, cfgbuf, true, lidarliteAddress);   //-- SIG_COUNTVAL
    this->cfgval.sig_countval=(int)cfgbuf[0];

    read(0x04, 1, cfgbuf, true, lidarliteAddress);   //-- ACQ_CONFIG_REG, byte
    this->cfgval.acq_config_reg=cfgbuf[0];
    
    read(0x11, 1, cfgbuf, true, lidarliteAddress);   //-- OUTER_LOOP_COUNT
    this->cfgval.outer_loop_count=(int)cfgbuf[0];
    
    read(0x12, 1, cfgbuf, true, lidarliteAddress);   //-- REF_COUNT_VAL(0x05)
    this->cfgval.ref_count_val=(int)cfgbuf[0];
    
    read(0x1C, 1, cfgbuf, true, lidarliteAddress);   //-- THRESHOLD_BYPASS(0x00)
    this->cfgval.threshold_bypass=(int)cfgbuf[0];
    
    read(0x45, 1, cfgbuf, true, lidarliteAddress);   //-- MEASURE_DELAY
    this->cfgval.measure_delay=(int)cfgbuf[0];
    
    read(0x5D, 1, cfgbuf, true, lidarliteAddress);   //-- ACQ_SETTINGS
    this->cfgval.acq_settings=(int)cfgbuf[0];  

     //-- report configuration -----------------------------------------------------------

    Serial.println("* Current Configuration ...");
    Serial.print("\t* 0x02 SIG_COUNTVAL    : "); Serial.println(this->cfgval.sig_countval);

    byte cfgbyte=this->cfgval.acq_config_reg;
    
    Serial.print("\t* 0x04 ACQ_CONFIG_REG  : 0x"); Serial.println(cfgbyte, HEX);
    
    if(cfgbyte & REFERENCE_PROCESS) {  //-- ACQ_CONFIG_REG[6]
        Serial.println("\t\t* [1] Disable reference process during measurement");
    } else {
        Serial.println("\t\t* [0] Enable reference process during measurement");
    }
    if(cfgbyte & DEFAULT_DELAY) {      //-- ACQ_CONFIG_REG[5]
        Serial.println("\t\t* [1] Use delay from MEASURE_DELAY(0x45) for burst and free running mode");
    } else {
        Serial.println("\t\t* [0] Use default delay for burst and free running mode");
    }
    if(cfgbyte & REFERENCE_FILTER) {   //-- ACQ_CONFIG_REG[4]
        Serial.println("\t\t* [1] Disable reference filter");
    } else {
        Serial.println("\t\t* [0] Enable reference filter");
    }
    if(cfgbyte & QUICK_TERMINATION) {  //-- ACQ_CONFIG_REG[3]
        Serial.println("\t\t* [1] Disalbe measurement quick termination");
    } else {
        Serial.println("\t\t* [0] Enable measurement quick termination");
    }
    if(cfgbyte & REFERENCE_ACQUISITION_COUNT) { //-- ACQ_CONFIG_REG[2])
        Serial.println("\t\t* [1] Use reference acquisition control from REF_COUNT_VAL(0x12)");
    } else {
        Serial.println("\t\t* [0] Use default reference acquistion count of 5");
    }
    
    switch(cfgbyte & 0x03) {    //-- Mode Selection Pin Function Control
    case DEFAULT_PWM_MODE:
        Serial.print("\t\t* Default PWM mode, 0x"); Serial.println(cfgbyte & 0x03);
        break;
    case STATUS_OUTPUT_MODE:
        Serial.print("\t\t* Status output mode, 0x");Serial.println(cfgbyte & 0x03);
        break;
    case FIXED_DELAY_PWM_MODE:
        Serial.print("\t\t* Fixed delay PWM mode, 0x");Serial.println(cfgbyte & 0x03);
        break;
    case OSCILLATOR_OUTPUT_MODE:
        Serial.print("\t\t*Oscillator output mode, 0x");Serial.println(cfgbyte & 0x03);
        break;
    } 
    
    Serial.print("\t* 0x11 OUTER_LOOP_COUNT: "); Serial.println(this->cfgval.outer_loop_count);
    Serial.print("\t* 0x12 REF_COUNT_VAL   : "); Serial.println(this->cfgval.ref_count_val);
    Serial.print("\t* 0x1C THRESHOLD_BYPASS: "); Serial.println(this->cfgval.threshold_bypass);
    Serial.print("\t* 0x45 MEASURE_DELAY   : "); Serial.println(this->cfgval.measure_delay);
    Serial.print("\t* 0x0E ACQ_SETTINGS    : "); Serial.println(this->cfgval.acq_settings);
}   /* LIDARLite::getconfigvalues() */
//== end of new addtions ========================================================
