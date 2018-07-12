/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  LIDARLite.h

  This library provides quick access to all the basic functions of LIDAR-Lite
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
#ifndef LIDARLitev3_h
#define LIDARLitev3_h
#include <Arduino.h>

#define LIDARLITE_ADDR_DEFAULT 0x62

//== Important, Garmin LiDAR Lite v3 Register Map ======================================== 
#define ACQ_COMMAND     0x00    //-- "Receiver Bias Correction", 0x03: without, 0x04: with
                                //-- every 100 sequential measurements
                                //-- To achieve good performance at high measurement rates
                                //-- receiver bias correction must be performed periodically
#define STATUS          0x01    //-- status
#define SIG_COUNT_ALL   0x02    //-- "Maximum Acquistion Count"(initial value: 0x80)
                                //-- rate = 1/n, range=n^(1/4), where n = the number of acquisition
#define ACQ_CONFIG_REG  0x04    //-- "Measurement Quick Termination Detection", Acquisition mode control
                                //-- enable/disable quick-termination detection
                                //-- This allows for aster and slightly less accurate operation
                                //-- at strong signal strengths without sacrificing long range performance
#define THRESHOLD_BYPASS 0x1C   //-- "Detection Sensitivity"
                                //-- Peak detection threshold bypass
                                //-- by setting non-zero value, the algorithm by simple threshold
                                //-- default alrogithm is based on the peak value, signal strength,
                                //-- and noise in the correlation record.
                                //-- Recommendation:
                                //--    0x20 for higher sensitivity with more frequent erroneous measurement
                                //--    0x60 for reduced sensitivity and fewer erroneous measurement
                                
                                //-- Burst Measurements and Free Running Mode
                                //-- the device can be configured to take multiple measurements for each
                                //-- measurement command or repeat indefinitely for free running mode
#define OUTER_LOOP_COUNT 0x11   //-- Burst Measurement Count Control
                                //-- control the number of times the device will retrigger itsef.
                                //-- default one measurement per one command
                                //-- 0x02 ~ 0xfe, Value 0xff will enable free running mode 
#define MEASURE_DELAY   0x45   //-- Delay between automatic measurements, default : 10Hz, 
                                //-- A delay value of 0x14 roughly corresponds to 100Hz
#define VELOCITY        0x09    //-- "Velocity measurement output"
                                //-- the difference between the current measurement and the previous one
                                //-- signed (2's complement) 8 bit number in cm
                                //-- + : away from the device
                                //-- This can be combined with free running mode for a constant measurement frequency
                                //-- the default free running frequency of 10Hz therefore results in
                                //-- a velocity measurement in 1m/s                                
//== End of Register Map =================================================================

typedef struct {
    short sig_countval;
    byte acq_config_reg;
    short outer_loop_count;
    short ref_count_val;
    short threshold_bypass;
    short measure_delay;
    short acq_settings;
} configure_t;


#define POWER_ENABLE_PIN D3     //-- default power enable pin 

class LIDARLite
{
    private:
        int  serialno;
        int  signalstrength;
        byte power_enable_pin = POWER_ENABLE_PIN;
        configure_t cfgval;
        
    public:
        LIDARLite();
        void begin(int = 0, bool = false, char = LIDARLITE_ADDR_DEFAULT);
        void configure(int = 0, char = LIDARLITE_ADDR_DEFAULT);
        void reset(char = LIDARLITE_ADDR_DEFAULT);
        int  getdistance(bool = true, char = LIDARLITE_ADDR_DEFAULT);   //-- changed the name by kyu
        void write(char, char, char = LIDARLITE_ADDR_DEFAULT);
        void read(char, int, byte*, bool, char);
        void correlationRecordToSerial(char = '\n', int = 256, char = LIDARLITE_ADDR_DEFAULT);
        
        //-- new addition by Kyu -------------------------------------------
        int  getvelocity(char = LIDARLITE_ADDR_DEFAULT);
        byte getdevstatus(char = LIDARLITE_ADDR_DEFAULT);
        int  getserialno(char = LIDARLITE_ADDR_DEFAULT);
        int  getsignalstrength(char = LIDARLITE_ADDR_DEFAULT);
        int  getdistancefast(bool = true, char = LIDARLITE_ADDR_DEFAULT);
        void getconfigvalues(char = LIDARLITE_ADDR_DEFAULT);
        //-- end of addition -----------------------------------------------
};

#endif
