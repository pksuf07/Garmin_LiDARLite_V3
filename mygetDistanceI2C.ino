/******************************************************************************
  Garmin LiDAR Lite v3
  -----------------------------------------------------------------------------
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  -----------------------------------------------------------------------------
  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND
  -----------------------------------------------------------------------------
  Things to do:
      - Need to implement full function of Garmin LiDAR Lite v3
  -----------------------------------------------------------------------------
  Changes in library
      1) Power Control Function:
          - need to use power enable (internal pull-up) pin
      2)  
*******************************************************************************/
#include <Wire.h>
#include <math.h>

#include "LIDARLitev3.h"

//-- application types --------------------------------------------------------
#define GET_DISTANCE    1           //-- basic proof of function, distance only
// #define GET_VELOCITY    2           //-- basic proof of function, distance and velocity
// #define GET_BENCHMARK   3        //-- get statistics
// #define GET_CORRELATION 4        //-- get correclation record

LIDARLite myLidarLite;

#ifdef GET_BENCHMARK                //-- data structure for benchmarking
    long    elapsed, st;
    float   maxrate;

    typedef struct {
        float avg, sd;
        int measurement[100];
    } burst_statistics_t;
    int burst_idx=0;
    float avg_all=0., sd_all=0.;
    burst_statistics_t burst_data[100];
#endif

//== setup() =====================================================================
void setup()
{
    Serial.begin(115200); // Initialize serial connection to display distance readings

    Serial.println("\n\tStarting Garmin LiDAR Lite v3 ...\n");
    myLidarLite.begin(0, true);     //-- Set configuration to default and I2C to 400 kHz
    myLidarLite.configure(0);       //-- Change this number to try out alternate configurations
    
    myLidarLite.getserialno();      //-- get serial number of unit
    myLidarLite.getdevstatus();     //-- get current status of device
    myLidarLite.getconfigvalues();  //-- get the configuration
     
#ifdef GET_DISTANCE
    Serial.print("\nType any key to proceed...");
    while(!Serial.available()) {
        char c = Serial.read();
        if(c=='s') break;
    }
    Serial.flush();
    while(Serial.available()) Serial.read();
#endif
}

//== GET_CORRELATION ==================================================================
#ifdef GET_CORRELATION
void loop() 
{
    Serial.println("\nGetting 512 correlaion record...\n");
    myLidarLite.correlationRecordToSerial(',' , 512);
    
    Serial.println("\n\nType any key to get correlation record again ...");
    while(!Serial.available()) {
        char c = Serial.read();
        if(c=='s') break;
    }
    Serial.flush();
    while(Serial.available()) Serial.read();
}
#endif

//== GET_BENCHMARK ====================================================================
#ifdef GET_BENCHMARK            //-- get (100) burst based benchmarking -----------
void loop() //-- loop by burst
{
    Serial.print("[");Serial.print(burst_idx);Serial.println("] Starting burst benchmarking");
    
    st=micros();
    
    burst_data[burst_idx].measurement[0]=myLidarLite.getdistance();   //-- with RBC
    for(int i=1; i<100; i++)
        burst_data[burst_idx].measurement[i]=myLidarLite.getdistance(false);
    elapsed = micros() - st;
    Serial.print("* elapsed time (microseconds) for 100 measurements = ");
    Serial.println(elapsed);
    Serial.print("* measurement rate = ");
    Serial.print(1./(elapsed/1000000.)*100.);
    Serial.println("Hz");    

    //-- analysis the 100 measurements while the unit is fixed in to one location
    analyzeit_1st_order(burst_idx);
    
    burst_idx++;    
    
    if(burst_idx>=100) {
        
        analyzeit_2nd_order();

        report_raw_value_for_further_analysis();
        
        Serial.print("\nType any key to perform burst analysis again ...");
        while(!Serial.available()) {
            char c = Serial.read();
            if(c=='s') break;
        }
        Serial.flush();
        while(Serial.available()) Serial.read();
        burst_idx=0;
    }

}   /* loop() for BENCHMARK */
#endif

//== GET_DISTANCE ===========================================================================
#ifdef GET_DISTANCE                 //-- basic functions ----------------------------------------
void loop()
{
    //-- every 100 sequential measurements, receiver bias correction should be performed!
    Serial.println(myLidarLite.getdistancefast());
    for(int i = 0; i < 99; i++) {
        Serial.print(myLidarLite.getdistancefast(false));
        Serial.print(",");
        Serial.print(myLidarLite.getvelocity(), DEC);
        Serial.print(",");
        Serial.print(myLidarLite.getsignalstrength(), DEC);
        Serial.println("");
    }    
}   /* loop() */
#endif

//== GET_BENCHMARK =========================================================================
#ifdef GET_BENCHMARK        //-- basic descriptive statistics ---------------------------
void analyzeit_1st_order(int bidx)  //-- 1st order analaysis for each burst (100 measurement)
{
    //-- we already have 100 measurement
    float avg, median, sd;
    float sum=0., devsum=0.;

    for(int i=0; i<100; i++) 
        sum+=burst_data[bidx].measurement[i];
    burst_data[bidx].avg = sum/100.;
    for(int i=0; i<100; i++) 
        devsum +=pow(burst_data[bidx].measurement[i]-burst_data[bidx].avg, 2);
    burst_data[bidx].sd = sqrt(devsum/99.);
    
    Serial.println("* Simple statistics:");
    Serial.print("\t* Average (100 measurement) : "); Serial.println(burst_data[bidx].avg);
    Serial.print("\t* Standard deviation        : "); Serial.println(burst_data[bidx].sd);
}   /* analyzeit_1st_order() */

void analyzeit_2nd_order()
{
    //-- we already have 100 measurement
    float sum=0., devsum=0.;

    for(int i=0; i<100; i++) {
        for(int j=0; j<100; j++) {
            sum+=burst_data[i].measurement[j];
        }
    }
    avg_all = sum/10000.;
    for(int i=0; i<100; i++) {
        for(int j=0; j<100; j++) {
            devsum +=pow(burst_data[i].measurement[j]-avg_all, 2);
        }
    }
    sd_all = sqrt(devsum/10000.);
    Serial.println("* \nSimple statistics for all bursts:");
    Serial.print("\t* Average (100 measurement) : "); Serial.println(avg_all);
    Serial.print("\t* Standard deviation        : "); Serial.println(sd_all);
}   /* analyzeit_2nd_order() */
#endif
//== end of program ===================================================================
