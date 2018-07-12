# Garmin_LiDARLite_V3
Garmin LiDARLite V3 on Arduino

NOTE: This minor code is completely based on https://github.com/garmin/LIDARLite_Arduino_Library I would say there is no major changes or additions to the original library but simple functionalities to collect measurement results and some statistics.

I added following methods to the LIDARLite class:

        //-- new addition by Kyu -------------------------------------------
        int  getvelocity(char = LIDARLITE_ADDR_DEFAULT);        
        byte getdevstatus(char = LIDARLITE_ADDR_DEFAULT);
        int  getserialno(char = LIDARLITE_ADDR_DEFAULT);
        int  getsignalstrength(char = LIDARLITE_ADDR_DEFAULT);
        int  getdistancefast(bool = true, char = LIDARLITE_ADDR_DEFAULT);
        void getconfigvalues(char = LIDARLITE_ADDR_DEFAULT);
        //-- end of addition -----------------------------------------------

"mygetDistanceI2C.ino" is also based on the example code "getDistanceI2C.ino" in Garmin library. I added several different version of loop functions for different purpose.

One can choose a different loop() function by uncomment one of followig definision. The source code itself is simple and self exaplanatory. 

        //-- application types --------------------------------------------------------
        #define GET_DISTANCE    1           //-- basic proof of function, distance only
        // #define GET_VELOCITY    2           //-- basic proof of function, distance, signal strength, and velocity
        // #define GET_BENCHMARK   3        //-- get statistics
        // #define GET_CORRELATION 4        //-- get correclation record

The simple benchmarks are coded to get basic statistics for each 100 measurements to the wall/object at fixed distance. This simple codes will be expanded further to get more interesting numbers. Enjoy.
