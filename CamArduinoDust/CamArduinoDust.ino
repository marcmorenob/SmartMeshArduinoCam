/*
Copyright (c) 2013, Dust Networks.  All rights reserved.

Arduino sketch which connects to a SmartMesh IP mote and periodically sends a
2-byte value to the mananger. You can use the SensorDataReceiver application of
the SmartMesh SDK to see that data arrive.

\license See attached DN_LICENSE.txt.
*/

#include <SmartMesh.h>
#include <Adafruit_VC0706.h>
#include <ImageGenerator.h>
#include <SpiUart.h>
#include <SPI.h>
#include <time.h>

//#define DUMMY

ImageGenerator    generator;
SmartMesh         smartmesh;

uint32_t timeA;

//=========================== data generator ==================================

void generateData(uint8_t* ptr, uint8_t maxLen, uint8_t* lenWritten,TIME_T* fDataPeriod) {
   if(generator.isPictureRead()){
        uint32_t timeC = millis();
        Serial.print("New Picture. Last spend: ");
        Serial.print(timeC-timeA,DEC);
        Serial.println("ms");
        #ifdef DUMMY
            generator.takePictureDummy();
        #else
            generator.takePicture();
        #endif
        timeA = millis();
        generator.newPictureMsg(ptr,maxLen,lenWritten,fDataPeriod);
        
   }else{
        Serial.println("NextValue");
        generator.nextValue(ptr,maxLen,lenWritten,fDataPeriod);
   }
}

//=========================== "main" ==========================================

void setup() {
   #ifdef DUMMY
       generator.setupDummy();
   #else
       generator.setup();
   #endif
   
   smartmesh.setup(
      60000,                           // srcPort_param
      (uint8_t*)ipv6Addr_manager,      // destAddr_param
      61000,                           // destPort_param
      100,                              // dataPeriod_param
      generateData                     // dataGenerator_param
   );
}

void loop() {
   smartmesh.loop();
}
