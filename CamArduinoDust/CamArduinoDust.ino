
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

ImageGenerator    generator;
SmartMesh         smartmesh;

//=========================== data generator ==================================

void generateData(uint8_t* ptr, uint8_t maxLen, uint8_t* lenWritten,TIME_T* fDataPeriod) {
   generator.dataGenerator(ptr,maxLen,lenWritten,fDataPeriod);
}

void processData(uint8_t* ptr, uint8_t lenRead) {
   generator.dataProcessor(ptr,lenRead);
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
      1000,                            // dataPeriod_param
      generateData,                    // dataGenerator_param
      processData                      // dataProcessor_param
      );
}

void loop() {
   smartmesh.loop();
}
