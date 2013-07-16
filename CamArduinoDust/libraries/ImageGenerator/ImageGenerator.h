/*
 author: Marc Moreno Berengue
 email: marc.morenob@gmail.com
 personal web site: http://marcmorenob.com
*/

#ifndef ImageGenerator_h
#define ImageGenerator_h

#include <Arduino.h>
#include "Adafruit_VC0706.h"

//=========================== defines =========================================
#define MAX_RAW_LEN 64

#define SEND_TIME_BETWEEN_MSGS 100 //ms

#define POLL_TIME 1000 //ms

#define COMPRESSION_RATE 100

//#define DUMMY

//Image type
#define SMALLI

#ifdef BIGI
    #define IM_SIZE VC0706_640x480       // biggest
    #define IM_BUFFER_SIZE 30000
    #define TESTIMG testImageMedium
#endif


#ifdef MEDIUMI
    #define IM_SIZE VC0706_320x240    // medium
    #define IM_BUFFER_SIZE 7500
    #define TESTIMG testImageMedium
#endif

#ifdef SMALLI
    #define IM_SIZE VC0706_160x120     //small
    #define IM_BUFFER_SIZE 4500
    #define TESTIMG testImageSmall
#endif


//=========================== ImageGenerator object ========================

class ImageGenerator {
   public:
      //===== methods
      ImageGenerator();
      void         setup();
      void         setupDummy();
      void         dataGenerator(uint8_t* ptr, uint8_t  maxLen, uint8_t* lenWritten,unsigned long* fDataPeriod);
      void         dataProcessor(uint8_t* ptr, uint8_t  lenRead);

   private:
      void         nextValue(uint8_t* ptr, uint8_t  maxLen, uint8_t* lenWritten,unsigned long* fDataPeriod);
      void         newPicture(uint8_t* ptr, uint8_t  maxLen, uint8_t* lenWritten,unsigned long* fDataPeriod);
      void         takePicture();
      void         takePictureDummy();
      uint8_t      isDataAvailable();
      void         readPicture();
      uint8_t       camStatus;
      
      //Image parameters
      uint16_t      imageSize;
      uint8_t       *ptrImageData;
      uint8_t       imageData[IM_BUFFER_SIZE];
      
      //App parameters
      boolean           isData2Send;             // When capture data is received
      boolean           isAutoMode;              // When auto send images is received
};


#endif ImageGenerator_h

