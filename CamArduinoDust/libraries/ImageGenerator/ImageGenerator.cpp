/*
 author: Marc Moreno Berengue
 email: marc.morenob@gmail.com
 personal web site: http://marcmorenob.com
*/

#include "Arduino.h"
#include "ImageGenerator.h"
#include "TestImage.h"
Adafruit_VC0706 cam=Adafruit_VC0706();
//########################### public ##########################################

/**
\brief Constructor.
*/
ImageGenerator::ImageGenerator() 
{
    
  imageSize = 0;
  
  //Flag for capture one picture
  isData2Send=false;
  
  //Flag for auto mode
  isAutoMode=false;
}

/**
\brief Setup function to configure and start the camera
*/
void ImageGenerator::setup()
{  
  Serial.begin(115200);
  Serial.println("Image Constructor");
  
  Serial.println("Start Cam");
  if (cam.begin(38400)){
    camStatus=1;
    Serial.println("Camera Found!!");
  }else {
    camStatus=0; 
    Serial.println("No camera found at 38400?");
    return;
  }
  
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    camStatus=0; 
    Serial.print("Failed to get version");
  } 
  else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }
  
  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  cam.setImageSize(IM_SIZE);

  delay(1);
  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  cam.setCompression(COMPRESSION_RATE);

  uint8_t c = cam.getCompression();
  Serial.print("Compression: ");
  Serial.println(c,DEC);
   
  Serial.end();
  
}

/**
\brief Dummy fonction for setup
*/
void ImageGenerator::setupDummy()
{
  Serial.begin(115200);
  Serial.println("Image Constructor");
  Serial.println("Start Dummy Cam");
  Serial.end();
}

/* Message to sync py App between arduino. NewPicture send a 
 * specific message to py App. That message has the images size and
 * a human message. All data is less thant the maxim user data 
 * abailable. So, only 1 message is needed to sync both
 * applications.
 * 
 * @param prt         Pointer to the smartMesh message payload
 * @param maxLen      Maximum user data
 * @param lenWritten  Data wrote in the use data payload zone
 * @param fDataPeriod Time between consecutive smartMesh message.
 * 
 */
void ImageGenerator::newPicture(
      uint8_t* ptr,
      uint8_t  maxLen,
      uint8_t* lenWritten,
      unsigned long* fDataPeriod
   ) {
 
  // Send out the camera version information (optional)
  char msg[]="New picture";
  Serial.println(msg);
  memcpy(ptr,msg,sizeof(msg));
  memcpy(ptr+sizeof(msg),&imageSize,sizeof(imageSize));
  *lenWritten=sizeof(msg);
  *fDataPeriod = SEND_TIME_BETWEEN_MSGS;  
  
}

/**
\brief Take picture, read and store that imag in-memory
*/
void ImageGenerator::takePicture()
{
    //Take a picture
    cam.takePicture();
    
    imageSize = cam.frameLength();
    Serial.print("Image Size: ");
    Serial.println(imageSize);
    readPicture();
    Serial.print("Image read");
}

/**
\brief Take picture and read image from camera buffer
*/
void ImageGenerator::takePictureDummy()
{
    //Load image from TestImage.h
    imageSize=sizeof(TESTIMG);
    memcpy(imageData,TESTIMG,imageSize);
    delay(200);
    
    //First position to read values
    ptrImageData = imageData;
}

/**
\brief Read image from camera buffer and store it in CamBuffer
*/
void ImageGenerator::readPicture(){

    uint16_t data2read;
    uint8_t *ptrCamBuffer;
    
    //First position to store the image
    ptrImageData = imageData;
    
    //Number of bytes to be read
    data2read = imageSize;
    
    do{
        if(data2read>=MAX_RAW_LEN){
            ptrCamBuffer = cam.readPicture(MAX_RAW_LEN);
            memcpy(ptrImageData,ptrCamBuffer,MAX_RAW_LEN);
            data2read -= MAX_RAW_LEN;
            ptrImageData += MAX_RAW_LEN;
        }else{
            ptrCamBuffer = cam.readPicture(data2read);
            memcpy(ptrImageData,ptrCamBuffer,data2read);
            data2read = 0;
            //Free camera buffer
            cam.resumeVideo();
        }
    }while (data2read > 0);
    
    //First position to read values
    ptrImageData = imageData;
}  

/**
\brief Get the next values from the image.
*/
void ImageGenerator::nextValue(
      uint8_t* ptr,
      uint8_t  maxLen,
      uint8_t* lenWritten,
      unsigned long* fDataPeriod
   ) {
   
    if(imageSize>=maxLen){
        memcpy(ptr,ptrImageData,maxLen);
        imageSize -= maxLen;
        *lenWritten = maxLen;
        
        //Next position to read values
        ptrImageData += maxLen;
    }else{
        memcpy(ptr,ptrImageData,imageSize);
        *lenWritten = imageSize;
        imageSize = 0;
        //Just in case isData2Send is true. Only take 1 picture
        isData2Send=false;
    }
    
    *fDataPeriod = SEND_TIME_BETWEEN_MSGS;
}

/*
* Check if there are data to be read or a new picture can be capatured 
* 
*/
uint8_t ImageGenerator::isDataAvailable()
{
    if(imageSize>0){
        return 1;
    }
    return 0;
}


/* Read new data from buffer or take a new picture if no data 
 * available
 * 
 * @param prt         Pointer to the smartMesh message payload
 * @param maxLen      Maximum user data
 * @param lenWritten  Data wrote in the use data payload zone
 * @param fDataPeriod Time between consecutive smartMesh message.
 * 
 */
void ImageGenerator::dataGenerator(uint8_t* ptr, uint8_t  maxLen, uint8_t* lenWritten,unsigned long* fDataPeriod){

   if(!camStatus){
      *lenWritten = 0;
      *fDataPeriod = POLL_TIME;
   }
   
   if(!isData2Send && !isAutoMode){ //Nothing to do
            *lenWritten=0;
            *fDataPeriod = POLL_TIME; //Poll every second
            return;
   }
   
   if(isDataAvailable()){
        Serial.println("NextValue");
        nextValue(ptr,maxLen,lenWritten,fDataPeriod);
   }else{
        Serial.print("New Picture.");
        #ifdef DUMMY
            takePictureDummy();
        #else
            takePicture();
        #endif
        newPicture(ptr,maxLen,lenWritten,fDataPeriod);
   }

}
 
 
/* Receive data call back. This function process all messages received
*
* @param prt      Pointer to the smartMesh message received payload
* @param lenRead  Data to be read
* 
*/   

void ImageGenerator::dataProcessor(uint8_t* ptr, uint8_t  lenRead){

      uint8_t cmd;
      
      cmd = *ptr;
      
      switch(cmd){
        case 0xC1: //Capture 1 (C1)
            Serial.println("Capture Images");
            isData2Send=true;
            break;
        case 0xCA: //Capture Automatic (CA)
            Serial.println("Start automatic");
            isAutoMode=true;
            break;
        case 0xC0:  //Capture 0 (C0)
            Serial.println("Stop automatic");
            isAutoMode = false;
            break;      
        default:
            Serial.print("Unknown command: ");
            Serial.println(cmd,HEX);
            break;      
      }
}


//########################### private #########################################
