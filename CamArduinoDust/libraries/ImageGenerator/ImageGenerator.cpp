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
    
}

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

  cam.setCompression(100);

  uint8_t c = cam.getCompression();
  Serial.print("Compression: ");
  Serial.println(c,DEC);

  imageSize = 0;
  Serial.end();
}

void ImageGenerator::setupDummy()
{
  Serial.begin(115200);
  Serial.println("Image Constructor");
  Serial.println("Start Dummy Cam");
  Serial.end();
}

void ImageGenerator::newPictureMsg(
      uint8_t* ptr,
      uint8_t  maxLen,
      uint8_t* lenWritten,
      unsigned long* fDataPeriod
   ) {
 
  // Send out the camera version information (optional)
  char msg[]="New picture";
  Serial.println(msg);
  memcpy(ptr,msg,sizeof(msg));
  *lenWritten=sizeof(msg);
  *fDataPeriod = 100;  
  
}

//Take picture and read image from camera buffer
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

//Take picture and read image from camera buffer
void ImageGenerator::takePictureDummy()
{
    //Load image from TestImage.h
    imageSize=sizeof(TESTIMG);
    memcpy(imageData,TESTIMG,imageSize);
    delay(200);
    //First position to read values
    ptrImageData = imageData;
}

//Read image from camera buffer
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
    }
    
    *fDataPeriod = READ_IMG_DELAY;
}

uint8_t ImageGenerator::isPictureRead()
{
    if(imageSize>0){
        return 0;
    }
    return 1;
}
//########################### private #########################################
