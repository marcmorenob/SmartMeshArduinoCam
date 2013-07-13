/*
Copyright (c) 2013, Dust Networks.  All rights reserved.

Arduino library to connect to a SmartMesh IP mote and periodically send data.

This library will:
- Connect to the SmartMesh IP mote over its serial port.
- Have the SmartMesh IP mote connect to a SmartMesh IP network, open and bind a
  UDP socket
- Periodically, invoke a data generation function and send the generated
  payload to the specified IPv6 address and UDP port.
  
\license See attached DN_LICENSE.txt.
*/

#include "Arduino.h"
#include "ImageGenerator.h"
#include "SmartMesh.h"
#include "SpiUart.h"



#if defined(UBRR1H)  // DJE: If the microcontroller has a second UART (i.e. Due):
  HardwareSerial& SerialAPI=Serial1;  // Use Serial1 for SerialAPI
  HardwareSerial& SerialDebug=Serial; // Use Serial for SerialDebug
#else // If the microcontroller only has one UART (i.e. Linduino/Uno)
  SpiUartDevice SerialAPI;  // Use external SPI UART for SerialAPI
  HardwareSerial& SerialDebug=Serial; // Use Serial for SerialDebug
#endif

#define _DEBUG_SMARTMESH_

#if defined(_DEBUG_SMARTMESH_)  // If _DEBUG_SMARTMESH_ is defined, Print debug statements.
    #define SerialDebugPrint(...) Serial.print(__VA_ARGS__) 
    #define SerialDebugPrintln(...) Serial.println(__VA_ARGS__) 
#else
    #define SerialDebugPrint(...) 
    #define SerialDebugPrintln(...)  
#endif

//########################### public ##########################################

/**
\brief Constructor.
*/
SmartMesh::SmartMesh() {
}

/**
\brief Setting up the instance.
*/
void SmartMesh::setup(
      uint16_t       srcPort_param,
      uint8_t*       destAddr_param,
      uint16_t       destPort_param,
      TIME_T         dataPeriod_param,
      data_generator dataGenerator_param
   ) {
   // reset local variables
   memset(&hdlc_vars,   0, sizeof(hdlc_vars));
   memset(&serial_vars, 0, sizeof(serial_vars));
   serial_vars.txPacketId = SERIAL_PACKETID_NOTSET;
   serial_vars.rxPacketId = SERIAL_PACKETID_NOTSET;
   memset(&fsm_vars,    0, sizeof(fsm_vars));
   
   // store params
   srcPort              = srcPort_param;
   memcpy(destAddr,destAddr_param,IPv6ADDR_LEN);
   destPort             = destPort_param;
   dataPeriod           = dataPeriod_param;
   dataGenerator        = dataGenerator_param;
   
   // initialize the serial port connected to the computer
   //Serial.begin(BAUDRATE_CLI);
   //DJE: Changed BAUDRATE_CLI to BAUDRATE_DEBUG
   Serial.begin(BAUDRATE_DEBUG);
   //If SerialSPI is being used, make pin 4 (SPI chip select) an output.
   #if !defined(UBRR1H)
      digitalWrite(4,HIGH);
      pinMode(4,OUTPUT);
   #endif
   
   // initialize the serial port connected to the mote
   Serial1.begin(BAUDRATE_API);
   
   // print banner
   SerialDebugPrintln("");
   SerialDebugPrint(" SmartMesh Library, version ");
   SerialDebugPrint(VERSION_MAJOR);
   SerialDebugPrint(".");
   SerialDebugPrint(VERSION_MINOR);
   SerialDebugPrint(".");
   SerialDebugPrint(VERSION_PATCH);
   SerialDebugPrint(".");
   SerialDebugPrint(VERSION_BUILD);
   SerialDebugPrintln(F(" (c) Dust Networks, 2013."));
   SerialDebugPrintln("");
   
   // schedule first event
   fsm_scheduleIn(2*CMD_PERIOD, &SmartMesh::fsm_getMoteStatus);
}

void SmartMesh::loop() {
   byte    temp;
   TIME_T  currentTime;
   
   // receive and react to HDLC frames
   while (Serial1.available()) {
      
      // read a serial byte
      temp = uint8_t(Serial1.read());
      
      // hand over byte to HDLC receiver
      hdlc_receiver(temp);
      
      // hand over complete HDLC frame to serial receiver, if any
      if (hdlc_vars.busyReceiving==FALSE && hdlc_vars.inputBufFill>0) {
         
         serial_receiver();
         
         // clear inputBuffer
         hdlc_vars.inputBufFill=0;
      }
   }
   
   // kick the fsm
   currentTime = millis();
   if (fsm_vars.delay>0 && (currentTime-fsm_vars.previousEvent>fsm_vars.delay)) {
      // cancel event
      fsm_vars.delay=0;
      
      // handle event (by calling the callback)
      (this->*fsm_vars.cb)();
   }
}

//########################### private #########################################

//=========================== hdlc ============================================

void SmartMesh::hdlc_receiver(uint8_t rxbyte) {
   
   if        (
                hdlc_vars.busyReceiving==FALSE  &&
                hdlc_vars.lastRxByte==HDLC_FLAG &&
                rxbyte!=HDLC_FLAG
              ) {
      // start of frame
      
      // I'm now receiving
      hdlc_vars.busyReceiving         = TRUE;
      
      // create the HDLC frame
      inputHdlcOpen();
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
   } else if (
                hdlc_vars.busyReceiving==TRUE   &&
                rxbyte!=HDLC_FLAG
             ) {
      // middle of frame
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
      if (hdlc_vars.inputBufFill+1>HDLC_INPUT_BUFFER_SIZE){
         // input buffer overflow
         SerialDebugPrintln(F("ERROR input_buffer_overflow"));
         hdlc_vars.inputBufFill       = 0;
         hdlc_vars.busyReceiving      = FALSE;
      }
   } else if (
                hdlc_vars.busyReceiving==TRUE   &&
                rxbyte==HDLC_FLAG
              ) {
         // end of frame
         
         // finalize the HDLC frame
         inputHdlcClose();
         
         if (hdlc_vars.inputBufFill==0){
            // invalid HDLC frame
            SerialDebugPrintln(F("ERROR invalid CRC"));
         }
         
         hdlc_vars.busyReceiving      = FALSE;
   }
   
   hdlc_vars.lastRxByte = rxbyte;
}

//===== output

/**
\brief Start an HDLC frame in the output buffer.
*/
inline void SmartMesh::outputHdlcOpen() {
   // initialize the value of the CRC
   hdlc_vars.outputCrc                                 = HDLC_CRCINIT;
   
   // write opening HDLC flag
   Serial1.write(HDLC_FLAG);
}
/**
\brief Add a byte to the outgoing HDLC frame being built.

\todo escape 0x7e and 0x7d.
*/
inline void SmartMesh::outputHdlcWrite(uint8_t b) {
   
   // iterate through CRC calculator
   hdlc_vars.outputCrc = crcIteration(hdlc_vars.outputCrc,b);
   
   // write optional escape byte
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      Serial1.write(HDLC_ESCAPE);
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   
   // data byte
   Serial1.write(b);
}
/**
\brief Finalize the outgoing HDLC frame.
*/
inline void SmartMesh::outputHdlcClose() {
   uint16_t   finalCrc;
    
   // finalize the calculation of the CRC
   finalCrc   = ~hdlc_vars.outputCrc;
   
   // write the CRC value
   outputHdlcWrite((finalCrc>>0)&0xff);
   outputHdlcWrite((finalCrc>>8)&0xff);
   
   // write closing HDLC flag
   Serial1.write(HDLC_FLAG);
}

//===== input

/**
\brief Start an HDLC frame in the input buffer.
*/
inline void SmartMesh::inputHdlcOpen() {
   // reset the input buffer index
   hdlc_vars.inputBufFill                       = 0;
   
   // initialize the value of the CRC
   hdlc_vars.inputCrc                           = HDLC_CRCINIT;
}
/**
\brief Add a byte to the incoming HDLC frame.

\todo escape 0x7e and 0x7d.
*/
inline void SmartMesh::inputHdlcWrite(uint8_t b) {
   if (b==HDLC_ESCAPE) {
      hdlc_vars.inputEscaping = TRUE;
   } else {
      if (hdlc_vars.inputEscaping==TRUE) {
         b                             = b^HDLC_ESCAPE_MASK;
         hdlc_vars.inputEscaping = FALSE;
      }
      
      // add byte to input buffer
      hdlc_vars.inputBuf[hdlc_vars.inputBufFill] = b;
      hdlc_vars.inputBufFill++;
      
      // iterate through CRC calculator
      hdlc_vars.inputCrc = crcIteration(hdlc_vars.inputCrc,b);
   }
}
/**
\brief Finalize the incoming HDLC frame.
*/
inline void SmartMesh::inputHdlcClose() {
   
   // verify the validity of the frame
   if (hdlc_vars.inputCrc==HDLC_CRCGOOD) {
      // the CRC is correct
      
      // remove the CRC from the input buffer
      hdlc_vars.inputBufFill    -= 2;
   } else {
      // the CRC is incorrect
      
      // drop the incoming fram
      hdlc_vars.inputBufFill     = 0;
   }
}

//===== helpers

uint16_t SmartMesh::crcIteration(uint16_t crc, uint8_t byte) {
   return (crc >> 8) ^ hdlc_fcstab[(crc ^ byte) & 0xff];
}

//=========================== serial ==========================================

//===== TX

void SmartMesh::serial_sendRequest(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   uint8_t i;
   
   // log
#ifdef DEBUG
   SerialDebugPrint(F("DEBUG:    serial_sendRequest "));

   printByteArray(payload, length);
#endif   
   // send the frame over serial
   outputHdlcOpen();
   outputHdlcWrite(cmdId);                                                     // cmdId
   outputHdlcWrite(length);                                                    // length
   if (serial_vars.txPacketId==SERIAL_PACKETID_NOTSET) {                       // flags
      serial_vars.txPacketId = 0x00;
      outputHdlcWrite(0x01<<3 | serial_vars.txPacketId<<1);
   } else {
      outputHdlcWrite(          serial_vars.txPacketId<<1);
   }
   for (i=0;i<length;i++) {                                                    // payload
      outputHdlcWrite(payload[i]);
   }
   outputHdlcClose();
   
   // toggle the txPacketId
   if (serial_vars.txPacketId==0x00) {
      serial_vars.txPacketId = 0x01;
   } else {
      serial_vars.txPacketId = 0x00;
   }
}

void SmartMesh::serial_sendReply(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   uint8_t i;
   
   // log
   SerialDebugPrint(F("DEBUG:    serial_sendReply cmdId="));
   SerialDebugPrint(cmdId);
   SerialDebugPrint(F(" rxPacketId="));
   SerialDebugPrint(serial_vars.rxPacketId);

   printByteArray(payload, length);
   
   outputHdlcOpen();
   outputHdlcWrite(cmdId);                                                     // cmdId
   outputHdlcWrite(length);                                                    // length
   outputHdlcWrite(SERIAL_API_MASK_RESPONSE | (serial_vars.rxPacketId<<1));    // flags
   outputHdlcWrite(SERIAL_RC_OK);                                              // rc
   for (i=0;i<length;i++) {                                                    // payload
      outputHdlcWrite(payload[i]);
   }
   outputHdlcClose();
}

//===== RX

void SmartMesh::serial_receiver() {
   // fields in the serial API header
   uint8_t cmdId;
   uint8_t length;
   uint8_t isResponse;
   uint8_t packetId;
   // misc
   uint8_t tempByte;
   uint8_t isRepeatId;
   uint8_t updateRxPacketId;
   uint8_t i;
   
   // log
   SerialDebugPrint(F("DEBUG:    serial_receiver "));

   printByteArray(hdlc_vars.inputBuf, hdlc_vars.inputBufFill);
   
   // assert length is OK
   if (hdlc_vars.inputBufFill<3) {
      SerialDebugPrintln(F("WARNING:  not enough serial API header bytes"));
      return;
   }
   
   // parse header
   cmdId      = hdlc_vars.inputBuf[0];
   length     = hdlc_vars.inputBuf[1];
   tempByte   = hdlc_vars.inputBuf[2];
   isResponse = ((tempByte & SERIAL_API_MASK_RESPONSE)!=0);
   packetId   = ((tempByte & SERIAL_API_MASK_PACKETID)!=0);
   
   // check if valid packet ID
   if (isResponse) {
      
      // dispatch
      if (length>0) {
         serial_dispatch_response(cmdId,&hdlc_vars.inputBuf[3],length);
      }
      
   } else {
      if (packetId==serial_vars.rxPacketId) {
         SerialDebugPrintln(F("WARNING:  isRepeatId"));
         isRepeatId             = TRUE;
      } else {
         isRepeatId             = FALSE;
         serial_vars.rxPacketId = packetId;
      }
      
      // ACK
      serial_sendReply(cmdId,NULL,0);
      
      // dispatch
      if (length>0) {
         serial_dispatch_request(cmdId,&hdlc_vars.inputBuf[3],length);
      }
   }
}

void SmartMesh::serial_dispatch_request(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   
   switch (cmdId) {
      case SERIAL_CMDID_EVENT:
         serial_rx_event(payload,length);
         break;
      default:
         // log
         SerialDebugPrint(F("WARNING:  no handler for cmdId "));
         SerialDebugPrint(cmdId,DEC);
         SerialDebugPrint("\n");
         SerialDebugPrint(F("length="));
         SerialDebugPrintln(length,DEC);
         SerialDebugPrint(F("payload="));
         printByteArray(payload, length);
         break;
   }
}

void SmartMesh::serial_dispatch_response(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   uint8_t rc;
   
   rc = payload[0];
   if (rc==SERIAL_RC_OK) {
      if (cmdId==fsm_vars.replyCmdId && fsm_vars.replyCb!=NULL) {
         // call the callback
         (this->*fsm_vars.replyCb)(cmdId,&payload[1],length);
         
         // reset
         fsm_vars.replyCmdId  = 0x00;
         fsm_vars.replyCb     = NULL;
      }
   } else {
      SerialDebugPrint(F("WARNING:  RC="));
      SerialDebugPrintln(rc);
      
      if (cmdId==fsm_vars.replyCmdId) {
         // reset
         fsm_vars.replyCmdId  = 0x00;
         fsm_vars.replyCb     = NULL;
      }
   }
}

void SmartMesh::serial_rx_event(uint8_t* payload,uint8_t length) {
   uint8_t state;
   
   // log
   SerialDebugPrintln(F("INFO:     RX event"));

   // assert length is OK
   if (hdlc_vars.inputBufFill<9) {
      SerialDebugPrintln(F("ERROR:    event notification too short"));
      return;
   }
   
   // read state
   state = payload[4];
   
   switch (state) {
      case MOTE_STATE_IDLE:
         SerialDebugPrintln(F("INFO:     state=IDLE"));
         fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_getMoteStatus);
         break;
      case MOTE_STATE_SEARCHING:
         SerialDebugPrintln(F("INFO:     state=SEARCHING"));
         break;
      case MOTE_STATE_NEGOCIATING:
         SerialDebugPrintln(F("INFO:     state=NEGOCIATING"));
         break;
      case MOTE_STATE_CONNECTED:
         SerialDebugPrintln(F("INFO:     state=CONNECTED"));
         break;
      case MOTE_STATE_OPERATIONAL:
         SerialDebugPrintln(F("INFO:     state=OPERATIONAL"));
         fsm_scheduleIn(CMD_PERIOD,&SmartMesh::fsm_getServiceInfo);
         break;
      default:
         // nothing to do
         break;
   }
}

//=========================== fsm =============================================

void SmartMesh::fsm_scheduleIn(uint16_t delay, fsm_timer_callback cb) {
   fsm_vars.delay       = delay;
   fsm_vars.cb          = cb;
}

void SmartMesh::fsm_waitForReply(uint8_t cmdId, fsm_reply_callback cb) {
   fsm_vars.replyCmdId  = cmdId;
   fsm_vars.replyCb     = cb;
}

//===== getMoteStatus

void SmartMesh::fsm_getMoteStatus(void) {
   uint8_t payload[1];
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm getMoteStatus"));

   // prepare payload
   payload[0] = SERIAL_PARAMID_MOTESTATUS;
   
   // send request
   serial_sendRequest(SERIAL_CMDID_GETPARAMETER,payload,sizeof(payload));
   
   // wait for reply
   fsm_waitForReply(SERIAL_CMDID_GETPARAMETER, &SmartMesh::fsm_getMoteStatus_reply);
}

void SmartMesh::fsm_getMoteStatus_reply(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   uint8_t state;
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm getMoteStatus reply"));

   if (length!=11) {
      SerialDebugPrintln(F("ERROR:    wrong length in fsm_getMoteStatus_reply"));
      return;
   }
   
   state = payload[1];
   
   SerialDebugPrint(F("INFO:     state="));
   SerialDebugPrintln(state);
   
   switch (state) {
      case MOTE_STATE_IDLE:
         fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_openSocket);
         break;
      case MOTE_STATE_OPERATIONAL:
         // the API currently does not allow to find out what the open sockets are
         socketId = DEFAULT_SOCKETID;
         
         // mote already operational, send data
         fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_getServiceInfo);
         break;
      default:
         fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_getMoteStatus);
         break;
      }
}

//===== openSocket

void SmartMesh::fsm_openSocket(void) {
   uint8_t payload[1];
   
   fsm_vars.previousEvent = millis();
   
   SerialDebugPrintln(F("INFO:     fsm openSocket"));

   // prepare payload
   payload[0] = 0;
   
   // send request
   serial_sendRequest(SERIAL_CMDID_OPENSOCKET,payload,sizeof(payload));
   
   // wait for reply
   fsm_waitForReply(SERIAL_CMDID_OPENSOCKET, &SmartMesh::fsm_openSocket_reply);
}

void SmartMesh::fsm_openSocket_reply(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm openSocket reply"));

   if (length!=1) {
      SerialDebugPrintln(F("ERROR:    wrong length in fsm_openSocket_reply"));
      return;
   }
   
   // store the socketID
   socketId = payload[0];
   
   // log
   SerialDebugPrint(F("INFO:     socketId="));
   SerialDebugPrintln(socketId);

   // schedule next event
   fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_bindSocket);
}

//===== bindSocket

void SmartMesh::fsm_bindSocket(void) {
   uint8_t payload[3];
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm bindSocket"));

   // prepare payload
   payload[0] = socketId;                        // socketId
   write16b(&payload[1],srcPort);                // port
   
   // send request
   serial_sendRequest(SERIAL_CMDID_BINDSOCKET,payload,sizeof(payload));
   
   // schedule next event
   fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_join);
}

//===== join

void SmartMesh::fsm_join(void) {
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm join"));

   // send request
   serial_sendRequest(SERIAL_CMDID_JOIN,NULL,0);
}

//===== getServiceInfo

void SmartMesh::fsm_getServiceInfo(void) {
   uint8_t payload[3];
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm getServiceInfo"));

   // prepare payload
   payload[0] = 0xff;             // destAddr (0xfffe==manager)
   payload[1] = 0xfe;
   payload[2] = SERVICE_TYPE_BW;  // type
   
   // send request
   serial_sendRequest(SERIAL_CMDID_GETSERVICEINFO,payload,sizeof(payload));
   
   // wait for reply
   fsm_waitForReply(SERIAL_CMDID_GETSERVICEINFO, &SmartMesh::fsm_getServiceInfo_reply);
}

void SmartMesh::fsm_getServiceInfo_reply(uint8_t cmdId, uint8_t* payload, uint8_t length) {
   uint32_t serviceValue;
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm getServiceInfo reply"));

   if (length!=8) {
      SerialDebugPrintln(F("ERROR:    wrong length in fsm_getServiceInfo_reply"));
      return;
   }
   
   // store the socketID
   serviceValue  = payload[4]<<(8*3) | \
                   payload[5]<<(8*2) | \
                   payload[6]<<(8*1) | \
                   payload[7]<<(8*0);
   
   // log
   SerialDebugPrint(F("INFO:     serviceValue="));
   SerialDebugPrintln(serviceValue);

   // schedule next event
   if (serviceValue<=dataPeriod) {
      fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_sendTo);
   } else {
      fsm_scheduleIn(CMD_PERIOD, &SmartMesh::fsm_requestService);
   }
}

//===== requestService

void SmartMesh::fsm_requestService(void) {
   uint8_t payload[7];
   
   fsm_vars.previousEvent = millis();
   SerialDebugPrintln(F("INFO:     fsm requestService"));

   // prepare payload
   payload[0] = 0xff;                       // destAddr (0xfffe==manager)
   payload[1] = 0xfe;
   payload[2] = SERVICE_TYPE_BW;            // serviceType
   payload[3] = (dataPeriod>>(8*3))&0xff;   // value
   payload[4] = (dataPeriod>>(8*2))&0xff;
   payload[5] = (dataPeriod>>(8*1))&0xff;
   payload[6] = (dataPeriod>>(8*0))&0xff;
   
   // send request
   serial_sendRequest(SERIAL_CMDID_REQUESTSERVICE,payload,sizeof(payload));
}

//===== sendTo
 
void SmartMesh::fsm_sendTo(void) {
   uint8_t payload[SENDTO_HEADER_LENGTH+MAX_PAYLOAD_LENGTH];
   uint8_t lenWritten;
   TIME_T fDataPeriod;
   
   fDataPeriod =  dataPeriod;  
   // prepare payload
   payload[0]  = socketId;                       // socketId
   memcpy(&payload[1],destAddr,IPv6ADDR_LEN);    // destIP
   write16b(&payload[17],destPort);              // destPort
   payload[19] = 0x00;                           // serviceType
   
   payload[20] = 0x00;                           // priority (Low priority)
   //payload[20] = 0x02;                           // priority (High priority)
   
   payload[21] = 0xff;                           // packetId
   payload[22] = 0xff;

#ifdef DEBUG
   SerialDebugPrintln(F("INFO:     fsm sendTo"));  
   SerialDebugPrintln(F("Call data generator"));
#endif
   
   
   dataGenerator(&payload[23],MAX_PAYLOAD_LENGTH,&lenWritten,&fDataPeriod);
        
    // send request
    serial_sendRequest(
      SERIAL_CMDID_SENDTO,                       // cmdId
      payload,                                   // payload
      SENDTO_HEADER_LENGTH+lenWritten            // length
    );
    
    SerialDebugPrintln(F("finished call"));   
    // schedule next transmission
    fsm_vars.previousEvent = millis();
    fsm_scheduleIn(fDataPeriod, &SmartMesh::fsm_sendTo);
}

//=========================== helpers =========================================

void SmartMesh::printByteArray(uint8_t* payload, uint8_t length) {
   uint8_t i;
   
   SerialDebugPrint(" ");
   for (i=0;i<length;i++) {
      SerialDebugPrint(payload[i]);
      if (i<length-1) {
         SerialDebugPrint("-");
      }
   }
   SerialDebugPrint("\n");

}

void SmartMesh::write16b(uint8_t* ptr, uint16_t val) {
   ptr[0] = (val>>8) & 0xff;
   ptr[1] = (val>>0) & 0xff;
}
