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

#ifndef SmartMesh_h
#define SmartMesh_h

#include <Arduino.h>
#include "SmartMeshVersion.h"

//=========================== defines =========================================

#define TIME_T                    unsigned long

//#define BAUDRATE_CLI              9600
#define BAUDRATE_DEBUG            115200
#define BAUDRATE_API              115200

#define TRUE                      0x01
#define FALSE                     0x00

//===== HDLC

//this table is used to expedite execution (at the expense of memory usage)
static const uint16_t hdlc_fcstab[256] = {
   0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
   0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
   0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
   0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
   0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
   0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
   0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
   0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
   0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
   0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
   0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
   0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
   0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
   0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
   0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
   0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
   0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
   0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
   0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
   0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
   0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
   0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
   0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
   0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
   0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
   0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
   0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
   0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
   0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
   0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
   0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
   0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

#define HDLC_FLAG                      0x7e
#define HDLC_ESCAPE                    0x7d
#define HDLC_ESCAPE_MASK               0x20
#define HDLC_CRCINIT                   0xffff
#define HDLC_CRCGOOD                   0xf0b8

#define HDLC_MAX_FRAME_LENGTH          128
#define HDLC_INPUT_BUFFER_SIZE         HDLC_MAX_FRAME_LENGTH
#define HDLC_OUTPUT_BUFFER_SIZE        HDLC_MAX_FRAME_LENGTH

//===== serial

#define SERIAL_API_MASK_RESPONSE       0x01
#define SERIAL_API_MASK_PACKETID       0x02
#define SERIAL_API_MASK_SYNC           0x08

#define SERIAL_PACKETID_NOTSET         0x02

// command IDs
#define SERIAL_CMDID_GETPARAMETER      0x02
#define SERIAL_CMDID_JOIN              0x06
#define SERIAL_CMDID_EVENT             0x0f
#define SERIAL_CMDID_REQUESTSERVICE    0x11
#define SERIAL_CMDID_GETSERVICEINFO    0x12
#define SERIAL_CMDID_OPENSOCKET        0x15
#define SERIAL_CMDID_BINDSOCKET        0x17
#define SERIAL_CMDID_SENDTO            0x18
#define SERIAL_CMDID_RECEIVEFROM       0x19

// parameter IDs
#define SERIAL_PARAMID_MOTESTATUS 0x0E

// return code
#define SERIAL_RC_OK                   0x00

// mote state
#define MOTE_STATE_IDLE                0x01
#define MOTE_STATE_SEARCHING           0x02
#define MOTE_STATE_NEGOCIATING         0x03
#define MOTE_STATE_CONNECTED           0x04
#define MOTE_STATE_OPERATIONAL         0x05

// service types
#define SERVICE_TYPE_BW                0x00

#define MAX_PAYLOAD_LENGTH             71
#define SENDTO_HEADER_LENGTH           (1+16+2+1+1+2)
#define IPv6ADDR_LEN                   16
#define DEFAULT_SOCKETID               22

//===== fsm

#define CMD_PERIOD                1000      // number of ms between two commands being sent

// well-known IPv6 address of the SmartMesh IP manager
static const uint8_t ipv6Addr_manager[16] = {
   0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02
};

// IPv6 address of http://motedata.dustnetworks.com/
static const uint8_t ipv6Addr_motedata[16] = {
   0x20,0x01,0x04,0x70,0x00,0x66,0x00,0x17,
   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02
};

//=========================== typedef =========================================

class SmartMesh; // forward declaration needed here

typedef void (SmartMesh::*fsm_timer_callback)(void);
typedef void (SmartMesh::*fsm_reply_callback)(uint8_t cmdId, uint8_t* payload, uint8_t length);
typedef void (*data_generator)(uint8_t* ptr, uint8_t maxLen, uint8_t* lenWritten,TIME_T* fDataPeriod);
typedef void (*data_processor)(uint8_t* ptr, uint8_t lenRead);

//=========================== variables =======================================

//===== HDLC

typedef struct {
   // input
   uint8_t              lastRxByte;
   bool                 busyReceiving;
   bool                 inputEscaping;
   uint16_t             inputCrc;
   uint8_t              inputBufFill;
   uint8_t              inputBuf[HDLC_INPUT_BUFFER_SIZE];
   // output
   bool                 outputBufFilled;
   uint16_t             outputCrc;
   uint8_t              outputBufIdxW;
   uint8_t              outputBufIdxR;
   uint8_t              outputBuf[HDLC_OUTPUT_BUFFER_SIZE];
} hdlc_vars_t;

//===== serial

typedef struct {
   uint8_t              txPacketId;
   uint8_t              rxPacketId;
} serial_vars_t;

//===== fsm

typedef struct {
   uint8_t              state;
   TIME_T               previousEvent;
   TIME_T               delay;
   fsm_timer_callback   cb;
   uint8_t              replyCmdId;
   fsm_reply_callback   replyCb;
} fsm_vars_t;

//=========================== SmartMesh object ================================

class SmartMesh {
   public:
      //===== methods
      SmartMesh();
      void              setup(
         uint16_t       srcPort_param,
         uint8_t*       destAddr_param,
         uint16_t       destPort_param,
         TIME_T         dataPeriod_param,
         data_generator dataGenerator_param,
         data_processor dataProcessor_param
      );
      void              loop();
      //===== attributes
      
      //===== methods
      //=== HDLC
      // receiver
      void              hdlc_receiver(uint8_t rxbyte);
      // output
      void              outputHdlcOpen();
      void              outputHdlcWrite(uint8_t b);
      void              outputHdlcClose();
      // input
      void              inputHdlcOpen();
      void              inputHdlcWrite(uint8_t b);
      void              inputHdlcClose();
      // helpers
      uint16_t          crcIteration(uint16_t crc, uint8_t byte);
      //=== serial
      // TX
      void              serial_sendRequest(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              serial_sendReply(  uint8_t cmdId, uint8_t* payload, uint8_t length);
      // RX
      void              serial_receiver();
      void              serial_dispatch_request(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              serial_dispatch_response(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              serial_rx_event(uint8_t* payload,uint8_t length);
      //=== fsm
      void              fsm_scheduleIn(uint16_t delay, fsm_timer_callback cb);
      void              fsm_waitForReply(uint8_t cmdId, fsm_reply_callback cb);
      void              fsm_getMoteStatus();
      void              fsm_getMoteStatus_reply(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              fsm_openSocket();
      void              fsm_openSocket_reply(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              fsm_bindSocket();
      void              fsm_join();
      void              fsm_getServiceInfo();
      void              fsm_getServiceInfo_reply(uint8_t cmdId, uint8_t* payload, uint8_t length);
      void              fsm_requestService();
      void              fsm_sendTo();
      //=== helpers
      void              printByteArray(uint8_t* payload, uint8_t length);
      void              write16b(uint8_t* ptr, uint16_t val);
   private:
      //===== attributes
      uint8_t           socketId;                // ID of the mote's UDP socket
      uint16_t          srcPort;                 // UDP source port
      uint8_t           destAddr[IPv6ADDR_LEN];  // IPv6 destination address
      uint16_t          destPort;                // UDP destination port
      TIME_T            dataPeriod;              // number of ms between transmissions
      data_generator    dataGenerator;           // data generating function
      data_processor    dataProcessor;           // data processor function
      hdlc_vars_t       hdlc_vars;               // HDLC-specific variables
      serial_vars_t     serial_vars;             // serial-specific variables
      fsm_vars_t        fsm_vars;                // fsm-specific variables
};

#endif SmartMesh_h
