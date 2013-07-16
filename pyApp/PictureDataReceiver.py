#!/usr/bin/python

# add the SmartMeshSDK/ folder to the path
import sys
import os
import time;

temp_path = "C:\mmoreno\SmartMeshSDK-full-1.0.3.73\SmartMeshSDK-1.0.3.73\src\SmartMeshSDK-1.0.3.73"
if temp_path:
    sys.path.insert(0, os.path.join(temp_path, 'dustUI'))
    sys.path.insert(0, os.path.join(temp_path, 'SmartMeshSDK'))
    sys.path.insert(0, os.path.join('GUI'))

# verify installation
import SmsdkInstallVerifier
(goodToGo,reason) = SmsdkInstallVerifier.verifyComponents(
                            [
                                SmsdkInstallVerifier.PYTHON,
                                SmsdkInstallVerifier.PYSERIAL,
                            ]
                        )
if not goodToGo:
    print "Your installation does not allow this application to run:\n"
    print reason
    raw_input("Press any button to exit")
    sys.exit(1)

#Libs for refresh Image
import Image 
import ImageTk
from cStringIO import StringIO

#GUI libs
import Tkinter
import threading

from dustWindow           import dustWindow
from dustFrameConnection  import dustFrameConnection
from dustFrameImage  import dustFrameImage
from dustFrameText  import dustFrameText

#DUST API
from ApiDefinition  import IpMgrDefinition
from ApiException            import APIError

from IpMgrConnectorMux  import IpMgrConnectorMux
from IpMgrConnectorMux  import IpMgrSubscribe

from IpMgrConnectorSerial  import IpMgrConnectorSerial
from IpMgrConnectorSerial import IpMgrConnectorSerialInternal

from optparse import OptionParser

UPDATEPERIOD = 250 # in ms
DEFAULT_HOST = '127.0.0.3'
DEFAULT_PORT = 9900
DEFAULT_SPORT = 'COM26'

ARDUINO_COMMANDS = {'Capture':[0xc1],'StartAuto':[0xCA],'StopAuto':[0xC0]}

STADISTICS_MSG = "Downloaded files # {0} \nLast image info:\n\tRx pkts# {1} \n\tRx data # {2} \n\tDownload time: {3} ms"
class notifClient(object):
    '''
    \ingroup MgrListener
    '''
    
    def __init__(self, connector, disconnectedCallback, showDataCallBack):
        
        # store params
        self.connector = connector
        self.disconnectedCallback = disconnectedCallback
        self._showDataCB = showDataCallBack
        
        # variables
        self.data      = None
        self.dataLock  = threading.Lock()
        self.currentImg= ""
        self.lastImg= ""
        self.numDownloadedFiles = 0
        self.numfiles = 1
        self.numPkts = 0
        self.timeS = [0,0]
        
        # subscriber
        self.subscriber = IpMgrSubscribe.IpMgrSubscribe(self.connector)
        self.subscriber.start()
        self.subscriber.subscribe(
            notifTypes =    [
                                IpMgrSubscribe.IpMgrSubscribe.NOTIFDATA,
                            ],
            fun =           self._notifDataCallback,
            isRlbl =        False,
        )
        self.subscriber.subscribe(
            notifTypes =    [
                                IpMgrSubscribe.IpMgrSubscribe.ERROR,
                                IpMgrSubscribe.IpMgrSubscribe.FINISH,
                            ],
            fun =           self.disconnectedCallback,
            isRlbl =        True,
        )
    
    #======================== public ==========================================
    
    def disconnect(self):
        self.connector.disconnect()
    
    def resetData(self):
        self.currentImg= ""
        self.numfiles = 1
        self.numPkts = 0
        self.timeS = [0,0]

    def saveImg(self):
        filename = "photos/picture{0}.jpg".format(self.numfiles)
        f = open(filename, 'wb')
        f.write(self.lastImg)
        f.close()

        if self.numfiles == 10:
            self.numfiles = 1
        else:
            self.numfiles = self.numfiles + 1
            
        #filenameImage= "photos/picture"+str(self.numfiles-1)+".h"
        #f = open(filenameImage, 'w')
        #f.write("uint8_t image[]={0x")
        #f.write(",0x".join("{:02x}".format(ord(c)) for c in self.currentImg))
        #f.write("};\n")
        #f.close()
    #======================== private =========================================
    def _showDataCB(self,imgJpg,textS):
            print "Show Data"

    def _dataProcessor(self,notifParams):
        
        if len(self.currentImg)>0:

            timeR = [notifParams.utcSecs,notifParams.utcUsecs]
             
            if self.timeS != [0,0]:
                diffSec = (timeR[0] - self.timeS[0])*1000
                diffUSec = (timeR[1] - self.timeS[1])/1000
                downloadTime = diffSec-diffUSec
            else:
                downloadTime = 0
            
            self.numDownloadedFiles = self.numDownloadedFiles + 1    
            
            textS = STADISTICS_MSG.format(self.numDownloadedFiles,
                                          self.numPkts,
                                          len(self.currentImg),
                                          downloadTime)
                                                
            self._showDataCB(self.currentImg,textS)

            self.lastImg = self.currentImg
            self.currentImg = ""
            self.numPkts = 0
        
        self.numPkts = self.numPkts + 1
        
        self.timeS = [notifParams.utcSecs,notifParams.utcUsecs]
            
                           
    def _notifDataCallback(self,notifName,notifParams):
        
        self.dataLock.acquire()
        self.data              = {}
        self.data['srcMac']    = notifParams.macAddress
        self.data['srcPort']   = notifParams.srcPort
        self.data['destPort']  = notifParams.dstPort
        self.data['payload']   = notifParams.data
        self.data['ts_sec']    = notifParams.utcSecs
        self.data['ts_usec']   = notifParams.utcUsecs
        self.dataLock.release()
        sData = "".join(map(chr,notifParams.data))

        if sData.find("picture")>=0:
            self._dataProcessor(notifParams)
        else:
            self.currentImg = self.currentImg + sData
            print "Data size: "+str(len(sData))+" jpgContentSize:"+str(len(self.currentImg))
            self.numPkts = self.numPkts + 1
            if len(sData)<71:
                self._dataProcessor(notifParams)
           
class dataGui(object):
    '''
    \ingroup MgrListener
    '''
   
    def __init__(self):
        
        # local variables
        self.guiLock            = threading.Lock()
        self.apiDef             = IpMgrDefinition.IpMgrDefinition()
        self.notifClientHandler = None
        
        # create window
        self.window = dustWindow('PictureReceiver',
                                 self._windowCb_close)
                                 
        # add a sensor data frame row=0 col=0
        self.connectionFrame = dustFrameConnection(
                                    self.window,
                                    self.guiLock,
                                    self._connectionFrameCb_connected,
                                    frameName="Serial connection",
                                    row=0,column=0)
        self.connectionFrame.apiLoaded(self.apiDef)
        self.connectionFrame.show()
        
        # add a sensor data frame row=0 col=1
        self.sensorDataFrame = dustFrameImage(
                                self.window,
                                self.guiLock,
                                self._captureButton,
                                self._automaticCaptureButton,
                                self._saveButton,
                                frameName="Picture from camera",
                                row=0,column=1)
        self.isAutomatic = False                                      
        self.sensorDataFrame.show()
        
        # add text info frame row=1 col=0
        self.textFrame = dustFrameText(
                                self.window,
                                self.guiLock,
                                frameName="Stadistics",
                                row=1,column=0)
        self.textFrame.show()
        self.textFrame.write(STADISTICS_MSG.format(0,0,0,0))
        
        self.sensorDataFrame.captureButton.configure(state=Tkinter.DISABLED)
        self.sensorDataFrame.startStopAutoButton.configure(state=Tkinter.DISABLED)
    #======================== public ==========================================
    
    def start(self, connect_params):
        
        # start Tkinter's main thead
        try:
            self.window.mainloop()
        except SystemExit:
            sys.exit()

    #======================== private =========================================
    def _getOperationalMotesMacAddresses(self):
        returnVal = []
           
        currentMac     = (0,0,0,0,0,0,0,0) # start getMoteConfig() iteration with the 0 MAC address
        continueAsking = True
        while continueAsking:
            try:
                res = self.connector.dn_getMoteConfig(currentMac,True)
            except APIError:
                continueAsking = False
            else:
                if ((not res.isAP) and (res.state in [4,])):
                    returnVal.append(tuple(res.macAddress))
                currentMac = res.macAddress
        # order by increasing MAC address
        returnVal.sort()    
        
        return returnVal
        
    def  _sendCmdArduino(self,cmdArduino):
        macList = self._getOperationalMotesMacAddresses()
        mac= macList[0]
        priority =  2
        srcPort =  61000
        dstPort = 60000
        options = 0

        try:
            self.connector.dn_sendData( mac,
                                    priority,
                                    srcPort,
                                    dstPort,
                                    options,
                                    cmdArduino)
        except APIError as e:
            print "Exception -> {0}".format(e)
        
        
    def _captureButton(self):
        self.guiLock.acquire()
        self._sendCmdArduino(ARDUINO_COMMANDS['Capture'])
        self.guiLock.release()

    def _automaticCaptureButton(self):
        if self.isAutomatic:
            self.guiLock.acquire()
            self._sendCmdArduino(ARDUINO_COMMANDS['StopAuto'])
            print "Stop automatic capturing ..."
            self.sensorDataFrame.startStopAutoButton.configure(text='Start Auto')
            self.isAutomatic = False
            self.guiLock.release()    
            
        else:
            self.guiLock.acquire()
            self._sendCmdArduino(ARDUINO_COMMANDS['StartAuto'])
            print "Start automatic capturing ..."
            self.isAutomatic = True
            self.sensorDataFrame.startStopAutoButton.configure(text='Stop Auto')
            self.guiLock.release()

    def _saveButton(self):
        if self.notifClientHandler:
            self.notifClientHandler.saveImg()
                        
    def _windowCb_close(self):
        if self.notifClientHandler:
            self.notifClientHandler.disconnect()
    
    def _connectionFrameCb_connected(self,connector):
        '''
        \brief Called when the connectionFrame has connected.
        '''
        
        # store the connector
        self.connector = connector
        
        # start a notification client
        self.notifClientHandler = notifClient(
                    self.connector,
                    self._connectionFrameCb_disconnected,
                    self._showData
                )
        
        self.sensorDataFrame.captureButton.configure(state=Tkinter.NORMAL)
        self.sensorDataFrame.startStopAutoButton.configure(state=Tkinter.NORMAL)
        
    def _connectionFrameCb_disconnected(self,notifName,notifParams):
        '''
        \brief Called when the connectionFrame has disconnected.
        '''
        
        # update the GUI
        self.connectionFrame.updateGuiDisconnected()
        
        # delete the connector
        self.connector = None
        
        self.notifClientHandler.resetData()
        
        self.sensorDataFrame.captureButton.configure(state=Tkinter.DISABLED)
        self.sensorDataFrame.startStopAutoButton.configure(state=Tkinter.DISABLED)

    def _showData(self,imgJpg,textS):
        '''
        \brief Called when the connectionFrame has disconnected.
        '''
        
        #refresh image        
        try:
    
            file_jpgdata = StringIO(imgJpg)
                        
            img = Image.open(file_jpgdata)
            
            self.sensorDataFrame.photo = ImageTk.PhotoImage(img)

            self.sensorDataFrame.imgCanvas.itemconfigure(self.sensorDataFrame.item, image=self.sensorDataFrame.photo)
        except:
            print "Canvas error"
            
        #refresh text
        self.textFrame.write(textS)
        
    def _startDataPressedCb(self):
        print " _startPressedCb called"

    def _stopDataPressedCb(self):
        print " _startPressedCb called"
                    
    
def main(connect_params):
    dataGuiHandler = dataGui()
    dataGuiHandler.start(connect_params)

if __name__ == '__main__':
    # Parse the command line
    parser = OptionParser("usage: %prog [options]", version="%prog 1.0")
    parser.add_option("--host", dest="host", 
                      default=DEFAULT_HOST,
                      help="Mux host to connect to")
    parser.add_option("-p", "--port", dest="port", 
                      default=DEFAULT_PORT,
                      help="Mux port to connect to")
    parser.add_option("--sport", dest="sport", 
                      default=DEFAULT_SPORT,
                      help="serial port to connect to")
    (options, args) = parser.parse_args()
    
    connect_params = {'host': options.host,
                      'port': int(options.port),
                      'sport': options.sport}
    main(connect_params)
