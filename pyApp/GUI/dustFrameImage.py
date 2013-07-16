#!/usr/bin/python

import sys
import os
if __name__ == '__main__':
    temp_path = sys.path[0]
    sys.path.insert(0, os.path.join(temp_path, '..', 'SmartMeshSDK'))

import Tkinter
import dustGuiLib
import dustFrame
from dustStyle import dustStyle
from ApiDefinition import ApiDefinition

import Image 
import ImageTk
from Tkinter import Tk, Canvas, Frame, BOTH,NW
from cStringIO import StringIO

class dustFrameImage(dustFrame.dustFrame):
    
    def __init__(self,parentElem,guiLock,inStartCaptureBPCB,inStartCAutoBPCB,inSaveBPCB,frameName="Image Frame",row=0,column=0):
        # record params
        self.startCaptureBPCB = inStartCaptureBPCB
        self.startCAutoBPCB = inStartCAutoBPCB
        self.saveBPCB = inSaveBPCB
        
        # init parent
        dustFrame.dustFrame.__init__(self,parentElem,guiLock,frameName,row,column)   

        self.captureButton = dustGuiLib.Button(self.container,
                                             text='Capture',
                                               command=self.startCaptureBPCB)
        
        self.startStopAutoButton = dustGuiLib.Button(self.container,
                                             text='Start Auto',
                                             command=self.startCAutoBPCB)
        
        self.saveButton = dustGuiLib.Button(self.container,
                                             text='Save image',
                                             command=self.saveBPCB)
        self._add(self.saveButton,0,0)
        self._add(self.captureButton,1,1)
        self._add(self.startStopAutoButton,1,2)
        
        self.isAutomatic = False
        
        # row 1: canvas
        self.imgCanvas = Tkinter.Canvas(self.container,
                                        width=320,
                                        height=240)

        #Simulate In-memory
        f = open("geek_inside.jpg","rb")
        jpgdata = f.read()
        f.close()

        ##Draw from in-memory data
        file_jpgdata = StringIO(jpgdata)
        
        self.img = Image.open(file_jpgdata)
        
        
        self.photo = ImageTk.PhotoImage(self.img)

        self.item=self.imgCanvas.create_image(0,0,anchor=NW,image=self.photo)

        self._add(self.imgCanvas,0,1,columnspan=2)

    #======================== private =========================================

    def _startStopButtonPressed(self):
            self.guiLock.acquire()
            print "Capture img"
            self.guiLock.release()


    def _startStopAutomaticButtonPressed(self):
        if self.isAutomatic:
            self.guiLock.acquire()
            print "Stop automatic capturing ..."
            self.startStopButton.configure(text='Start Auto')
            self.guiLock.release()    
        else:
            self.guiLock.acquire()
            print "Start automatic capturing ..."
            self.isAutomatic = True
            self.startStopButton.configure(text='Stop Auto')
            self.guiLock.release()
            
#============================ sample app ======================================
# The following gets called only if you run this module as a standalone app, by
# double-clicking on this source file. This code is NOT executed when importing
# this module is a larger application
#
class exampleApp(object):
    
    def __init__(self):
        self.window  = dustWindow("dustFrameImagesTest",
                                  self._closeCb)
        self.guiLock            = threading.Lock()
        self.frame = dustFrameImage(self.window,self.guiLock,self._startPressedCb,row=0,column=0)
        self.frame.show()
        self.window.mainloop()
    
    def _startPressedCb(self):
        print " _startPressedCb called"
    
    def _closeCb(self):
        print " _closeCb called"

if __name__ == '__main__':
    import threading
    from dustWindow import dustWindow
    exampleApp()
