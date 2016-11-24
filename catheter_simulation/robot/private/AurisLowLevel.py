"""
Auris Surgical Robotics, Inc.
(C) Copyright 2015. All Rights Reserved.

Description: Library for setting up python for use with developmental systems

Author: Ben Fredrickson
Date: Jan 2015
"""

import sys
import os
import time
from winreg import *

#load the path for 
reg = OpenKey(HKEY_LOCAL_MACHINE, "SOFTWARE\\Python\\PythonCore\\%s\\" % (sys.version[:3]))
aurispath = QueryValue(reg, "AurisPath")
sys.path.append(aurispath)


# Add auris dlls to the path for importing
os.environ["PATH"] += ";" + aurispath + "..\\DLLS;"

import AurisInterface

LOW_LEVEL_VERSION = 8

g_btnMap = {
        'DUP': 0,
        'DDWN': 1,
        'DLEFT': 2,
        'DRIGHT': 3,
        'START': 4,
        'BACK': 5,
        'STICKPRESSL': 6,
        'STICKPRESSR': 7,
        'LEFTB': 8,
        'RIGHTB': 9,
        'A': 10,
        'B': 11,
        'X': 12,
        'Y': 13,
        }

g_jntMap = {
        'LX': 0,
        'TRIGL': 1,
        'LY': 2,
        'RX': 3,
        'RY': 4,
        'TRIGR': 5,
        }

class PyAurisInterface(object):
    def __init__(self):
        # make sure the interface is initailized for the correct use case
        self.interInit = False
        self.idmInit = False
        self.visionInit = False
        self.gamepadInit = False
        self.slideInit = False

        # need a ref to AurisInterface for the garbage collector
        self.interface = AurisInterface
        if (AurisInterface.GetVersion() != LOW_LEVEL_VERSION):
            print("Python Lib does not match compiled Lib",
                    AurisInterface.GetVersion(),
                    "!= auris low level version",
                    LOW_LEVEL_VERSION)
            raise Exception("Version Mismatch.")


    def __del__(self):
        """ Closes down network and does any other work that needs to be done. """
        AurisInterface.CloseNetwork()


    def CheckIDM(self):
        if (not self.interInit or not self.idmInit):
            print("Attempting to use IDM before it is initialized.")
            raise Exception("IDM Initialization")


    def CheckVision(self):
        if (not self.interInit or not self.visionInit):
            print("Attempting to use Vision before it is initialized.")
            raise Exception("Vision Initialization")


    def CheckGamePad(self):
        if (not self.gamepadInit):
            print("Attempting to use Game Pad before it is initialized.")
            raise Exception("Game Pad Initialization")

    def CheckLinearSlide(self):
        if (not self.slideInit):
            print("Attempting to use Linear Slide before it is initialized.")
            raise Exception("Linear Slide Initialization")


    def Initialize(self, domainId, testIPAddr = None):
        print("Initializing Network ...")
        if (testIPAddr != None):
            print("Pinging ip (", testIPAddr, ") (must be UID 0) ...")
            resp = os.system("ping -c 1 " + testIPAddr)
            if (resp != 0):
                print("Failed to ping IDM.")
        if (AurisInterface.GetVersion() != LOW_LEVEL_VERSION):
            print("Python Lib does not match compiled Lib",
                    AurisInterface.GetVersion(),
                    "!= auris low level version ",
                    LOW_LEVEL_VERSION)
            return -1
        if (AurisInterface.InitializeNetwork(domainId) != 0):
            print("Failed to connect to RTI")
            return -1
        else:
            print("Network Configured.")
            self.visionInit = True
            self.interInit = True
            sys.stdout.flush()
            return 0


    def EstablishIDMConnection(self, wait = 10.0):
        if (AurisInterface.EstablishIDMConnection(10.0) != 0):
            print("Failed to establish connection")
            return -1
        else:
            print("Connection Established.")
            self.idmInit = True
            return 0


    def InitializeGamePad(self):
        """ Start the joystick code running. """
        err = AurisInterface.InitializeGamePad()
        if err == 0:
            self.gamepadInit = True
        sys.stdout.flush()
        return self.gamepadInit

    def InitializeLinearSlide(self):
        """ Start the linear slide code running. """
        err = AurisInterface.InitializeLinearSlide()
        if err == 0:
            self.slideInit = True
        sys.stdout.flush()
        return self.slideInit
        

    def LoadLeader(self, instrument):
        """ Loads a leader instrument. """
        self.CheckIDM()
        return AurisInterface.LoadLeader(instrument)


    def LoadSheath(self, instrument):
        """ Loads a sheath instrument. """
        self.CheckIDM()
        return AurisInterface.LoadSheath(instrument)
  
  
    def GetMotorCurrents(self):
        """ Returns the actual currents being applied to each axis. """
        self.CheckIDM()
        return AurisInterface.GetMotorCurrents()

  
    def GetForceSensors(self):
        """ Returns the value of the force sensors on each axis. """
        self.CheckIDM()
        return AurisInterface.GetForceSensors()

  
    def GetDesiredPosition(self):
        """ Returns the position we have set on each axis. """
        self.CheckIDM()
        return AurisInterface.GetDesiredPosition()

  
    def SetDesiredPosition(self,
            s1, s2, s3, s4, l1, l2, l3, l4, i1, i2):
        """ Returns the position we have set on each axis. 
        NOTE THE ADDED NEGATIVE!!!"""
        self.CheckIDM()
        return AurisInterface.SetDesiredPosition(
                s1, s2, s3, s4, l1, l2, l3, l4, -i1, i2)

  
    def GetActualPosition(self):
        """ Returns the position of each axis. """
        self.CheckIDM()
        return AurisInterface.GetActualPosition()

  
    def SendIDMCommand(self, commandId):
        """ Send a command (integer) to the IDM. """
        self.CheckIDM()
        return AurisInterface.SendIDMCommand(commandId)


    def GetIDMStatus(self):
        """ Returns the current IDM status (int). """
        self.CheckIDM()
        return AurisInterface.GetIDMStatus()


    def GetLeaderTensionStatus(self):
        """ Returns the tensioning status of the leader """
        self.CheckIDM()
        return AurisInterface.GetTensionStatus()[1]


    def GetSheathTensionStatus(self):
        """ Returns the tensioning status of the leader """
        self.CheckIDM()
        return AurisInterface.GetTensionStatus()[0]


    def GetVisionResult(self):
        """ Returns the last vision result from the ClinicalUI. """
        self.CheckVision()
        return AurisInterface.GetVisionResult()


    def GetGamePadButton(self, index):
        """ Return the value of the button at the given index. """
        self.CheckGamePad()
        return AurisInterface.GetGamePadButton(index)


    def GetGamePadJoint(self, index):
        """ Return the value of the joint at the given index. """
        self.CheckGamePad()
        return AurisInterface.GetGamePadJoint(index)

    def LinearSlideHome(self):
        """ Reset and goto the zero on the linear slide. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideHome()

    def LinearSlideMove(self, Xmm):
        """ Set the x position in milimeters of the slide. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideMove(Xmm)
    
    def LinearSlideMotorRunning(self):
        """ Returns true when the motor is on. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideMotorRunning()

    def LinearSlideWaitForStop(self, timeoutSec):
        """ Blocks until the motor stops moveing or timeout is reached. Returns -1 if
            timedout and zero otherwise."""
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideWaitForStop(timeoutSec)

    def LinearSlidePosition(self):
        """ Get the current x position in millimeters. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlidePosition()

    def LinearSlideMinLimit(self):
        """ Return the lowest allowable position in millimeters. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideMinLimit()
    
    def LinearSlideMaxLimit(self):
        """ Return the highest allowable position in millimeters. """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideMaxLimit()

    def LinearSlideEnableButtonMode(self, run):
        """ Toggle automatic listening to joystick (must init joystick). """
        self.CheckLinearSlide()
        return AurisInterface.LinearSlideEnableButtonMode(run)


# g_PyAuris = PyAurisInterface() 
