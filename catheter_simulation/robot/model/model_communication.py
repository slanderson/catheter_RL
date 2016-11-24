'''
 Author: Jake Sganga
Date: 10/21/15
Edit: 8/22/2016 (file name, location)

The model_communication script should be imported in place of the AurisLowLevel.py  
This will allow for the most seamless implementation of model-tested algorithms on the robot itself. 
For that reason, the classes and functions should match the names and expected behavior of AurisLowLevel.

'''
import sys
import time
import numpy as np 
from robot.model.model_kinematics import catheter 

class RobotModelInterface(object):
    def __init__(self, 
                 use_leader       = True, 
                 use_sheath       = False,
                 sensor_noise     = [0,0,0], 
                 use_obstacle     = False,
                 use_heartbeat    = False,
                 use_lung_task    = False,
                 bend_constraint  = np.pi,
                 touch_wall       = False):
        # make sure the interface is initailized for the correct use case
        self.interInit = True
        self.idmInit = True
        self.visionInit = True
        self.gamepadInit = True
        self.slideInit = True
        self.robot = catheter(use_leader       = use_leader, 
                              use_sheath       = use_sheath,
                              sensor_noise     = sensor_noise, 
                              use_obstacle     = use_obstacle,
                              use_heartbeat    = use_heartbeat,
                              use_lung_task    = use_lung_task,
                              bend_constraint  = bend_constraint,
                              touch_wall       = touch_wall)

    def __del__(self):
        pass

    def CheckIDM(self):
        pass

    def CheckVision(self):
        pass

    def CheckGamePad(self):
        pass

    def CheckLinearSlide(self):
        pass

    def Initialize(self, domainId, testIPAddr = None):
        print("Initializing Model ...")
        return 0

    def EstablishIDMConnection(self, wait = 10.0):
        return 0

    def InitializeGamePad(self):
        # Allows game pad to control model - need AurisInterface to talk to xbox controller
        if USE_XBOX:
            err = AurisInterface.InitializeGamePad()
            if err == 0:
                self.gamepadInit = True
            sys.stdout.flush()
            return self.gamepadInit
        else:
            pass

    def InitializeLinearSlide(self):
        return True

    def LoadLeader(self, instrument):
        return 0 # not sure if this

    def LoadSheath(self, instrument):
        return 0
    
    def GetMotorCurrents(self):
        """ Returns the actual currents being applied to each axis. """
        return self.robot.getTensions()
  
    def GetForceSensors(self):
        """ Returns the value of the force sensors on each axis. """
        return 0

    def GetDesiredPosition(self):
        """ Returns the position we have set on each axis. """
        return self.robot.getDesiredPosition()
  
    def SetDesiredPosition(self,
            s1, s2, s3, s4, l1, l2, l3, l4, i1, i2):
        """ Sets the position we have set on each axis. 
        NOTICE THE NEGATIVE SIGN, to match real robot inversion"""
        q = np.asarray([s1, s2, s3, s4, l1, l2, l3, l4, i1, i2])
        return self.robot.setDesiredPosition(q)
  
    def GetActualPosition(self):
        """ Returns the position of each axis. """
        return self.robot.getActualPosition()
  
    def SendIDMCommand(self, commandId):
        """ Send a command (integer) to the IDM. """
        return 0

    def GetIDMStatus(self):
        """ Returns the current IDM status (int). """
        return 0

    def GetLeaderTensionStatus(self):
        """ Returns the tensioning status of the leader """
        return 0

    def GetSheathTensionStatus(self):
        """ Returns the tensioning status of the leader """
        return 0

    def GetVisionResult(self):
        """ Returns the last vision result from the ClinicalUI. """
        return 0

    def GetGamePadButton(self, index):
        """ Return the value of the button at the given index. """
        pass

    def GetGamePadJoint(self, index):
        """ Return the value of the joint at the given index. """
        pass

    ##########################################################################################################
    # added for the model

    def GetCoordinatePosition(self):
        ''' returns position and orientation'''
        return self.robot.getCoordinatePosition()

    def Reset(self):
        ''' Reinitializes the kinematic model'''
        del self.robot
        self.robot = ModelInterface.catheter(with_sheath = True, with_4tendons = True)

    def updateForcesAndTorques(self):
        '''returns forces and torques'''
        return self.robot.getForces()

    def getStiffness(self):
        return self.robot.getStiffness()

    def estimateModelJacobian(self, q):
        return self.robot.estimateModelJacobian(q)

    def getTensions(self):
        return self.robot.getTensions()

    def getModelRotation(self):
        return self.robot.getModelRotation()

    def getModelPose(self):
        return self.robot.getModelPose()

    def getPositionFromCurve(self, curve):
        return self.robot.getPositionFromCurve(curve)

    def get_leader_jacobian(self, q, curve = []):
        return self.robot.get_leader_jacobian(q, curve)

    def get_leader_curvature(self, q_tendons):
        return self.robot.get_leader_curvature(q_tendons)
