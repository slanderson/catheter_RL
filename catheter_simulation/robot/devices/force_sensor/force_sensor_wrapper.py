
import sys
import numpy as np
sys.path.append("C:\\ATI_Files\\ATI_boost\\x64\\Debug")
import ATI_boost 

'''
The function getForces() returns a numpy array of the xyz forces and torques
The read frequency is determined in the boost build (~10000Hz) with a 10 pt average
Based on the ATI Mini40 force sensor

Not sure if I need to stop it somehow...
J.Sganga 3/18/16
'''


class ForceSensor(object):
    """docstring for ForceSensor"""
    def __init__(self):
        super(ForceSensor, self).__init__()
        self.sensor_ID = None
        self.initialize()

    def initialize(self):
        self.sensor_ID = ATI_boost.ATIForceSensorWrapper()
        self.sensor_ID.initializeSensor()

    def readForceSensor(self):
        # Forces shown in N
        # Torques shown in Nm
        # this can be changed in the boost build
        forces = np.zeros(6)
        self.sensor_ID.updateForcesAndTorques()
        forces[0] = self.sensor_ID.getFx()
        forces[1] = self.sensor_ID.getFy()
        forces[2] = self.sensor_ID.getFz()
        forces[3] = self.sensor_ID.getTx()
        forces[4] = self.sensor_ID.getTy()
        forces[5] = self.sensor_ID.getTz()

        R = np.array([ # aligns the axes of the force sensor with the ascension axes
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, -1]])
        forces[:3] = R.dot(forces[:3])
        forces[3:] = R.dot(forces[3:])
        return forces.copy()

    def find_contact_frame(self, forces):
        force_threshold = 0.02
        force_norm = np.sqrt(forces.dot(forces))
        # unit vector in direction of forces
        uF = np.zeros(3) 
        if force_norm > force_threshold:
            uF = forces[:3] / force_norm

        # projection onto force/constrained direction
        # P = np.outer(uF, uF)
        P = np.zeros(3) # leaving out for now...
        # if force_norm > force_threshold: # heuristic 
        #     self.P = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 1]]) # in z-dir
        # unconstrained plane, angles always unconstrained for now
        P_free = np.eye(3) - P
        return P, P_free

force_sensor = ForceSensor()