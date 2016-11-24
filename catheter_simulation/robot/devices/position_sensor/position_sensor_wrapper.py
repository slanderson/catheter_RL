'''
This wrapper is called by the robot_state.py to give the position and orientation 
of the tip and guide sensor positions. 

Also gives the quality of these sensor readings.

In the case of a real robot run, it references the global variables being written in 
the position sensor thread. 

In the case of a simulated run, it references the ModelInterface to give the determinitic 
pose. Guide sensor will always be zeros.

J. Sganga 8/22/2016
'''

import time, sys
import numpy as np

x_num_full = 6 # xyzaer

class position_sensor(object):
    """class allows for the easy reading of a real pose sensor or model based simulation"""
    def __init__(self, global_variables, robot_com):
        super(position_sensor, self).__init__()
        self.global_variables = global_variables
        self.robot_com        = robot_com
        if global_variables:
            self.x_raw_global = global_variables[7]

    def read_pose_sensor(self):
        if self.global_variables:
            self.x_raw_global[-1] = 1 # letting that thread know it's being read
            return np.asarray(self.x_raw_global[:x_num_full]), self.x_raw_global[x_num_full]
        else:
            pose = self.robot_com.GetCoordinatePosition()
            pose[3:] *= 180 / np.pi # mimics ascension in degrees
            return pose, 0

    def read_second_sensor(self):
        if self.global_variables:
            return (np.asarray(self.x_raw_global[x_num_full + 1:2 * x_num_full + 1]), 
                    self.x_raw_global[2 * x_num_full + 1])
        else:
            return np.zeros(x_num_full), 0

