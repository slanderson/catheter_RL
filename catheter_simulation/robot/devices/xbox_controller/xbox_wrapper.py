'''
Wrapper for xbox controller used with the Auris robot
communicates throught the AurisInterface and AurisLowLevel
exposed to IDM 

J.Sganga 8/22/2016
'''
import sys, time
import numpy as np

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


class xbox(object):
    """class allows for common calls, needs handle to robot_com (AurisLowLevel) """
    def __init__(self, robot_com):
        super(xbox, self).__init__()
        self.robot_com = robot_com
        robot_com.InitializeGamePad()

    def read_controller(self, location):
        """ Return the value of the button or joint, specified by the string """
        if location in g_jntMap:
            return self.robot_com.GetGamePadJoint(g_jntMap[location])
        else:
            return self.robot_com.GetGamePadButton(g_btnMap[location])

    def read_joystick(self):
        joint_threshold = 0.2
        # joints range from [-1, 1]
        ly = self.read_controller("LY")       # Left vert
        lx = self.read_controller("LX")       # Left horiz
        rx = self.read_controller("RX")       # Right horiz
        ry = self.read_controller("RY")       # Right vert
        tr = self.read_controller("TRIGR")    # right trigger
        tl = self.read_controller("TRIGL")    # left trigger
        t = tl - tr
        joyLY = ly if(abs(ly) > joint_threshold) else 0
        joyLX = lx if(abs(lx) > joint_threshold) else 0
        joyRY = ry if(abs(ry) > joint_threshold) else 0
        joyRX = rx if(abs(rx) > joint_threshold) else 0
        joyT  = t  if(abs(t)  > joint_threshold) else 0
        return np.array([joyLY, joyLX, joyRY, joyRX, joyT], dtype = float)