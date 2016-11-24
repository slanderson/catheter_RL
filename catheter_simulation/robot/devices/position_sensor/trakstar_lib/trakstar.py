'''
This function should be called by IDM.py to initialize  the Ascension to the desired settings,
set the data file format, and return the object pointer to the trakstar system, so within the IDM
script, the current position can be extracted using the call trakstar.get_asynchronous_data_dict()

Jake Sganga
10/27/15
'''

import os
import sys
# import robot.devices.position_sensor.trakstar_interface_3
# from robot.devices.position_sensor.trakstar_interface_3 import TrakSTARInterface
import numpy as np
import time


def settings():
    trakstar_s = None
    # trakstar = TrakSTARInterface()
    # trakstar.initialize()
    # trakstar.set_system_configuration(measurement_rate=120,
                                    # max_range=36, 
                                    # metric=True, 
                                    # power_line=60, 
                                    # report_rate=1, 
                                    # print_configuration=True)
    """
    measurement_rate in Hz: 20.0 < rate < 255.0
    max_range: valid values (in inches): 36.0, 72.0, 144.0
    metric: True (data in mm) or False (data in inches)
    power_line in Hz: 50.0 or 60.0 (frequency of the AC power source)
    report_rate: (int), between 1 and 127 --> report every 2nd, 3rd, etc. value
    """
    return trakstar_s
