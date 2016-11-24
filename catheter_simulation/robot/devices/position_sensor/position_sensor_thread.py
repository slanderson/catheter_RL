# dumps raw data into the global variables. Avoids the issue with random lags on the Ascension calls 
# from holding up the rest of the loop. The filtering still happends in IDM, so another global variable, x_raw_global
# was created for this thread to communicate with IDM. 
import sys
import time
import numpy as np

def PoseThread(global_variables):
    print('Started Ascension Thread \n')
    sys.stdout.flush()
    x  = global_variables[7] #raw positions 
    from robot.devices.position_sensor.position_sensor_interface import PoseSensor
    position_sensor = PoseSensor()
    position_sensor.initialize()
    loop_start = time.perf_counter()
    last_interaction = time.perf_counter()
    while (time.perf_counter() - last_interaction) < 10:
        if (time.perf_counter() - loop_start > 1 / position_sensor.measurement_rate):
            x[:14] = position_sensor.readPoseSensor()[:]
            # work around for killing this thread after a cntrl-C interupt, expect idm to set x[13] to 1 after read
            if x[14]:
                x[14] = 0
                last_interaction = time.perf_counter()


    #close ascension, not sure if necessary
    position_sensor.close()