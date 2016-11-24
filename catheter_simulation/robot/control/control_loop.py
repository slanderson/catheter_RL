'''
This file contains the high level loop for running the robot 
it calls the appropriate control functions and passes the robot_state handle

Jake Sganga
8/20/2016
'''

import sys
import time
import numpy as np
from robot.control.task_space_control import set_dx
from robot.control.optimal_control import find_dq 
from robot.control.motion_control import move_dq, move_loop, test_moves
from robot.control.tension_control import run_tensioning, tension_update
from functions.state_estimation import jacobian_update


def run_loop(robot):
    time_auris      = time.perf_counter()
    time_sensors    = time.perf_counter()
    print('Beginning Loop')
    sys.stdout.flush()
    # Starting loop and checking buttons that switch modes
    while not robot.end_loop:
        time_loop_start = time.perf_counter()
        # Get position and force updates (history recording here), 100 Hz position reading
        if (time.perf_counter() - time_sensors > 1. / robot.sensor_freq):
            # Make moves based on joystick or trajectory motions 
            # on a 10 Hz freq too, placing before sensor update
            # because time to read sensors (ascension) are variable
            robot.check_xbox_controller()
            robot.sensorUpdate()
            time_sensors = time.perf_counter()
            # motion update
            set_dx(robot)
            find_dq(robot)
            move_dq(robot)
            
            
        # Get Auris update and new Jacobian
        if (time.perf_counter() - time_auris > 1. / robot.twincat_freq):
            robot.aurisUpdate()
            time_auris = time.perf_counter()
            jacobian_update(robot)
            
            
        # if (time.perf_counter() - time_loop_start)*1000 > 1:
        #     print((time.perf_counter() - time_loop_start)*1000)
    sys.stdout.flush()
    robot.saveHistory()
    # let tendons go slack (where they started when cath was loaded)
    move_loop(robot, robot.q_slack)
    print('End Loop :)')

def test_loop(robot):
    print('Beginning Test Loop')
    sys.stdout.flush()
    test_moves(robot)
    robot.saveHistory()
    print('End Test Loop :)')
    
def run_model_loop(robot):
    '''
    mimicing frequency disparaty in run_loop, without actually waiting 
    performs the same functions as run_loop
    robot.end_loop flag gets thrown by the trajectory 
    '''
    print('Beginning Loop')
    sys.stdout.flush()
    time_start = time.perf_counter()
    freq_ratio = int(robot.sensor_freq / robot.twincat_freq)
    freq_counter = 0
    loop_counter = 0
    while not robot.end_trajectory and loop_counter < 3e3:
        freq_counter += 1
        loop_counter += 1
        # Get position and force updates (history recording here), 100 Hz position reading
        robot.sensorUpdate()
        set_dx(robot)
        find_dq(robot)
        move_dq(robot, wait_for_loop = False)
        if freq_counter == freq_ratio:
            robot.aurisUpdate()
            jacobian_update(robot)
            freq_counter = 0
    robot.saveHistory()
    print('End Loop :), time elapsed: ', time.perf_counter() - time_start)







