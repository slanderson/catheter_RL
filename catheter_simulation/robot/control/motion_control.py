'''
motion_control.py

this file controls the commands for moving the auris robot through the AurisInterface
it references the robot_state to determine the desired position and current position of the motor axes

J. Sganga 8/20/2016 
'''

import numpy as np
import time, sys
from functions.low_pass_filter import filter_data


q_num_full = 10
estimated_cycle_time = 1/100. # inside 100 Hz loop
max_velocity = 25. # mm / s - can go higher, but no need to push it

"""
move_dq goes to the commanded wire position at the velocity such 
that all leader and sheath wires reach the commanded position at the same 
time when move_together is set to True.
When move_together is False (like in move_loop), each motor moves as fast as
possible.
Motor speed is a function of cycle time and max_velocity. cycle time is set to 100Hz.
velocity can be set, roughly in mm/s (conversion from rotations to mm done in 
twincat code and is based on geometry of the tendon caps)
"""

def move_dq(robot, dq = [], velocity = [], move_together = True, wait_for_loop = True, compensate_deadzone = True):
    if not len(dq):
        dq = robot.dq_desired
    if not velocity:
        velocity = robot.velocity
    ready = check_move(dq, robot.time_set_position, wait_for_loop)
    if ready:
        dq = scale_dq(dq, velocity, move_together)
        robot.q_desired += dq 
        robot.dq_desired -= dq
        # handle backlash... mult. relaxed tendons by scalar to reduce deadzone
        q_desired = robot.q_desired.copy()
        # if compensate_deadzone:
        #     q_from_initial = robot.q_desired - robot.q_initial
        #     q_from_initial[q_from_initial < 0] *= 0.1
        #     q_desired[:-2] =  robot.q_initial[:-2] + q_from_initial[:-2] # ignoring insertion
        # move
        robot.robot_com.SetDesiredPosition(*q_desired)
        prepare_next_move(robot)

def check_move(dq, time_set_position, wait_for_loop):
    #checks cycle time and if there's any dq to send
    # wait_for_loop flag will be false in simulation runs, allowing for faster runs
    delta_t = time.perf_counter() - time_set_position
    if dq.dot(dq) > 0 and (not wait_for_loop or delta_t >= estimated_cycle_time):
        return True
    else:
        return False

def scale_dq(dq, velocity, move_together):
    if move_together: # scales everything by the max dq
        max_dq_scale = min(1., velocity * estimated_cycle_time / max(abs(dq))) 
        dq_scale = np.ones(q_num_full) * max_dq_scale
    else: # scales everything individually to go at max speeds
        dq_scale = np.minimum(velocity * estimated_cycle_time * np.ones(q_num_full), abs(dq))
        dq = np.sign(dq)
    return dq * dq_scale
         
  
def prepare_next_move(robot):
    # filters q_desired, updates dx_expected, resets timer
    robot.q_desired_raw = robot.q_desired.copy()
    robot.q_desired = filter_data(robot.q_desired_raw, 
                                  robot.q_filter, 
                                  sensor_freq = robot.sensor_freq, 
                                  filter_order = robot.filter_order).copy()
    dq = robot.q_desired - robot.q_filter[1][-3] # delta of filtered points
    if not robot.model_flag:
        robot.dx_expected += robot.J.dot(robot.shrink_q.dot(dq)) # for rotation update
    robot.time_set_position = time.perf_counter()

def double_button(robot):
    return True if robot.xbox.read_controller('X') and robot.xbox.read_controller('A') else False

def move_loop(robot, q, cancel_function = double_button, velocity = [], threshold = 1e-9):
    '''
    runs move_dq many times until the q is reached
    used to go to home, for example
    '''
    if not velocity:
        velocity = robot.max_velocity
    time_auris   = time.perf_counter()
    time_sensors = time.perf_counter()
    cancel_flag  = False
    loop_counter = 0
    dq = q - robot.q_desired
    while dq.dot(dq) > threshold and not cancel_function(robot):
        if (time.perf_counter() - time_sensors > 1. / robot.sensor_freq): # 100 Hz, every 10 ms
            loop_counter += 1
            move_dq(robot, 
                    dq, 
                    velocity * min(1., loop_counter / robot.sensor_freq), 
                    move_together = False,
                    compensate_deadzone = False) # each axis goes as fast as possible
            robot.sensorUpdate()
            time_sensors = time.perf_counter()
            dq = q - robot.q_desired
            
        if (time.perf_counter() - time_auris > 1. / robot.twincat_freq):
            robot.aurisUpdate()
            time_auris = time.perf_counter()


def test_moves(robot):
    '''
    simple way to check outputs of the system based on q's
    '''
    print('Testing')
    delay = 0.01
    robot.aurisUpdate()
    robot.sensorUpdate()
    dq = 10.
    dq_list = np.array([
        [dq, 0, -dq, 0],
        [-dq, 0, dq, 0],
        [dq, 0, -dq, 0],
        [-dq, 0, dq, 0],
        [0, dq, 0, -dq],
        [0, -dq, 0, dq],
        [0, dq, 0, -dq],
        [0, -dq, 0, dq],
        # [dq, dq, -dq, -dq],
        # [-dq, -dq, dq, dq],
        # [dq, dq, -dq, -dq],
        # [-dq, -dq, dq, dq],
        # [dq, -dq, -dq, dq],
        # [-dq, dq, dq, -dq],
        # [dq, -dq, -dq, dq],
        [-dq, dq, dq, -dq]])

    for dq_leader in dq_list:
        dq_move = np.zeros(10)
        dq_move[4:8] = dq_leader
        print(dq_leader)
        sys.stdout.flush()
        move_loop(robot, robot.q_initial +  dq_move, velocity = 5)
        robot.spinWait(delay)
        move_loop(robot, robot.q_initial, velocity = 5)
        robot.spinWait(delay)

def test_model_moves(robot):    
    # simple way to check outputs of the system based on q's
    robot.aurisUpdate()
    robot.q_desired = np.zeros(10) + robot.q_initial
    # robot.q_desired[-1] = 20
    # if robot.q_num == 5:
    # robot.q_desired[-1] = 0  
    # robot.q_desired[0] = -5
    # robot.q_desired[1] = 5
    # robot.q_desired[7] = 0.5
    # robot.q_desired[8] = 40

    for i in range(500):
        robot.q_desired[8] += 0.1
        # robot.q_desired[4] -= 0.05
        robot.q_desired[4] -= 5 * np.cos(i / 20)
        # robot.q_desired[5] += 5 * np.sin(i / 30)
        robot.robot_com.SetDesiredPosition(*robot.q_desired)
        robot.aurisUpdate()
        robot.sensorUpdate()
    robot.saveHistory()






    