'''
tension_control.py 

this file performs the tensioning routine for the catheters
first, all four wires will contract until a current threshold is 
reached above the baseline
next, with all wires tensioned, each wired will pull a prescribed 
displacement to create an estimate of the linear stiffness matrix d_tau = K dq

J. Sganga 8/25/16
'''
import sys, time
import numpy as np
from robot.control.motion_control import move_dq, move_loop
from functions.state_estimation import get_full_model_J
import functions.trig as trig

amps_above_baseline = 0.05
amps_tolerance = 0.03
amp_gain = 1e-2 # inital estimate of amps/dq


def run_tensioning(robot, use_guide = False):
    '''
    taking the place of the built in auris tensioner
    moves and checks if there was 
    a time that had a certain number of filtered data
    points above a threshold over the baseline average.

    should make dq a function of current and move all 
    at the same time, move_evenly, adjust dq with 
    real time current readings. Get each one to get into a 
    within a range some offset above each one's baseline

    sets robot.amps_baseline, robot.amps_inital
    '''
    amps_avg, amps_std = get_average_amps(robot, 1)
    robot.amps_baseline = np.maximum(np.zeros(robot.q_num_full), amps_avg)
    # print('6th axis', robot.amps_baseline[6])
    # robot.amps_baseline[6] += 0.02 # this tendon doesn't like to tension....
    tensioning_loop(robot)
    if use_guide and robot.quality_guide < 10:
        print('starting to line up tip with base')
        sys.stdout.flush()
        line_up_with_base(robot)
    print('done tensioning, initial amps: ', robot.amps_initial[robot.q_list])
    
    sys.stdout.flush()


def get_average_amps(robot, wait_for_average_current = 1.):
    i_start = robot.history_index
    robot.spinWait(wait_for_average_current)
    amps_avg, amps_std = robot.get_averaged_array(i_start, robot.history_index)
    return amps_avg, amps_std

def tensioning_loop(robot, 
                    amps_above_baseline = amps_above_baseline, 
                    amps_tolerance = amps_tolerance):
    '''
    based on control_loop's run_loop() function
    '''
    tensioned    = False
    time_auris   = time.perf_counter()
    time_sensors = time.perf_counter()
    q_start      = robot.q_desired.copy()
    dq_max       = 30

    print('Beginning Tensioning Loop')
    sys.stdout.flush()
    while not tensioned:
        # Get position and force updates (history recording here), 100 Hz position reading
        if (time.perf_counter() - time_sensors > 1. / robot.sensor_freq):
            robot.sensorUpdate()
            time_sensors = time.perf_counter()
            move_dq(robot, velocity = 10)
            
        # Get Auris update and change robot.dq_desired, to be implemented by move_dq
        if (time.perf_counter() - time_auris > 1. / robot.twincat_freq):
            robot.aurisUpdate()
            time_auris = time.perf_counter()
            tensioned = find_tension_dq(robot, amps_above_baseline, amps_tolerance)
            if max(abs(robot.q_desired - q_start)) > dq_max:
                print('Wires NOT tensioned!!! ', np.argmax(abs(robot.q_desired - q_start)))  
                break          

    sys.stdout.flush()


def find_tension_dq(robot, 
                    amps_above_baseline = amps_above_baseline, 
                    amps_tolerance = amps_tolerance):
    '''
    uses a simple proportional control loop to move each 
    wire independently (decent assumption when not tensioned)
    allowing for a band of current that's good, so the 
    control looks like this shape:  \_/

    d_amps = amp_gain * dq
    '''
    q_max = 30
    amps_lower_threshold = robot.amps_baseline + amps_above_baseline
    amps_upper_threshold = robot.amps_baseline + amps_above_baseline + amps_tolerance

    tendons = robot.q_list[:-1]
    d_amps_lower = amps_lower_threshold[tendons] - robot.amps[tendons] # positive when too low
    d_amps_upper = amps_upper_threshold[tendons] - robot.amps[tendons] # negative when too high
    # take only the ones that need increasing or decreasing
    d_amps_increase = np.maximum(d_amps_lower, np.zeros(len(tendons))) 
    d_amps_decrease = np.minimum(d_amps_upper, np.zeros(len(tendons))) 

    dq = (1 / amp_gain) * (d_amps_increase + d_amps_decrease)

    robot.dq_desired *= 0
    if dq.dot(dq) < 1e-8: 
        tensioned = True
        robot.amps_initial = robot.amps.copy()
        print('q_initial', robot.q_initial[tendons])
        print(robot.q_desired[tendons])
    else:
        tensioned = False
        robot.dq_desired[tendons] = dq
    return tensioned


def get_robot_stiffness(robot):
    '''
    finds linear relationship between dq and current
    should be 

    might turn into get_initial_jacobian

    d_tau = K dq
    '''
    dq        = 10
    q_start   = robot.q_desired.copy()
    tendons   = robot.q_list[:-1]# ignoring insert
    K         = np.zeros((robot.q_num_full, robot.q_num_full))
    amps_start, amps_std = get_average_amps(robot, 0.5)
    for qi in tendons: 
        q_tension = q_start.copy()
        q_tension[qi] += dq
        move_loop(robot, q_tension, velocity = 10)
        amps, amps_std = get_average_amps(robot, 0.5)
        K[:, qi] = amps / dq
        move_loop(robot, q_start)
    print(K[tendons,:][:,tendons])
    robot.K = K.copy()

        
def line_up_with_base(robot):
    '''
    lining up the azimuth and elevation of base and tip
    '''
    psi = np.array([-1.80,-0.15,  1.45,  3.10])
    J = np.array([np.sin(psi), np.cos(psi)])
    J = np.hstack((J, np.zeros((2,1))))
    # J = get_full_model_J(robot)[3:5,:][:,robot.q_list] # only grabbing rows for alpha and beta (ignoring gamma/roll bc it will get set to 0)
    d_ab = trig.aer_to_abc(robot.x_guide[3:])[:2] - trig.aer_to_abc(robot.x_sensed[3:])[:2]
    print(robot.expand_q.dot(np.linalg.pinv(J).dot(d_ab)))
    time_auris   = time.perf_counter()
    time_sensors = time.perf_counter()
    print('initial offsets: ', d_ab * 180/np.pi, robot.end_loop)
    while max(abs(d_ab)) > 2. * np.pi / 180 and not robot.end_loop:
        # Get position and force updates (history recording here), 100 Hz position reading
        if (time.perf_counter() - time_sensors > 1. / robot.sensor_freq):
            robot.check_xbox_controller()
            robot.sensorUpdate()
            time_sensors = time.perf_counter()
            d_ab = trig.aer_to_abc(robot.x_guide[3:])[:2] - trig.aer_to_abc(robot.x_sensed[3:])[:2]
            robot.dq_desired = robot.expand_q.dot(np.linalg.pinv(J).dot(d_ab))
            tension_update(robot)
            move_dq(robot, velocity = 5)

        # Get Auris update and change robot.dq_desired, to be implemented by move_dq
        if (time.perf_counter() - time_auris > 1. / robot.twincat_freq):
            robot.aurisUpdate()
            time_auris = time.perf_counter()
            sys.stdout.flush()

    print('straighten, d_ab: ', d_ab * 180/np.pi)     
    robot.x_initial = robot.x_sensed.copy()
    # robot.x_initial[5] -= 10 # adding roll degrees...
    robot.x_guide_initial = robot.x_guide.copy()
    sys.stdout.flush()

def tension_update(robot):
    # if any tendon dips below its inital tension value, all
    # wires will pull evenly to correct for it
    # will also release tension evenly if min is far enough above initial tension
    dq_scale      = 0. 
    tendons = robot.q_list[:-1]

    for qi in tendons:
        if robot.dq_desired[qi] > 0 and robot.amps[qi] < robot.amps_initial[qi]:
            robot.dq_desired[qi] += (1 / amp_gain) * (robot.amps_initial[qi] - robot.amps[qi])

    # amps_lower_threshold = robot.amps_initial - amps_tolerance
    # amps_upper_threshold = robot.amps_initial + amps_tolerance

    # d_amps_lower = amps_lower_threshold[tendons] - robot.amps[tendons] # positive when too low
    # d_amps_upper = amps_upper_threshold[tendons] - robot.amps[tendons] # negative when too high
    
    # if max(d_amps_lower) > 0: # min tension below it's inital value
    #     # print('low tension, ', max(d_amps_lower))
    #     dq_scale = (1 / amp_gain) * max(d_amps_lower) # tension eveyone 
    # elif max(d_amps_upper) < 0: # min tenison above the upper threshold
    #     # print('high tension')
    #     dq_scale = (1 / amp_gain) * max(d_amps_upper) # relax everyone

    # d_amps_increase = np.maximum(d_amps_lower, np.zeros(len(tendons))) 
    # d_amps_decrease = np.zeros(len(tendons))
    # if max(d_amps_lower) < 0:
    #     d_amps_decrease = np.minimum(d_amps_upper, np.zeros(len(tendons))) 
    # dq = (1 / amp_gain) * (d_amps_increase + d_amps_decrease)

    # # robot.dq_desired *= 0
    # if dq.dot(dq) < 1e-8: 
    #     tensioned = True
    #     # robot.amps_initial = robot.amps.copy()
    #     # print('q_initial', robot.q_initial[tendons])
    #     # print(robot.q_desired[tendons])
    # else:
    #     tensioned = False
    #     print('add tension ', dq)
    #     # robot.dq_desired[tendons] += dq
    #     robot.dq_desired[tendons] += dq_scale * np.ones(len(tendons))
    #     move_dq(robot)
    # return tensioned


