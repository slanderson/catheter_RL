'''
This file will contain the high level loops for running the catheter robot model
See main.py for levels of abstraction

J Sganga
3/19/16
'''

import sys
import time
import numpy as np
import pickle

from robot_model import IDM_model
from robot_model.functions.trajectories import getTrajectory
from robot_model.private.ModelLowLevel import RobotModelInterface


def controlModel(use_leader       = True,
                 use_sheath       = False,
                 use_orientation  = True,
                 sensor_noise     = [0,0,0], # standard dev for position, angle, force sensors
                 use_obstacle     = False,
                 use_heartbeat    = False,
                 use_lp_filter    = False,
                 use_lung_task    = False,
                 use_curve_est    = False,
                 touch_wall       = False,
                 bend_constraint  = np.pi,
                 model_estimation = 'mlc',
                 motion_control   = 'mpc',
                 update_threshold = 1,
                 run_loop         = True):
    # initializes the the low level kinematic model (formerly g_PyAuris)
    robot_com = RobotModelInterface(use_leader      = use_leader, 
                                    use_sheath      = use_sheath,
                                    sensor_noise    = sensor_noise,
                                    use_obstacle    = use_obstacle,
                                    use_heartbeat   = use_heartbeat,
                                    use_lung_task   = use_lung_task,
                                    bend_constraint = bend_constraint,
                                    touch_wall      = touch_wall)
    # initializes the IDM control level
    idm = IDM_model.IDM(robot_com           = robot_com, 
                        global_variables    = [], 
                        use_leader          = use_leader, 
                        use_sheath          = use_sheath,
                        use_orientation     = use_orientation,
                        use_low_pass_filter = use_lp_filter,
                        model_estimation    = model_estimation,
                        motion_control      = motion_control,
                        use_curve_est       = use_curve_est)
    # runs normal task loop
    if run_loop:
        loopIDM(idm, update_threshold = update_threshold, use_lung_task = use_lung_task)
    else: # test specific q moves
        idm.testMoves()
    idm.saveHistory()
    print(':)')


    
def loopIDM(idm, update_threshold = 1, use_lung_task = False):
    '''
    High level control loop of the robot
    Controls the timing (for real robot) of the updates and commands
    '''
    # constants
    J_threshold = update_threshold
    idm.getInitialJacobian()
    if use_lung_task:
        trajectory = getTrajectory('lung', steps = 1, cycles = 1, steps_to_ins = 1)
    else:
        trajectory = getTrajectory('circle', steps = 5, cycles = 1, steps_to_ins = 1)
    idm.setTrajectory(trajectory)
    loopStartTime = time.perf_counter() 
    totalIter = 0

    print('Beginning Loop')
    while idm.trajectory and totalIter < 3e3:
        loopStart = time.perf_counter()
        if totalIter % 1000 == 0: 
            print(totalIter, len(idm.trajectory))
        # Make moves based on next point on trajectory
        tm = time.perf_counter()
        idm.motionUpdate()
        tm = 1000 * (time.perf_counter() - tm)
        # Get Auris update, reads motor positions, tensions, etc 
        ta = time.perf_counter()
        idm.aurisUpdate()
        ta = 1000 * (time.perf_counter() - ta)
        # Get position and force updates (history recording here)
        ts = time.perf_counter()
        idm.sensorUpdate()
        ts = 1000 * (time.perf_counter() - ts)
        # Jacobian update, covers all model estimation functions.
        tj = time.perf_counter()
        idm.jacobianUpdate(threshold = J_threshold) 
        tj = 1000 * (time.perf_counter() - tj)
        # if totalIter % 200 == 0: 
        #     print(tm, ta, ts, tj)
        totalIter += 1
    print('End Loop')





