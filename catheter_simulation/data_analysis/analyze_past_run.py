'''
analyze_past_run.py allows different state estimation techniques to be tried on 
a history.p collection. 

expects to receive in the form of a load_history object

needs to load a model based robot_state and loop through feeding in 
the x and q values, recording the saved ones...
'''

import sys, time
import numpy as np
sys.path.append('..')

from robot.model.model_communication import RobotModelInterface
from robot.robot_state import robot_state
from functions.state_estimation import jacobian_update

robot_com = RobotModelInterface(use_leader      = True, 
                                use_sheath      = False,
                                sensor_noise    = [0,0,0],
                                use_obstacle    = False,
                                use_heartbeat   = False,
                                use_lung_task   = False,
                                bend_constraint = np.pi,
                                touch_wall      = False)
robot = robot_state(robot_com           = robot_com,
                    xbox                = [], 
                    global_variables    = [], 
                    use_leader          = True, 
                    use_sheath          = False,
                    use_orientation     = False,
                    use_low_pass_filter = True,
                    model_estimation    = 'abc',
                    motion_control      = 'jinv',
                    dx_threshold        = 25.,
                    q_threshold         = 1e-5,
                    trajectory          = [])


def get_state_estimate(data, model_estimation, start_pt): 
    # model_estimation string
    # start_pt is the index for q_initial (when wires were tensioned)
    N = len(data.time_pts)
    robot.model_estimation = model_estimation
    robot.q_initial = data.q_desired[start_pt,:]
    robot.x_initial = data.x_sensed[start_pt,:]
    J_estimate = np.zeros((N, robot.x_num, robot.q_num))
    R_estimate = np.zeros((N, 3, 3))
    angle_estimate = np.zeros((N, 3))
    robot.dx_expected = np.zeros(3)
    # set x_sensed and q_desired for each loop
    for i in range(start_pt,N):
        robot.q_desired = data.q_desired[i,:]
        robot.x_sensed  = data.x_sensed[i,:]
        dq = data.q_desired[i,robot.q_list] - data.q_desired[i-1,robot.q_list]
        R = data.R[i,:,:]
        J = data.J[i,:,:]
        robot.dx_expected += R.dot(J.dot(dq))
        jacobian_update(robot)
        J_estimate[i,:,:] = robot.J.copy()
        R_estimate[i,:,:] = robot.R.copy()
        angle_estimate[i,:] = robot.angles_sensed.copy()
    return J_estimate, R_estimate, angle_estimate

