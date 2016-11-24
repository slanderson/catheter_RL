'''
optimal_control.py

this file determines the changes in motor positions (currents in the future)
to accomplish the desired task space motions

J. Sganga 8/20/2016 
'''

import time, sys
import numpy as np
from functions.modelless_helper import inv_equal_opposite
import functions.convex_helper as ch
from robot.control.task_space_control import get_two_points

'''
Convert desired dx into actuator space (dq) using the psuedo inverse of J
ensuring equal and opposite dq, need to remove weights on dq after 
inverse (could be done during, but diagonal matrix is easily handled after)
'''
def find_dq(robot):
    if robot.model_flag:
        joy = robot.xbox.read_joystick()
        robot.dq_desired = joy_to_dq(robot, joy)

    elif robot.dx_desired.dot(robot.dx_desired) > 1e-8:
        J = robot.R.dot(robot.J) # rotating the model, R is identity for MLC and MBC
        if robot.model_estimation.lower() == 'abc': 
            J[:,-1] = robot.J_mlc[:,-1]
        jacobian_inverse(robot, J)
        mlc_tension(robot, J)# tension, single step, no dq constraint, 
        mpc_cvx(robot, J)

def jacobian_inverse(robot, J):
    # simple call, only safety is keeping wire displacement equal and opposite for 
    # antagonistic pairs
    if not robot.motion_control.lower() == 'jinv':
        return
    Wdq = inv_equal_opposite(J, robot.dx_desired)
    dq  = Wdq / np.diag(robot.W)
    robot.dq_desired = robot.expand_q.dot(dq)
    # long shot - didn't work!
    # delta_x = (dq[1] - dq[3])
    # delta_y = (dq[2] - dq[0])
    # robot.dq_desired = deltas_to_dq(robot, [delta_x, delta_y, dq[-1]])
    # robot.dq_desired[-2] = dq[-1] # insertion

def mlc_tension(robot, J):
    # from yip's mlc
    if not robot.motion_control.lower() == 'tension':
        return
    amps_initial = np.ones(robot.q_num) * 0.01, # initial current needs to be calculated better...
    amps_initial[-1] = 0 # insertion motor
    robot.dq_desired = ch.getDQTensionMin(J.dot(robot.W), 
                       robot.dx_desired, 
                       robot.K, 
                       robot.q_amps[robot.q_list], 
                       amps_initial) 

def mpc_cvx(robot, J):# mpc cvxgen, 3x5
    if not robot.motion_control.lower() == 'mpc_cvx':
        return
    next_two_x = get_two_points(robot)
    amps_initial = np.ones(robot.q_num) * 0.01
    amps_initial[-1] = 0 # insertion motor
    dq = ch.get_mpc_cvx(J.dot(robot.W), 
                        robot.x_sensed[robot.x_list], 
                        next_two_x,
                        robot.K, 
                        robot.amps[robot.q_list] - robot.amps_initial[robot.q_list],
                        robot.dq_desired[robot.q_list])
    robot.dq_desired  = robot.expand_q.dot(dq)
    robot.dx_predicted = ch.get_mpc_prediction()

def joy_to_dq(robot, joy):
    '''
    Integrates the joy stick motion bc mapping is a 1:1 with q,
    so take the instantaneous joy as djoy
    Need to make sure the speeds are set so that it doesn't get 
    reduced by the max_move_per_cyle term in the GoToDQ function. If it 
    does then you'll notice the q's changing after you release the joy stick
    '''
    q_joy_last = auris_joy_model(robot.joy_past) # last move we wanted to make
    dq = q_joy_last - (robot.q_desired - robot.q_initial)
    if dq.dot(dq) < 0.1: # prevents q_desired from getting pushed too far out.
        robot.joy_past += np.asarray(joy)
        q_desired_joy = auris_joy_model(robot.joy_past)
        dq = q_desired_joy - (robot.q_desired - robot.q_initial)
    return dq

def auris_joy_model(joy):
    # hard coded catheter gains (newer control from Allen Jiang 11/19/15)
    lKa = 10.0
    lKs = 1.6
    sKa = 13.0
    sKs = 0.1
    r = 1.89
    iK = 50
    scale_down = 0.01
    # rotating the inputs to match the 90 degree rotation of the camera image 
    # needs to be verified...
    joyLY, joyLX, joyRY, joyRX, joyT = joy
    sP, sY = [-joyLY, joyLX]
    # lP, lY = [(-joyRX + joyRY) / 2., (joyRX + joyRY) / 2.]
    lP, lY = -joyRY, joyRX
    insertion = joyT

    s1 = (-sKa if sY < 0.0 else (sKs - r)) * sY + sKs * abs(sP)
    s3 = (sKa if sY > 0.0 else (r - sKs)) * sY + sKs * abs(sP)
    l1 = (-lKa if lY < 0.0 else (lKs - r)) * lY + lKs * abs(lP)
    l3 = (lKa if lY > 0.0 else (r - lKs)) * lY + lKs * abs(lP)
    s4 = (-sKa if sP < 0.0 else (sKs - r)) * sP + sKs * abs(sY)
    s2 = (sKa if sP > 0.0 else (r - sKs)) * sP + sKs * abs(sY)
    l4 = (-lKa if lP < 0.0 else (lKs - r)) * lP + lKs * abs(lY)
    l2 = (lKa if lP > 0.0 else (r - lKs)) * lP + lKs * abs(lY)
    i = iK * insertion
    return scale_down * np.asarray([s1, s2, s3, s4, l1, l2, l3, l4, i, 0])

def deltas_to_dq(robot, deltas):
    # using auris model to try to mitigate backlash...
    delta_x, delta_y, d_ins = deltas
    joy = [0, 0, -delta_x, delta_y, d_ins]
    return joy_to_dq(robot, joy)



