'''
state_estimation.py

this file estimates the state of the robot, resulting
in an instantaneous jacobian to be used by the optimal_control.py 
references the robot_state.py for parameters

J. Sganga 8/20/2016 

'''
import numpy as np
import time, sys
from robot.model.model_kinematics import catheter 
import functions.modelless_helper as mlc
import functions.kalman_filter as kf
import functions.trig as trig
from functions.low_pass_filter import filter_data

robot_model = catheter(use_leader     = True,
                       use_sheath     = True,
                       sensor_noise   = [0,0,0], 
                       use_obstacle   = False,
                       use_heartbeat  = False)


def jacobian_update(robot):
    '''
    Each update (mlc, Kalman, Rotation, etc) needs to make sure a noticable 
    change in x and/or q has happened to either avoid singularities or 
    unnecessary computation
    '''
    # only updating in direction without current force (identity in most cases)
    dx  = robot.P_free.dot(robot.x_sensed[robot.x_list] - robot.x_past_J_update)
    dq  = robot.q_desired - robot.q_past_J_update
    # import pdb; pdb.set_trace()
    # each function decides if it runs based on robot.model_estimation 
    model_update(robot)
    base_pose_update(robot)
    if dq.dot(dq) > robot.q_threshold and not robot.model_flag:
        dx, dq = check_robustify(robot, dx, dq)
        if dx.dot(dx) > robot.dx_threshold:
            mlc_update(robot, dx, dq)
            ukf_update(robot, dx[:3], dq, use_pose = False) # pose not working yet

            robot.x_past_J_update = robot.x_sensed[:robot.x_num].copy()
            robot.q_past_J_update = robot.q_desired.copy()
            robot.force_past_J_update = robot.force_sensed.copy()

def model_update(robot, initialize_mlc = False):
    '''
    calls low level kinematic model to measure jacobian by taking tiny motions with
    each motor (similar to getInitialJacobian, but much finer bc no noise or physical 
    constraints)

    only works for both leader and sheath for now. slight tweaks with with motor is the insertion
    and there's a sign flip for the insertion motor.
    '''
    if robot.model_estimation.lower() == 'mlc' and not initialize_mlc: 
        return
    robot.J = get_full_model_J(robot)[robot.x_list,:][:,robot.q_list]
    if initialize_mlc: # super jank
        robot.J = trig.R_reorient.dot(robot.J[:3, :]) # only rotate xyz...

def get_full_model_J(robot):
    '''
    tension control needs access to the angle component of the jacobian,
    so adding this slighly unnecessary function
    '''
    q = robot.q_desired - robot.q_initial # zeroed at beginning
    q[-1] += robot_model.length_leader # adding insertion length offset
    return robot_model.estimateModelJacobian(q)



def check_robustify(robot, dx, dq):
    # only robustifies if it's MLC, 
    # adds index to robust_index list if point has moved, returns new dx, dq 
    # if mlc and a pair are found
    if not robot.model_estimation.lower() == 'mlc': 
        return dx, dq
    robust_threshold = robot.dx_threshold / 10
    if dx.dot(dx) > robust_threshold:
        # add indeces that pass the lower threshold
        robot.robust_index.append(robot.history_index - 1)
        # looks for dx, dq in history that passes threshold
        dx_robust, dq_robust, robust_flag = mlc.robustify(robot)
        if robust_flag:
            return dx_robust, dq_robust
    return dx, dq


def mlc_update(robot, dx, dq):
    '''
    model-less control jacobian update
    makes sure dq has moved (would get singular matrix dividing by 0)
    tests that dx has moved beyond the threshold (changed according how much noise is expected)
    calls the LinearJacobianUpdate module/function to do the linear update step
    updates the robot.J array
    hyper parameters: threshold, step size
    still need to add robustification option
    ''' 
    Wdq = robot.W.dot(robot.shrink_q.dot(dq))
    J_masked = robot.P_free.dot(robot.J_mlc)
    J, change_factor = mlc.linear_jacobian_update(J_masked, Wdq, dx, check_rank = False)
    J_masked_new = (1 - robot.step_size) * J_masked + robot.step_size * J
    robot.J_mlc = J_masked_new + robot.P.dot(robot.J_mlc) # preserving the constrained J components

    if robot.model_estimation.lower() == 'mlc': 
        robot.J = robot.J_mlc.copy()

    if robot.global_variables:
        if robot.vision_flag:
            robot.dx_global_last *= 0
        for i in range(2 * robot.q_num): # taking Y and Z rows
            robot.J_global[i] = np.ravel(robot.J[1:3, :])[i]


def ukf_update(robot, dx, dq, use_pose = False):
    '''
    EKF update, which is really just non-linear 
    recurssive estimation bc no time update
    '''
    if not robot.model_estimation.lower() == 'ukf': 
        return
    dx_model  = np.array([])
    ab_model  = np.array([])
    curve_model = np.array([])
    dx_sensed = np.array([])
    ab_sensed = np.array([])
    curve_sensed = np.array([])

    robot.curve_model = robot_model.get_leader_curvature(robot.q_desired[4:8] - robot.q_initial[4:8])

    if True:
        dx_sensed = np.zeros(3)
        dx_model  = np.zeros(3)
        norm_dx =  np.sqrt(dx.dot(dx))
        norm_model = np.sqrt(robot.dx_expected.dot(robot.dx_expected))
        if norm_dx:
            dx_sensed = dx.copy() / norm_dx #normalizing dx...
        if norm_model:
            dx_model  = robot.dx_expected / norm_model
    
    if use_pose: 
        ab_sensed = trig.aer_to_abc(robot.x_sensed[3:].copy()) # grabbing azimuth, elevation, roll

    cov_state = np.eye(3)
    # cov_state = robot.cov_state.copy()
    # print(cov_state)
    use_curve_est = False
    new_state, robot.cov_state = kf.ukf_measurement_update(robot.angles_sensed.copy(),
                                          robot.curve_model,
                                          dx_model,
                                          robot.curve_model,
                                          dx_sensed, 
                                          ab_sensed,
                                          sensor_noise = 0.05, 
                                          cov_state = cov_state,
                                          use_curve = use_curve_est)
    robot.step_size = 1.
    robot.angles_sensed = (1 - robot.step_size) * robot.angles_sensed + robot.step_size * new_state[:3]
    robot.R = trig.R_zyz(robot.angles_sensed)
    if use_curve_est:
        step_size = 1
        robot.curve_sensed[0] = robot.curve_model[0]
        robot.curve_sensed[1] = (1 - step_size) * robot.curve_sensed[1] + step_size * new_state[3]
        robot.curve_sensed[2] = robot.curve_model[2]
        robot.cov_state  += np.eye(4) * 1. #time update
        # print(curve_model_full, robot.curve_sensed)
    else: 
        robot.curve_sensed = robot.curve_model.copy()
        robot.cov_state  += np.eye(3) * 0.1 #time update
    robot.dx_expected *= 0 
    filter_make_R(robot, robot.angles_sensed)


def base_pose_update(robot):
    # finds the deterministic base angles to fit the curve parameters with the measured tip angles
    # R_tip_ground  = R_base_ground * R_tip_base
    # R_base_ground = R_tip_ground * R_tip_base^T
    # elevation = -pi/2 + beta
    if not robot.model_estimation.lower() == 'abc': 
        return
    curve          = robot_model.get_leader_curvature(robot.q_desired[4:8] - robot.q_initial[4:8])
    angles_to_base = robot_model.getPositionFromCurve(curve)[3:] # extracting expected alpha, beta, gamma from curve
    R_tip_base     = trig.R_zyz(angles_to_base) 
    tip_angles     = trig.aer_to_abc(robot.x_sensed[3:]) # grabbing azimuth, elevation, roll
    tip_angles[2] -= robot.x_initial[5] * np.pi / 180 # to line up with the taped sensor, which is also zeroed
    R_tip_ground   = trig.R_zyz(tip_angles)
    R_base_ground  = R_tip_ground.dot(R_tip_base.T)
    base_angles    = trig.getAnglesZYZ(R_base_ground)
    filter_make_R(robot, base_angles)
    
def filter_make_R(robot, base_angles):
    robot.angles_sensed = filter_data(base_angles, 
                                      robot.angles_filter, 
                                      sensor_freq  = robot.sensor_freq,
                                      filter_order = robot.filter_order)
    robot.R = trig.R_zyz(robot.angles_sensed)
    robot.dx_expected *= 0


