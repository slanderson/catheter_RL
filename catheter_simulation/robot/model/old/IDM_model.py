"""
IDM_model mimics the commands of the IDM.py that is used to control the acutal robot
Many commands seem pointless, but are left in place to keep the integration between
simulation and reality as seemless as possible

Author: Jake Sganga
Date: Oct 2015
"""

import sys
import time
# import cvxpy
import numpy as np
# import multiprocessing
import pickle

# IMPORTS
from robot_model.devices.position_sensor.position_sensor_wrapper import PoseSensor
from robot_model.devices.force_sensor.force_sensor_wrapper import ForceSensor
from robot_model.functions import LinearJacobianUpdate as lju
from robot_model.functions import ConvexHelper as ch
from robot_model.functions.FilterHelper import filter_segment
from robot_model.functions.KalmanFilter import kalmanFilterUpdate, ukf_measurement_update, ukf_time_update, ukf_curve_update, get_base_angles
from robot_model.functions.RotationUpdate import rotationEstimation
import robot_model.functions.trig as trig 

# cvx imports
from robot_model.lib import tension_greedy_3_5 as cvx_pyd_35
from robot_model.lib import tension_greedy_3_9 as cvx_pyd_39
from robot_model.lib import tension_greedy_5_9 as cvx_pyd_59


#################################################################################
 
###############################################################################
#
# Class IDM
#   Main object for controlling the IDM.
#   A nice way to use the system. 
#
###############################################################################
class IDM(object):
    def __init__(self, 
                 robot_com, 
                 global_variables = [], 
                 use_leader = True, 
                 use_sheath = False,
                 use_orientation = True, 
                 use_low_pass_filter = False,
                 model_estimation = 'mlc',
                 motion_control   = 'mpc',
                 use_curve_est    = False):
        
        self.robot_com = robot_com
        self.beginning_of_time = time.perf_counter()
        # Impose motion limits <-- Hardcoded,
        self.max_position_rel = [55]*(4) + [55]*(4) + [1000] * (2)
        self.min_position_rel = [-6]*(4) + [-6]*(4) + [-30] * (2)     

        '''
        FullQ = 10 motor values, need to tell the robot low level commands with all 10 
        [s1,s2,s3,s4,l1,l2,l3,l4,i1,i2]
        antagonistic pairs: (s1 - s3), (s2 - s4), (l1 - l3), (l2 - l4)
        i1: in model, is global ins of sheath and leader
        i2: in model, is relative inserstion (length of leader beyond tip of sheath)
        in robot, only using i1, depending on config, it controls one or the other

        Note, original code used g_PyAuris, which is now going to be replaced by 
        the class variable self.robot_com (robot communication). Allows parameters 
        to be passed from run_model level
        '''    
        
        self.q_num_full = 10 # lists that accept getcurrent commands, etc. get 10 values, will need to slice according to q_num
        self.q_list_full = list(range(self.q_num_full))
        self.use_leader = use_leader
        self.use_sheath = use_sheath
        if use_leader and use_sheath:
            self.q_num = 9# ASSUMES NO SECOND INSERTION MOTOR
            self.q_list = list(range(self.q_num))
        elif use_leader:
            self.q_num = 5
            self.q_list = [4,5,6,7,8]
        else:
            self.q_num = 5
            self.q_list = [0,1,2,3,8]
        

        self.x_num_full = 6
        self.x_num = 3 if not use_orientation else self.x_num_full
        self.x_list      = list(range(self.x_num)) # simple now, but could be changed to account or aex cntrl
        self.x_list_full = list(range(self.x_num_full))

        self.filter_order    = 2
        self.cutoff_freq     = 10
        self.sensor_freq     = 100

        self.q               = np.zeros(self.q_num)
        self.dq              = np.zeros(self.q_num_full)
        self.dq_desired      = np.zeros(self.q_num)
        self.q_amps          = np.zeros(self.q_num_full)
        self.q_amps_raw      = np.zeros(self.q_num_full)
        self.q_initial       = np.zeros(self.q_num_full)
        self.q_initial[-1]   = 50
        self.q_desired       = self.q_initial.copy()
        self.q_past_J_update = self.q.copy()
        self.q_desired_raw   = np.zeros((self.filter_order + 1, self.q_num_full))
        
        self.q_filter        = [np.tile(self.q_initial.copy(), (self.filter_order + 1, 1)),
                                np.tile(self.q_initial.copy(), (self.filter_order + 1, 1))]

        self.robot_com.SetDesiredPosition(*self.q_desired)

        self.x_initial       = np.asarray(self.robot_com.GetCoordinatePosition())[self.x_list_full]
        print(self.x_initial)
        self.x_sensed        = np.zeros(self.x_num_full)
        self.x_desired       = self.x_sensed[self.x_list].copy()
        self.x_past_J_update = self.x_sensed[self.x_list].copy()
        self.x_raw           = np.zeros(self.x_num_full)
        self.dx_desired      = np.zeros(self.x_num)
        self.dx_expected     = np.zeros(self.x_num)
        self.dx_predicted    = np.zeros((15, 3))
        self.x_filter        = [np.zeros((self.filter_order + 1, self.x_num_full)),
                                np.zeros((self.filter_order + 1, self.x_num_full))]

        self.force_num           = 3
        self.force_num_full      = 6 
        self.force_sensed        = np.zeros(self.force_num_full)
        self.force_past_J_update = np.zeros(self.force_num_full)
        self.force_raw           = np.zeros((self.filter_order + 1, self.force_num_full))
        self.force_filter        = [np.zeros((self.filter_order + 1, self.force_num_full)),
                                    np.zeros((self.filter_order + 1, self.force_num_full))]
        
        

        self.trajectory    = []
        self.history_array = []      

        self.position_sensor  = None
        self.force_sensor     = None
        self.robust_flag      = False
        self.use_low_pass_filter = use_low_pass_filter
        self.global_variables = None
        self.model_estimation = model_estimation # mlc, mbc, kf, r, ekf, ukf
        self.motion_control   = motion_control # jinv, tension, mpc
        self.use_curve_est    = use_curve_est # in UKF, estimate the curvature, theta

        self.J = np.zeros((self.x_num, self.q_num))
        self.W = np.eye(self.q_num)
        self.P = np.zeros((self.x_num,self.x_num))
        self.P_free = np.zeros((self.x_num,self.x_num))
        self.R = trig.R_reorient if not self.model_estimation.lower() == 'mlc' else np.eye(3)
        self.K = np.zeros((self.q_num, self.q_num))

        self.gain_pos = 1
        self.gain_force = 1
        self.force_ref = 0.040 # 40 mN

        self.cov_J = np.eye(self.x_num * self.q_num) * 1
        self.tension = np.zeros(self.q_num_full)

        #CVX initialization
        if self.q_num == 5:
            self.cvx_tension = cvx_pyd_35.CVXWrapper()
        elif self.x_num == 3:
            self.cvx_tension = cvx_pyd_39.CVXWrapper()
        else:
            self.cvx_tension = cvx_pyd_59.CVXWrapper()

        # state - angle estimation, start knowing reorient
        self.angles_sensed  = np.array([0, np.pi/2, 0])
        self.angles_model   = np.array([0, np.pi/2, 0])
        self.R_model        = trig.R_reorient
        self.R_last         = trig.R_reorient
        self.cov_state      = np.eye(1) # changes depending on number of states
        self.curve_model    = np.array([45., 0., 0.]) # s, theta, phi 
        self.curve_sensed   = self.curve_model.copy()
        if self.use_curve_est:
            self.cov_state = np.eye(4)
            self.cov_state      = np.eye(1)

        self.angle_filter = [np.tile(self.angles_sensed.copy(), (self.filter_order + 1, 1)),
                             np.tile(self.angles_sensed.copy(), (self.filter_order + 1, 1))]
            


        #################
        # start communicating with lower levels
        self.aurisUpdate()
        self.sensorUpdate()
        print('start, x ',self.x_sensed, 'q', self.q)

    ################# Logging ######################################

    def initializeLogging(self):
        ''' initializes the array that holds the recorded values. 
        Use a list of arrays of arrays. Allows changes in size (important for xbox controls
        switching between modes). Pickled at the end of the run. '''
        first_history = self.getHistoryArray()
        self.history_array = list(np.zeros((int(3e4), first_history.size)))
        self.history_array[0] = first_history
        self.history_index = 1

    def getHistoryArray(self):
        ''' array of recorded values for later analysis'''
        return np.asarray([
            [time.perf_counter() - self.beginning_of_time],
            self.x_sensed,
            self.x_raw,
            self.x_desired,
            self.q,
            self.q_desired,
            self.J, 
            self.W,
            self.q_amps,
            self.force_sensed,
            self.force_raw,
            self.R,
            self.tension,
            self.angles_sensed,
            self.angles_model,
            self.curve_sensed.copy(),
            self.curve_model,
            self.dx_predicted])

    def updateHistory(self):
        ''' fills the history array, advances the index, increases the size when necessary'''
        if not self.history_array:
            self.initializeLogging()
        self.history_array[self.history_index] = self.getHistoryArray() # copy?
        self.history_index += 1
        if self.history_index == len(self.history_array):
            self.history_array += list(np.zeros((int(3e4), len(self.history_array[0]))))
            
    def saveHistory(self):
        '''saves the history array in a pickle file, preserving the data structures '''
        with open( "robot_model/data/history.p", "wb" ) as output:
            pickle.dump(np.asarray(self.history_array[:self.history_index]), output) 
    


    ################# Reading Sensors #########################################
    def readMotorPositions(self):
        '''get positions of the motors (q) from kinematic model, equal to q_desired in this case,
        but in real robot, this is the encoder sensor reading of the motor'''
        self.q = np.asarray(self.robot_com.GetActualPosition()) - self.q_initial

    def readMotorAmps(self):
        '''does nothing in simulation currently, for robot reads current draw for each motor'''
        self.q_amps  = np.asarray(self.robot_com.GetMotorCurrents())

    def initializePoseSensor(self):
        ''' Talk to position sensor (Ascension) '''
        self.position_sensor = PoseSensor(self.robot_com)
        self.position_sensor.initialize()

    def readPoseSensor(self):
        """
        Reads the current the position and orientation of the robot from the 
        position sensor. Updates the variable self.x_sensed

        Units: mm and rad
        """
        # position based purely on forward kinematic model (q values) and disturbances
        if not self.position_sensor:
            self.initializePoseSensor()
        pos, quality  = self.position_sensor.readPoseSensor()
        self.x_sensed  = pos[self.x_list_full].copy() - self.x_initial
        if self.use_low_pass_filter:
            self.x_raw    = self.x_sensed.copy()
            self.x_sensed = self.filterData(self.x_raw, self.x_filter).copy()            

    def initializeForceSensor(self):
        ''' Talk to force sensor (ATI mini40) '''
        self.force_sensor = ForceSensor(self.robot_com)
        self.force_sensor.initialize()

    def readForceSensor(self):
        """
        returns a numpy array of the xyz forces and torques
        Needs to get forces from the low level kinematic model
        Makes the P and P_free matrices that project onto the constrained/unconstrained planes

        Units: N and Nm
        """
        if not self.force_sensor:
            self.initializeForceSensor()
        self.force_sensed = self.force_sensor.readForceSensor()
        if self.use_low_pass_filter:
            self.force_raw    = self.force_sensed.copy()
            self.force_sensed = self.filterData(self.force_raw, self.force_filter).copy()

        force_threshold = 0.02
        force_above_thresh = np.zeros(self.force_num)
        for i, f in enumerate(self.force_sensed[:self.force_num]):
            if abs(f) > force_threshold:
                force_above_thresh[i] = f

        force_norm = np.sqrt(force_above_thresh.dot(force_above_thresh))
        # force_norm = np.sqrt(self.force_sensed[:self.force_num].dot(self.force_sensed[:self.force_num]))
        # unit vector in direction of forces
        uF = np.zeros(self.x_num) 
        force_norm_threshold = 0
        if force_norm:
            # print('here', force_norm)
            # uF[:self.force_num] = self.force_sensed[:self.force_num] / force_norm
            uF[:self.force_num] = force_above_thresh / force_norm
        # projection onto force/constrained direction
        self.P = np.outer(uF, uF)
        # unconstrained plane, angles always unconstrained for now
        self.P_free = np.eye(self.x_num) - self.P 

    def readTensionSensor(self):
        ''' gets the tension reading, in simulation, just a heuristic on q displacments '''
        self.tension = self.robot_com.getTensions()

    def readModelAngles(self):
        ''' gets the true model angles, for records '''
        self.R_model, self.angles_model = self.robot_com.getModelRotation()


    def sensorUpdate(self):
        ''' grabs a pose reading, updates the history array. also grabbing force reading'''
        self.readPoseSensor()
        self.readForceSensor()
        self.updateHistory() #assumes pose/force sensors update faster than other variables in history_array 
        if self.global_variables:
            self.x_global[:]     = self.x_sensed
            self.force_global[:] = self.force_sensed

    def aurisUpdate(self):
        ''' reads motor positions, motor current draws, posts to global variables'''
        self.readMotorPositions()
        self.readMotorAmps()
        self.readTensionSensor()
        self.readModelAngles()
        if self.global_variables:
            self.q_global[:]  = self.q

    ################# Helper functions #########################################
    def spinWait(self, waitTimeSec):
        """ Burn CPU to make more accurate timing."""
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < waitTimeSec: 
            pass

    def fullQtoActuatedQ(self, q):
        ''' use a subset of q values for motions and linear alg 
        (don't want J matrix full of zeros)
        '''
        return np.asarray([q[index] for index in self.q_list])

    def actuatedQtoFullQ(self, q):
        ''' opposite of full to actuated'''
        fullQlist = np.zeros(self.q_num_full)
        for i, index in enumerate(self.q_list):
            fullQlist[index] = q[i]
        return fullQlist


    ################# Motion Commands #########################################
    def motionUpdate(self):
        ''' Sequence of commands, get a new dx based on trajectory list, then find
        corresponding dq that is estimated to reach the dx, then make that move'''
        self.setDxDesired()
        self.setDqDesired()
        self.goToDqDesired()

    def setDxDesired(self, margin = 0.5):
        '''
        In robot case, switches between grabbing next x_desired from xbox controller and from trajectory array
        in model, only using trajectory 
        Removes poses from list until trajectory list is empty, signifying a successful completion
        '''
        error = self.updateDxWithForce(self.x_desired - self.x_sensed[self.x_list])
        # if error.dot(error) < margin and self.trajectory:
        if max(abs(error)) < margin and self.trajectory:
             # removes list of position values from the trajectory list
            self.x_desired =  np.asarray(self.trajectory.pop(0))[self.x_list]
            # print(self.x_desired)
            if not self.trajectory: 
                print('Task completed!')
        self.dx_desired = self.updateDxWithForce(self.gain_pos * (self.x_desired - self.x_sensed[self.x_list]))
        model_frame = np.linalg.inv(self.R)
        self.dx_desired[:3] = model_frame.dot(self.dx_desired[:3]) # could solve the rotation differently so this isn't an inverse
        
        # adjust angles, if estimating rotation:
        if self.x_num == 5 and not np.array_equal(self.R, np.eye(3)):
            # forming rotation matrix of tip pose in ground frame, pre multiplying by R_inv to put it in robot's local frame,
            # then converting rotation matrix back into euler angles
            # beta = 90 - elevation 
            print('dont use, old formulation')
            # angle_current = trig.getABCfromR(np.linalg.inv(self.R).dot(trig.R_zyx([self.x_sensed[3], np.pi/2 - self.x_sensed[4], 0])))
            # angle_desired = trig.getABCfromR(np.linalg.inv(self.R_last).dot(trig.R_zyx([self.x_desired[3], np.pi/2 - self.x_desired[4], 0])))
            # da = angle_desired - angle_current
            # self.dx_desired[3] = da[0] # d_azimuth = d_yaw
            # self.dx_desired[4] = -da[1] # d_elevation = -d_pitch


    def setDqDesired(self):
        '''
        Convert desired dx into actuator space (dq) using the psuedo inverse of J
        ensuring equal and opposite dq
        need to remove weights on dq after inverse (could be done during, but 
        diagonal matrix is easily handled after)
        '''
        # self.dq_desired = np.zeros(self.q_num_full)
        if self.dx_desired.dot(self.dx_desired) > 0:
            tension_initial = np.zeros(self.q_num) 
            # simple jinv, equal opposite
            if self.motion_control.lower() == 'jinv':
                Wdq = lju.GetDQfromPsuedoInv_EqualOpposite(self.J, self.dx_desired)
                self.dq_desired = Wdq / np.diag(self.W)
            # tension, single step, no dq constraint, 
            elif self.motion_control.lower() == 'tension':
                self.dq_desired = ch.getDQTensionMin(self.J.dot(self.W), 
                                  self.dx_desired, 
                                  self.robot_com.getStiffness()[self.q_list,:][:,self.q_list], 
                                  self.tension[self.q_list], 
                                  tension_initial)
            # note that cvx solver only working for 5x9 case right now
            elif self.motion_control.lower() == 'tension_cvx':
                self.cvx_tension.runSolver(self.J.dot(self.W).flatten('F'),
                                           self.dx_desired,
                                           self.robot_com.getStiffness()[self.q_list,:][:,self.q_list].flatten('F'),
                                           self.tension[self.q_list] - tension_initial)
                self.dq_desired = np.asarray([self.cvx_tension.getData(i) for i in range(self.q_num)])
            # MPC
            elif self.motion_control.lower() == 'mpc':
                self.dq_desired = ch.getMPCdq(self.J.dot(self.W), 
                                              self.dx_desired, 
                                              self.robot_com.getStiffness()[self.q_list,:][:,self.q_list], 
                                              self.tension[self.q_list], 
                                              tension_initial,
                                              steps = 6)
            # MPC2
            elif self.motion_control.lower() == 'mpc2':
                next_two_x = np.zeros((self.x_num, 2))
                next_two_x[:,0] = self.x_desired.copy()
                if self.trajectory:
                    next_two_x[:,1] = np.asarray(self.trajectory[0])[self.x_list]
                self.dq_desired = ch.getMPCdq2(self.R.dot(self.J.dot(self.W)), 
                                              self.x_sensed[self.x_list], 
                                              next_two_x,
                                              self.robot_com.getStiffness()[self.q_list,:][:,self.q_list], 
                                              self.tension[self.q_list] - tension_initial, 
                                              self.dq_desired.copy(),
                                              steps = 5)
            # mpc cvxgen, 3x5
            elif self.motion_control.lower() == 'mpc_cvx':
                next_two_x = np.zeros((self.x_num, 2))
                next_two_x[:,0] = self.x_desired.copy()
                if self.trajectory:
                    next_two_x[:,1] = np.asarray(self.trajectory[0])[self.x_list]
                self.dq_desired = ch.get_mpc_cvx(self.R.dot(self.J.dot(self.W)), 
                                              self.x_sensed[self.x_list], 
                                              next_two_x,
                                              self.robot_com.getStiffness()[self.q_list,:][:,self.q_list], 
                                              self.tension[self.q_list] - tension_initial,
                                              self.dq_desired.copy())
                self.dx_predicted = ch.get_mpc_prediction().copy()

            elif self.motion_control.lower() == 'tension_mpc':
                self.dq_desired = ch.get_tension_min(self.R.dot(self.J.dot(self.W)), 
                                              self.dx_desired, 
                                              self.robot_com.getStiffness()[self.q_list,:][:,self.q_list], 
                                              self.tension[self.q_list] - tension_initial,
                                              self.dq_desired.copy())

            



    def goToDqDesired(self):
        '''
        Need to have goToDQ flexible for any dq, but would like to avoid 
        handling idm variables in the run_model
        '''
        if self.dq_desired.dot(self.dq_desired) > 0:
            self.goToDQ(self.actuatedQtoFullQ(self.dq_desired))

    def goToDQ(self, dq, max_velocity = 10):
        """
        GoToDQ -> Goes to the commanded wire position at the velocity such 
        that all leader and sheath wires reach the commanded position at the same 
        time, the fastest one at the maximum allowed velocity

        Expected to be run within a while loop so the q_desired gets updated accordingly

        needs dq as a full q list
        """
        estimated_cycle_time = 10./1000
        dq = np.asarray(dq)
        # now dq at max velocity: dq_max = max_velocity*cycletime (don't want ratio > 1)
        max_dq = 1.0/5.0 # the rough absolute limit that the SetDesired Position can handle
        max_move_per_cyle = min(1, 
                                max_velocity * estimated_cycle_time / max(abs(dq)),
                                max_dq / max(abs(dq)))
        # max_move_per_cyle = 1
        dq *= max_move_per_cyle
        self.q_desired += dq 
        self.q_desired[9] = self.q_initial[9]
        self.robot_com.SetDesiredPosition(*self.q_desired) # *list breaks it into the individual arguments
        if self.use_low_pass_filter:
            self.q_desired_raw = self.q_desired.copy()
            self.q_desired     = self.filterData(self.q_desired_raw, self.q_filter).copy()
            dq = self.q_desired - self.q_filter[1][-3] # this filtered point - last filtered
        self.dx_expected += self.J.dot(self.fullQtoActuatedQ(dq)) # for rotation update


    def goToQ_loop(self, q):
        '''
        runs goToDQ many times until the q is reached
        used to go to home, for example
        '''
        while(np.linalg.norm(np.asarray(q) - np.asarray(self.q_desired))**2 > 0.01):
            self.sensorUpdate()
            self.goToDQ(np.asarray(q) - np.asarray(self.q_desired))

    
    def testMoves(self):
        '''
        simple way to check outputs of the system based on q's
        '''
        self.aurisUpdate()
        self.q_desired = np.zeros(10) + self.q_initial
        # self.q_desired[-1] = 20
        # if self.q_num == 5:
        # self.q_desired[-1] = 0  
        # self.q_desired[0] = -5
        # self.q_desired[1] = 5
        # self.q_desired[7] = 0.5
        # self.q_desired[8] = 40

        for i in range(500):
            print(i)
            self.q_desired[8] += 0.1
            # self.q_desired[2] -= 0.05
            # self.q_desired[4] -= 5 * np.cos(i / 10)
            # self.q_desired[5] += 5 * np.sin(i / 30)
            self.robot_com.SetDesiredPosition(*self.q_desired)
            self.aurisUpdate()
            self.sensorUpdate()

    ###################### Filtering ####################################

    def findRobustDxDq(self, threshold = 5):
        '''
        Needs to be fixed. Should enable smoother J updates...
        '''
        x0 = self.xq_history[-1][0]
        dx_norm = 0
        i = len(self.xq_history) - 1
        while dx_norm < threshold:
            i -= 1
            dx = x0 - self.xq_history[i][0]
            dx_norm = dx.dot(dx)
            if (i == 0): return (0, 0, False)

        dx = self.xq_history[i][0] - x0
        dq = self.xq_history[i][1] - self.xq_history[-1][1]
        if np.linalg.norm(dq) ** 2 < 0.0001: 
            print('bad dq ', dq)
            return (0, 0, False)

        del self.xq_history[:i] # list before this point is unnecessary
        return(dx, dq, True)

    def filterData(self, data_point, filter_array):
        '''
        low pass filters the data using a butterworth filter with 
        a cutoff freq, sensor frequency, and filter order defined in the 
        idm class. the filter array stores the necessary number of filtered
        and raw data points, zero pads  in the beginning. Calls the func 
        filter_segment in the FilterHelper.py and returns the filtered data point.
        '''
        raw_data, filtered_data = filter_array
        # order = min(self.history_index, self.filter_order)
        raw_data[-1] = data_point
        filtered_data[-1] = filter_segment(raw_data, filtered_data[:-1], self.cutoff_freq, 
                                   self.sensor_freq, self.filter_order)
        # shuffling array to put most recent points in place
        raw_data[:-1] = raw_data[1:]
        filtered_data[:-1] = filtered_data[1:]
        return filtered_data[-1]



    #################### Trajcectories #####################################
    def setTrajectory(self, trajectory):
        '''
        sets up the list of position/orientation commands that the robot has to hit sequentially
        sets the first one as self.x_desired
        '''
        self.trajectory = list(np.asarray(trajectory)[:,self.x_list] + self.x_sensed[self.x_list])
        self.x_desired  = np.asarray(self.trajectory.pop(0))[self.x_list] 

    def updateDxWithForce(self, dx):
        '''
        When robot is moving into contact with surface (force'*dx < 0), hybrid force/pos 
        control for MLC separates position control into uncontstrained plane (I-P), while 
        force in P is controlled by scaling the dx_desired in that direction accordingly.
        '''
        if self.force_sensed[:self.force_num].dot(dx[:self.force_num]) < 0: # moving into contact
            dx = self.P_free.dot(dx)
            dx[:self.force_num] += self.getDxForce()
        return dx

    def getDxForce(self):
        # force_threshold = 0.01 #mN
        # dF = np.zeros(self.x_num)
        # for i, f in enumerate(self.force_sensed[:self.force_num]):
        #     if f > force_threshold:#only want to control sensed forces
        #         dF[i] = self.gain_force * (self.force_ref - abs(f)) * f/abs(f) # adjusting sign for pos or neg force
        # K_result = self.robot_com.getStiffness()
        K_result = 20/1000
        # dx_force = - self.P.dot(dF) * (1 / K_result) # dx = -F/k

        force_norm   = np.sqrt(self.force_sensed[:self.force_num].dot(self.force_sensed[:self.force_num]))
        dF_magnitude = self.gain_force * (self.force_ref - force_norm) 
        uF       = self.force_sensed[:self.force_num] / force_norm
        dx_force = - (1 / K_result) * dF_magnitude * uF # dx = -F/k 
        return dx_force

    ######################### model-less control jacobian updates ###########################
    def mlcUpdate(self, dx, dq, step_size):
        '''
        model-less control jacobian update
        makes sure dq has moved (would get singular matrix dividing by 0)
        tests that dx has moved beyond the threshold (changed according how much noise is expected)
        calls the LinearJacobianUpdate module/function to do the linear update step
        updates the self.J array
        hyper parameters: threshold, step size
        ''' 
        if not self.model_estimation.lower() == 'mlc': 
            return
        Wdq = self.W.dot(self.fullQtoActuatedQ(dq))
        J_masked = self.P_free.dot(self.J)
        # J, change_factor = lju.LinearJacobianUpdate(J_masked, Wdq, dx)
        J, change_factor = lju.LinearJacobianUpdate_NoSafety(J_masked, Wdq, dx)
        J_masked_new = (1 - step_size) * J_masked + step_size * J
        self.J = J_masked_new + self.P.dot(self.J) # preserving the constrained J components


    def getInitialJacobian(self): 
        '''
        Gets a Jacobian estimate by moving each set of independent actuators and measuring
        the position displacement. Antagonistic motor pairs are actuated together because 
        they are currently forced to be equal and opposite as tension feedback is not 
        possible.
        '''
        J             = np.zeros((self.x_num, self.q_num))
        self.K        = np.zeros((self.q_num-1, self.q_num-1))
        dq_list       = self.getInitialDqList()
        q_start       = self.q.copy()
        tension_start = self.tension[self.q_list].copy()
        self.goToQ_loop(q_start + self.q_initial)
        self.sensorUpdate()
        x_start = self.x_sensed[self.x_list].copy()
        # wiggle each motor
        for i, dq in enumerate(dq_list):
            q_desired = self.actuatedQtoFullQ(dq) + q_start + self.q_initial
            self.goToQ_loop(q_desired)
            self.sensorUpdate()
            self.aurisUpdate()
            dx   =  self.x_sensed[self.x_list] - x_start
            dtau =  self.tension[self.q_list] - tension_start
            for j, dq_j in enumerate(dq):
                if dq_j != 0:
                    if j < 8: #tendons
                        dq_j*=2
                        # K[:, j] = dtau / dq_j
                    J[:, j] = dx / dq_j
            self.goToQ_loop(q_start + self.q_initial)
        self.J = J

        # Fill out the weighting matrix to now take the form: dx = J W dq
        self.W = np.diag([np.linalg.norm(self.J[:, i]) for i in range(self.q_num)]) # puts angle and mm on same scale.. could separate
        self.J = np.transpose(np.asarray([self.J[:,i] / self.W[i,i] for i in range(self.q_num)]))

        # set up for next time
        self.x_past_J_update = self.x_sensed[self.x_list].copy()
        self.q_past_J_update = self.q_desired.copy()
        self.force_past_J_update = self.force_sensed.copy()

        self.dx_expected *= 0
        if not self.model_estimation.lower() == 'mlc': 
            self.modelUpdate()
            self.W = np.eye(self.q_num)

    '''makes list of q commands for initial J'''
    def getInitialDqList(self):
        dq = 0.75
        if self.q_num == 5:
            dq_list = dq * np.eye(3)
        else:
            dq_list = dq * np.eye(5)
            dq_list[:2] *= 1.5 #sheath tendons 
        dq_list[-1] *= 3 #insertion axis
        dq_list = [self.expandQPairs(this_dq) for this_dq in dq_list]
        return dq_list
    '''tries to make writing out dqs easier'''
    def expandQPairs(self, q):
        q_expanded = np.zeros(self.q_num)
        if self.q_num == 5:
            q_expanded[0] =  q[0]
            q_expanded[1] =  q[1]
            q_expanded[2] = -q[0]
            q_expanded[3] = -q[1]
            q_expanded[4] =  q[2]
        else:
            q_expanded[0] =  q[0]
            q_expanded[1] =  q[1]
            q_expanded[2] = -q[0]
            q_expanded[3] = -q[1]
            q_expanded[4] =  q[2]
            q_expanded[5] =  q[3]
            q_expanded[6] = -q[2]
            q_expanded[7] = -q[3]
            q_expanded[8] =  q[4]
        return q_expanded


    ########################## Kalman Filter ##########################
    def kalmanUpdate(self, dx, dq, step_size):
        '''
        Linear Kalman filter update, which becomes MLC when the variance is fixed with 
        no time update
        '''
        if not self.model_estimation.lower() == 'kf': 
            return
        sensor_noise  = 1
        process_noise = 1 #similar to step size
        model_scale   = 0.95
        Wdq = self.W.dot(self.fullQtoActuatedQ(dq))
        J_masked = self.P_free.dot(self.J)
        J, self.cov_J = kalmanFilterUpdate(J_masked, self.cov_J, dx, Wdq, 
                                            self.q_desired, self.q_past_J_update, 
                                            sensor_noise,process_noise, model_scale)
        J_masked_new = (1 - step_size) * J_masked + step_size * J
        self.J = J_masked_new + self.P.dot(self.J) # preserving the constrained J components
    ########################## Rotation Estimation ########################
    def rotationUpdate(self, dx, step_size):
        '''
        Calls the rotationEstimation function to find better R
        sets self.J to model-based J, rotation of dx is taken care of in setdxdesired
        resets dx_expected
        '''
        if not self.model_estimation.lower() == 'r': 
            return
        self.dx_expected = self.P_free.dot(self.dx_expected) # doing this here to match dx
        self.R_last = self.R.copy()
        self.R = rotationEstimation(self.R, dx, self.dx_expected, step_size)
        self.dx_expected *= 0

    def modelUpdate(self):
        '''
        calls lower level kinematic model to measure jacobian by taking tiny motions with
        each motor (similar to getInitialJacobian, but much finer bc no noise or physical 
        constraints)
        '''
        if self.model_estimation.lower() == 'mlc': 
            return
        if self.use_leader and not self.use_sheath:
            if self.use_curve_est:
                self.J = self.robot_com.get_leader_jacobian(self.q_desired, self.curve_sensed)[self.x_list,:]
            else:
                self.J = self.robot_com.get_leader_jacobian(self.q_desired)[self.x_list,:]
        else:
            self.J = self.robot_com.estimateModelJacobian(self.q_desired)[self.x_list,:][:,self.q_list]

    def jacobianUpdate(self, threshold = 1, step_size = 0.25):
        '''
        Each update (mlc, Kalman, Rotation, etc) needs to make sure a noticable 
        change in dx has happened and some dq occured to avoid singular matrices
        Note that model update is in a different threshold
        '''
        use_robust = bool(self.model_estimation.lower() == 'mlc')
        use_robust = False # need to fix slow down issue when stalled
        dx, dq, update_flag = self.getRobustDelta(threshold = threshold, use_robust = use_robust)

        # each function decides if it runs based on self.model_estimation 
        if dq.dot(dq) > 1e-3:
            self.modelUpdate()

        if update_flag:
            self.mlcUpdate(dx, dq, step_size)
            self.kalmanUpdate(dx, dq, step_size)
            self.rotationUpdate(dx, step_size)
            self.extendedKalmanUpdate(dx[:3], dq, step_size)
            self.unscentedKalmanUpdate(dx[:3], 
                                       dq, 
                                       step_size, 
                                       use_dx = False,
                                       use_pose = True) 
            self.ukf_curve()
            self.angle_update()

            self.x_past_J_update = self.x_sensed[self.x_list].copy()
            self.q_past_J_update = self.q_desired.copy()
            self.force_past_J_update = self.force_sensed.copy()

    def extendedKalmanUpdate(self, dx, dq, step_size = 0.1):
        '''
        EKF update, which is really just non-linear 
        recurssive estimation bc no time update
        '''
        if not self.model_estimation.lower() == 'ekf': 
            return
        residual = dx - self.P_free.dot(self.dx_expected)
        Wdq = self.W.dot(self.fullQtoActuatedQ(dq))
        J_masked = self.P_free.dot(self.J)
        dx_model = J_masked.dot(Wdq)
        angles, self.cov_state = ekfUpdate(self.angles_sensed, 
                                           dx_model, 
                                           residual, 
                                           sensor_noise = 0.05, 
                                           cov_angle = np.eye(3))
        self.angles_sensed = (1 - step_size) * self.angles_sensed + step_size * angles
        self.R = trig.R_zyx(self.angles_sensed) # DEPRECATED!
        self.dx_expected *= 0  

    def unscentedKalmanUpdate(self, 
                              dx, 
                              dq, 
                              step_size = 0.1, 
                              use_dx = True,
                              use_pose = False):
        '''
        UKF update, similar to EKF
        may include time update
        '''
        if not self.model_estimation.lower() == 'ukf': 
            return
        # Wdq = self.W.dot(self.fullQtoActuatedQ(dq))
        # J_masked = self.P_free.dot(self.J)
        # dx_model = J_masked.dot(Wdq)[:3]
        dx_model  = np.array([])
        ab_model  = np.array([])
        curve_model = np.array([])
        dx_sensed = np.array([])
        ab_sensed = np.array([])
        curve_sensed = np.array([])

        self.curve_model = self.robot_com.get_leader_curvature(self.q_desired[4:8])

        if use_dx:
            dx_sensed = dx.copy()
            dx_model  = self.dx_expected
        
        if use_pose: 
            ab_sensed = self.x_sensed[3:].copy() # grabbing azimuth, elevation, roll
            ab_sensed[1] = np.pi / 2 - ab_sensed[1] #turning elevation into beta

        # cov_state = static_cov_state
        cov_state = self.cov_state.copy()
        # print(cov_state)

        new_state, self.cov_state = ukf_measurement_update(self.angles_sensed.copy(),
                                              self.curve_sensed,
                                              dx_model,
                                              self.curve_model,
                                              dx_sensed, 
                                              ab_sensed,
                                              sensor_noise = 0.05, 
                                              cov_state = cov_state,
                                              use_curve = self.use_curve_est)

        self.angles_sensed = (1 - step_size) * self.angles_sensed + step_size * new_state[:3]
        self.R = trig.R_zyz(self.angles_sensed)
        if self.use_curve_est:
            step_size = 1
            self.curve_sensed[0] = self.curve_model[0]
            self.curve_sensed[1] = (1 - step_size) * self.curve_sensed[1] + step_size * new_state[3]
            self.curve_sensed[2] = self.curve_model[2]
            self.cov_state  += np.eye(4) * 1. #time update
            # print(curve_model_full, self.curve_sensed)
        else: 
            self.curve_sensed = self.curve_model.copy()
            self.cov_state  += np.eye(3) * 0.1 #time update
        self.dx_expected *= 0 

    def ukf_curve(self):
        '''
        UKF update that updates the curve parameters only, and deterministically finds base angles
        '''
        if not self.model_estimation.lower() == 'ukfc': 
            return
        last_curve_model = self.curve_model.copy() 
        self.curve_model = np.asarray(self.robot_com.get_leader_curvature(self.q_desired[4:8]))
        delta_curve = self.curve_model - last_curve_model# can update the curve_sensed with delta, if wanted...
        curve_indeces = [1] # indeces to update, [s, theta, phi]

        # self.curve_model[1] *= 1.25

        tip_angles = self.x_sensed[3:].copy() # grabbing azimuth, elevation, roll
        tip_angles[1] = np.pi / 2 - tip_angles[1] #turning elevation into beta

        # cov_state = static_cov_state
        cov_state = self.cov_state.copy()
        # print(cov_state)

        new_curve, new_angles, self.cov_state = ukf_curve_update(self.curve_sensed.copy() + delta_curve,
                                                                 curve_indeces, 
                                                                 self.curve_model.copy(),
                                                                 tip_angles,
                                                                 self.angles_sensed.copy(),
                                                                 sensor_noise = 0.05,
                                                                 cov_state = cov_state)
        step_size = 1.
        self.angles_sensed = self.angles_sensed = (1 - step_size) * self.angles_sensed + step_size * new_angles
        # filter?
        self.R = trig.R_zyz(self.angles_sensed)
        self.curve_sensed = (1 - step_size) * self.curve_sensed + step_size * new_curve

        self.cov_state  += np.eye(len(curve_indeces)) * 1. #time update on covariance
            # print(curve_model_full, self.curve_sensed)
        self.dx_expected *= 0 

    def angle_update(self):
        # determinstically finds the base angles based on the modelled curve and tip orientation
        if not self.model_estimation.lower() == 'au': 
            return
        self.curve_model = np.asarray(self.robot_com.get_leader_curvature(self.q_desired[4:8]))
        self.curve_model[1] *= 1.35
        self.curve_sensed = self.curve_model.copy()

        tip_angles = self.x_sensed[3:].copy() # grabbing azimuth, elevation, roll
        tip_angles[1] = np.pi / 2 - tip_angles[1] #turning elevation into beta

        angles_raw = get_base_angles(self.curve_sensed.copy(), tip_angles)
        self.angles_sensed = self.filterData(angles_raw.copy(), self.angle_filter).copy()
        # self.angles_sensed =  angles_raw.copy() 
        self.R = trig.R_zyz(self.angles_sensed)



    def getRobustDelta(self, threshold = 1, use_robust = False):
        '''
        Returns the most recent dx, dq pair for the jacobian update that fulfills
        the threshold requirements. Also passes an update_flag that reveals if the 
        requirements were met. Works for with and without the robust flag

        Return dx, dq, update_flag
        '''
        q_threshold      = 1e-3 
        robust_threshold = threshold /  10
        # Get singular matrix with no dq
        # only updating in direction without current force
        dx = self.P_free.dot(self.x_sensed[self.x_list] - self.x_past_J_update)
        dq = self.q_desired - self.q_past_J_update
        # immediately meets thresholds
        if dx.dot(dx) > threshold and dq.dot(dq) > q_threshold:
            return dx, dq, True
        # robustification looks through past date
        if use_robust and dx.dot(dx) > robust_threshold:
            index = 0
            while dx.dot(dx) < threshold or dq.dot(dq) < q_threshold:
                index += 1
                check_index = self.history_index - index
                if check_index < 0:
                    return dx, dq, False
                x_test  = self.history_array[check_index][1].copy() #x_sensed
                q_test  = self.history_array[check_index][5].copy() #q_desired
                dx = self.P_free.dot(self.x_sensed[self.x_list] - x_test[self.x_list])
                dq = self.q_desired - q_test
            return dx, dq, True
        return dx, dq, False   




        








