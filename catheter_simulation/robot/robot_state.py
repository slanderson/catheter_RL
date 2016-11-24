"""
Auris Surgical Robotics, Inc.

Cam Lab
Author: Jake Sganga
Date: Oct 2015
"""

import sys
import time
import numpy as np
import pickle

# IMPORTS
from robot.control.motion_control import move_dq, move_loop, test_moves
from robot.control.tension_control import run_tensioning, get_robot_stiffness
from robot.control.task_space_control import set_trajectory, end_trajectory
from robot.devices.position_sensor.position_sensor_wrapper import position_sensor
from functions.low_pass_filter import filter_data
from functions.modelless_helper import initialize_jacobian_estimate
import functions.trig as trig 
from functions.state_estimation import model_update
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()

# number of axes
IDM_AXES = 10

# stiffness, maybe overwritten with real time update...
primary = 2e-2
pair    = 2e-3
sides   = -1e-4 
K_leader = np.array([
                    [primary,   sides,    pair,   sides],
                    [  sides, primary,   sides,    pair],
                    [   pair,   sides, primary,   sides],
                    [  sides,    pair,   sides, primary]])

###############################################################################
#
# robot_state holds and updates the variables that define the robot system
#
###############################################################################
class robot_state(object):
    def __init__(self, 
                 robot_com, 
                 xbox                = [], 
                 global_variables    = [], 
                 use_leader          = True, 
                 use_sheath          = False,
                 use_orientation     = True, 
                 use_low_pass_filter = False,
                 model_estimation    = 'mlc',
                 motion_control      = 'mpc',
                 dx_threshold        = 1.,
                 q_threshold         = 1e-5,
                 trajectory          = []):

        self.robot_com = robot_com # handle to AurisLowLevel or ModelLowLevel depending on real or simulation, respectively
        self.xbox  = xbox  # handle to xbox
        # Impose motion limits <-- Hardcoded,
        # Need to be made dynamic <V.A. (12/4/14)
        # Also, only checking the limits on the IDM for now, skipping ids
        # Adding guess to linear slide (JS 6/26/15)
        self.max_velocity = 25. # mm / s, fast as I feel comfortable moving it 
        self.velocity     = 10. # mm / s, actual velocity used to move in a run
        self.max_position_rel = [55]*(4) + [55]*(4) + [45] * (2)
        self.min_position_rel = [-6]*(4) + [-6]*(4) + [-30] * (2)
        #thresholds for jacobian updates
        self.dx_threshold = dx_threshold
        self.q_threshold  = q_threshold
        self.step_size    = 0.25
        
        self.q_num = 5 # should become 10 when second insertion motor is functional!
        self.q_num_full = 10 # lists that accept getcurrent commands, etc. get 10 values, will need to slice according to q_num
        # motors are [s1, s2, s3, s4, l1, l2, l3, l4, i1, i2]
        self.x_num = 5 # with 3 position and 2 orientation, changes if not using orientation control
        self.x_num_full = 6 # fixed at 3 position and 3 orientation

        # For goToDQ
        self.time_set_position = time.perf_counter()
        # for logging
        self.beginning_of_time = time.perf_counter()

        # lots of initializing!
        self.use_leader_and_sheath = False
        self.use_orientation = False
        self.reduced_flag    = False
        self.use_shooter     = False
        self.cvx_flag        = False
        self.vision_flag     = False
        self.robust_flag     = False
        self.model_flag      = False if not global_variables else True
        self.reset_flag      = False 
        self.xyz_flag        = False
        self.use_guide_sensor = False
        self.end_loop         = False # run_loop uses this flag (pressing X) to end loop
        self.end_trajectory   = False # run_model_loop uses this flag to end simulation
        self.model_estimation = model_estimation #mlc, mbc, ukf, r, kf
        self.motion_control   = motion_control # jinv, mpc_cvx

        self.filter_order    = 2 # set to 0 if want to turn off filter
        self.cutoff_freq     = 5
        self.sensor_freq     = 100
        self.twincat_freq    = 15

        self.global_variables = global_variables # global variables serve as the main distinction with real runs vs model runs
        if global_variables:
            self.global_variables = global_variables
            self.dx_global = global_variables[0]
            self.J_global  = global_variables[1]
            self.q_global  = global_variables[2]
            self.x_global  = global_variables[3]
            self.flags_global     = global_variables[4]
            self.x_desired_global = global_variables[5]
            self.force_global = global_variables[6]
            self.x_raw_global = global_variables[7]
                
        self.dx_global_last = np.zeros(2) # for vision

        self.trajectory     = []
        self.x_history      = []
        self.history_array  = []
        self.joy            = list(np.zeros(5))
        self.joy_past       = list(np.zeros(5))

        self.gain_pos       = 1
        self.gain_force     = 20
        self.force_ref      = 0.04 # N, typical is 0.040 N
        self.force_num           = 3
        self.force_num_full      = 6 
        self.force_sensed        = np.zeros(self.force_num_full)
        self.force_past_J_update = np.zeros(self.force_num_full)
        self.force_raw           = np.zeros((self.filter_order + 1, self.force_num_full))
        self.force_filter        = [np.zeros((self.filter_order + 1, self.force_num_full)),
                                    np.zeros((self.filter_order + 1, self.force_num_full))]

        self.amps            = np.zeros(self.q_num_full)
        self.amps_initial    = np.zeros(self.q_num_full) # just after tensioning
        self.amps_raw        = np.zeros(self.q_num_full)
        self.amps_baseline   = np.zeros(self.q_num_full) # slack wires
        self.amps_filter     = [np.zeros((self.filter_order + 1, self.q_num_full)),
                                np.zeros((self.filter_order + 1, self.q_num_full))]

        

        # state - angle estimation, start knowing reorient
        self.R = np.eye(3)
        self.angles_sensed = np.array([0, np.pi/2, 0])
        self.angles_model  = np.array([0, np.pi/2, 0])
        self.R_model       = trig.R_reorient
        self.R_last        = trig.R_reorient
        self.angles_filter = [np.tile(self.angles_sensed.copy(), (self.filter_order + 1, 1)),
                              np.tile(self.angles_sensed.copy(), (self.filter_order + 1, 1))]
        self.K             = np.zeros((self.q_num_full, self.q_num_full))
        self.K[:4,:4]      = K_leader
        self.K[4:8, 4:8]   = K_leader

        # system all setup Now make sure we are zeroed
        #  sets q_initial!
        self.rezero()
        # Also, set that as the current goal, to check against future goals
        self.robot_com.SetDesiredPosition(*self.q_initial)
        # add on for initialization
        # q_desired is sent to the robot, so it can't have an offset
        # 
        self.q_slack         = self.q_initial.copy()
        self.q_past_J_update = self.q_initial.copy()
        self.q               = self.q_initial.copy()
        self.q_desired       = self.q_initial.copy()
        self.dq              = np.zeros(self.q_num_full)
        self.q_desired_raw   = self.q_initial.copy()
        
        # posistion sensor
        self.position_sensor = position_sensor(global_variables, robot_com) # initializes pose sensor wrapper based on type run
        if global_variables:
            print('Waiting on Ascension to wake up... \n')
            sys.stdout.flush()
            self.x_raw_global[-1] = 1 # Ascension thread flips this flag when a position is read
            while self.x_raw_global[-1]:  
                self.spinWait(1, update = False)# don't want to start history recording til getting real data
            print('initialized ascension ', self.x_raw_global[:self.x_num], 'quality ', self.x_raw_global[self.x_num]) # preserving az, el, roll

        self.x_raw, self.quality = self.position_sensor.read_pose_sensor()
        self.x_sensed            = self.x_raw.copy()
        self.x_desired           = self.x_raw.copy()
        self.x_initial           = self.x_raw.copy()
        self.x_past_J_update     = self.x_raw[:self.x_num].copy()
        self.dx_desired          = np.zeros(self.x_num) 
        self.x_filter            = [np.tile(self.x_raw.copy(), (self.filter_order + 1, 1)),
                                    np.tile(self.x_raw.copy(), (self.filter_order + 1, 1))]

        self.x_guide_raw, self.quality_guide = self.position_sensor.read_second_sensor()
        self.x_guide         = self.x_guide_raw.copy()
        self.x_guide_initial = self.x_guide_raw.copy()
        self.x_guide_filter  = [np.tile(self.x_guide_raw.copy(), (self.filter_order + 1, 1)),
                                np.tile(self.x_guide_raw.copy(), (self.filter_order + 1, 1))]
        self.quality_guide   = 0

        self.dx_expected   = np.zeros(3) # J dq
        self.dx_predicted  = np.zeros((15, 3)) # 15 steps from MPC
        self.robust_index    = []

        # populate inital variables, start history
        self.reinitialize()
        # trajectory offset by x_sensed
        if trajectory:
            set_trajectory(self, trajectory)
            print(self.x_sensed)
            print(self.trajectory)
            print(self.x_desired)

        if global_variables:
            run_tensioning(self, use_guide = True)
            self.rezero() # annoyingly redundant...
            self.update_flags()



        '''Sets the q_initial to current q reading'''
    def rezero(self):
        self.q_initial     = np.asarray(self.robot_com.GetActualPosition())
        self.q_filter      = [np.tile(self.q_initial.copy(), (self.filter_order + 1, 1)),
                              np.tile(self.q_initial.copy(), (self.filter_order + 1, 1))]

    # nominal current for MAXON 118746 is 0.652 A
    '''
    Calling this function should allow for easy switching between modes that require differently
    sized matrices/vectors. Called during initializtion and mid-loop when a mode is changed
    through an xbox command
    '''
    def reinitialize(self):
        self.q_num = 5 if not self.use_leader_and_sheath else 9 # ASSUMES NO INSERTION MOTOR
        self.q_list = [4,5,6,7,8] if not self.use_leader_and_sheath else list(range(self.q_num))
        self.make_sizing_matrices()
        self.x_num = 3 if not self.use_orientation else 5
        self.x_list     = list(range(self.x_num))
        self.dx_desired = np.zeros(self.x_num)
        # so it doesn't throw an error immediately if you try to move, 
        # should getInitialJacobian before doing anything
        self.W = np.eye(self.q_num)   
        self.J = np.eye(self.x_num, self.q_num) # overwritten in model_update
        self.P = np.zeros((self.x_num,self.x_num))
        self.P_free = np.eye(self.x_num)
        self.R = trig.R_reorient if not self.model_estimation.lower() == 'mlc' else np.eye(3)

        self.dq_desired = np.zeros(self.q_num_full)
        self.aurisUpdate()
        self.sensorUpdate() # history recording
        model_update(self, initialize_mlc = True)
        self.J_mlc = self.J.copy()
        self.x_past_J_update = self.x_sensed[:self.x_num].copy()
        self.q_past_J_update = self.q_desired.copy()

    def make_sizing_matrices(self):
        # calls to and from robot are in arrays of size q_num_full
        # other functions are easier with just the motors that are being used
        # shrink_q is a matrix:     small = shrink_q.dot(full)
        # expand_q is the transpose: full = expand_q.dot(small)
        self.shrink_q = np.zeros((self.q_num, self.q_num_full))
        for i, q_index in enumerate(self.q_list):
            self.shrink_q[i, q_index] = 1.
        self.expand_q = self.shrink_q.T
        
    ################# Logging ######################################
    def initializeLogging(self):
        first_history = self.getHistoryArray()
        self.history_array = list(np.zeros((int(3e4), first_history.size)))
        self.history_array[0] = first_history
        self.history_index = 1

        # logging, array-based for later pickle dump
    def getHistoryArray(self):
        return np.asarray([
            [time.perf_counter() - self.beginning_of_time],
            self.x_sensed.copy(),
            self.x_raw.copy(),
            self.x_desired.copy(),
            self.q.copy(),
            self.q_desired.copy(),
            self.q_desired_raw.copy(),
            self.J.copy(), 
            self.W.copy(),
            self.R.copy(),
            self.angles_sensed.copy(),
            self.angles_model.copy(),
            self.amps.copy(),
            self.amps_raw.copy(),
            self.force_sensed.copy(),
            self.force_raw.copy(),
            self.x_guide.copy(),
            self.dx_predicted.copy()])

    def updateHistory(self):
        if not self.history_array:
            self.initializeLogging()
        self.history_array[self.history_index] = self.getHistoryArray()
        self.history_index += 1
        if self.history_index == len(self.history_array):
            self.history_array += list(np.zeros((int(3e4), len(self.history_array[0]))))
        
    def saveHistory(self):
        with open( computer_paths.data_folder +  "history.p", "wb" ) as output:
            pickle.dump(np.asarray(self.history_array[:self.history_index]), output) 


    ################# Reading Sensors #########################################

    def readMotorPositions(self):
        '''get positions of the motors (q) from encoder sensor reading of the motors'''
        self.q = np.asarray(self.robot_com.GetActualPosition()) - self.q_initial

    def readMotorAmps(self):
        '''does nothing in simulation currently, for robot reads current draw for each motor'''
        self.amps     = np.asarray(self.robot_com.GetMotorCurrents())
        self.amps_raw = self.amps.copy()
        self.amps     = filter_data(self.amps_raw, 
                                      self.amps_filter, 
                                      sensor_freq = self.twincat_freq, 
                                      filter_order = self.filter_order).copy()

    def readPoseSensor(self):
        """
        Reads the current the position and orientation of the robot from the 
        position sensor. Updates the variable self.x_sensed

        Now using a different thread to call the Ascension, which populates the global
        array x_raw_global. It also communicates to the thread by setting the last variable 
        to 1, indicating that it was read. This is used to close the thread after a cntrl-C exit, 
        which normally doesn't kill the daemon threads. 

        Sensor: Ascension
        Units: mm and deg
        """
        self.x_raw, self.quality = self.position_sensor.read_pose_sensor()
        self.x_sensed = filter_data(self.x_raw, 
                                    self.x_filter, 
                                    sensor_freq = self.sensor_freq, 
                                    filter_order = self.filter_order).copy()

    def readSecondPoseSensor(self):
        """
        Reads the current the position and orientation of the second sensor - used
        for manually guiding the trajectory -  from the 
        position sensor. Updates the variable self.x_guide
        Sensor: Ascension
        Units: mm and deg
        """
        self.x_guide_raw, self.quality_guide = self.position_sensor.read_second_sensor()
        self.x_guide = filter_data(self.x_guide_raw, 
                                   self.x_guide_filter, 
                                   sensor_freq = self.sensor_freq, 
                                   filter_order = self.filter_order).copy()

    def readForceSensor(self):
        """
        returns a numpy array of the xyz forces and torques
        Needs to get forces from the low level kinematic model
        Sensor: ATI mini40
        Units: N and Nm
        """
        self.force_raw      = force_sensor.readForceSensor()
        self.force_sensed   = filter_data(self.force_raw, 
                                          self.force_filter, 
                                          sensor_freq = self.sensor_freq, 
                                          filter_order = self.filter_order).copy()
        self.P, self.P_free = force_sensor.find_contact_frame(self.force_sensed)

    def read_base_angles(self):
        ''' gets the true model angles, for records 
        in real robot, this is the orientation of the second sensor taped to the cath
        in simulation, this is the known rotation
        x_guide is being filtered already, so no need to filter again'''
        if self.global_variables:
            self.angles_model    = trig.aer_to_abc(self.x_guide[3:])# grabbing azimuth, elevation, roll
            self.angles_model[2] -= (self.x_guide_initial[5] * np.pi / 180) # roll is relative
            self.R_model         = trig.R_zyz(self.angles_model)
        else: 
            self.R_model, self.angles_model = self.robot_com.getModelRotation()

    def sensorUpdate(self):
        ''' grabs a pose reading, updates the history array. also grabbing force reading'''
        self.readPoseSensor()
        self.readSecondPoseSensor()
        self.read_base_angles()
        # self.readForceSensor()
        self.updateHistory() #assumes pose/force sensors update faster than other variables in history_array 
        if self.global_variables:
            self.x_global[:self.x_num_full] = self.x_sensed[:self.x_num_full]
            self.x_global[self.x_num_full]  = self.quality
            self.force_global[:]            = self.force_sensed[:3]

    def aurisUpdate(self):
        ''' reads motor positions, motor current draws, posts to global variables'''
        self.readMotorPositions()
        self.readMotorAmps()
        if self.global_variables:
            self.q_global[:]  = self.q

########## useful function ######################        
    def spinWait(self, waitTimeSec, update = True):
        """ Burn CPU to make more accurate timing. wait time in seconds
        allows sensor updates or not"""
        start_time = time.perf_counter()
        time_sensors = time.perf_counter()
        time_auris = time.perf_counter()
        while time.perf_counter() - start_time < waitTimeSec: 
            if (time.perf_counter() - time_sensors > 1. / self.sensor_freq) and update:
                self.sensorUpdate()
                time_sensors = time.perf_counter()
            if (time.perf_counter() - time_auris > 1. / self.twincat_freq) and update:
                self.aurisUpdate()
                time_auris = time.perf_counter()

    def get_averaged_array(self, i_start, i_end, array_index = 12):
        '''
        slices out an array from the history array, over the course of spin wait
        returns average and std of each element in array
        12 corresponds to filtered current
        typically run after a spinwait
        '''

        array_slice = [self.history_array[i][array_index] for i in range(i_start, i_end)]
        array_slice = np.asarray(array_slice)
        return np.mean(array_slice, axis = 0), np.std(array_slice, axis = 0)
        
 
######## Force + Position ######################
    '''
    When robot is moving into contact with surface (force'*dx < 0), hybrid force/pos 
    control for MLC separates position control into uncontstrained plane (I-P), while 
    force in P is controlled by scaling the dx_desired in that direction accordingly.
    '''
    def updateDxWithForce(self, dx):
        # if self.force_sensed[:self.force_num].dot(dx[:self.force_num]) < 0: # moving into contact
        if self.force_sensed[2] * dx[2] > 0: # moving into contact
            dx = self.P_free.dot(dx)
            dx[:self.force_num] += self.P.dot(self.getDxForce())
        return dx

    ''' finds the noticeable forces and adds the force control with gain, returns dx_force'''
    def getDxForce(self):
        force_norm   = np.sqrt(self.force_sensed[:self.force_num].dot(self.force_sensed[:self.force_num]))
        dF_magnitude = self.gain_force * (self.force_ref - force_norm) 
        uF       = self.force_sensed[:self.force_num] / force_norm
        dx_force = - (1 / self.K_result) * dF_magnitude * uF # dx = -F/k 
        dx_force = - (1 / self.K_result) * self.gain_force * (self.force_ref - self.force_sensed[2]) * np.array([0, 0, -1]) # dx = -F/k only in Z direction
        return dx_force

################### XBOX updates ###################################
    '''
    Allows the xbox buttons to change modes mid run, 
    toggles flags and reconfigures the state to handle changes
    in variable lengths
    '''
    def check_xbox_controller(self):
        delay = 0.25 # seconds to wait after pressing a button, prevents repeat hits
        if (self.xbox.read_controller("X")): # End Loop
            self.end_loop = True

        if (self.xbox.read_controller("A")): # Go to home position
            # relaxes tendons only
            q_relax = self.q_initial.copy()
            q_relax[-2:] = self.q_desired[-2:]
            move_loop(self, q_relax)
            self.joy_past = list(np.zeros(5))

        if (self.xbox.read_controller("B")): # Re-initialize J
            initialize_jacobian_estimate(self)

        if self.xbox.read_controller("START"): # auris model
            self.model_flag = not self.model_flag
            self.update_flags()
            self.reinitialize()
            self.spinWait(delay)

        if self.xbox.read_controller("DUP"):
            set_trajectory(self, 'real lung', steps = 1, cycles = 1, steps_to_ins = 1, global_coordinates = True)
            self.velocity = 5. 
            self.spinWait(delay)

        if self.xbox.read_controller("DRIGHT"):
            set_trajectory(self, 'square', steps = 5, cycles = 1, steps_to_ins = 10)
            # set_trajectory(self, 'circle', steps = 30, cycles = 1, steps_to_ins = 10)
            self.velocity = 5. # slowing down speed for trajectories
            self.spinWait(delay)

        if self.xbox.read_controller("DLEFT"):
            self.use_guide_sensor = not self.use_guide_sensor
            self.spinWait(delay)

        if self.xbox.read_controller("DDWN"): 
            end_trajectory(self)

        if self.xbox.read_controller("Y"): 
            run_tensioning(self)
            # get_robot_stiffness(self) # initial guess seems good enough
            self.spinWait(delay)

        # if self.xbox.read_controller("STICKPRESSL"): # use shooter mapping
        #     self.use_shooter = not self.use_shooter
        #     self.spinWait(delay)

        # self.reset_flag = False
        # if self.xbox.read_controller("RIGHTB"): # reset score
        #     self.reset_flag = True            

        # if self.xbox.read_controller("LEFTB"): # XYZ/AEX
        #     self.xyz_flag = not self.xyz_flag
        #     self.spinWait(delay)

        
        # if self.xbox.read_controller("BACK"): # redundancy
        #     self.use_leader_and_sheath = not self.use_leader_and_sheath
        #     self.reinitialize()
        #     self.spinWait(delay)

        # adjusting flag relationships
    def update_flags(self):
        if self.model_flag:
            self.use_leader_and_sheath = True
            self.use_orientation = True
            self.vision_flag  = False
            self.velocity = 20.
        elif self.vision_flag:
            self.use_orientation = False
            self.model_flag = False
            self.velocity = 1
        else:
            self.use_leader_and_sheath = False
            self.use_orientation = False
            self.vision_flag  = False
            self.velocity = 10.
        # communicate flags to the gui
        if self.global_variables:
            ascension   = 1 if (not self.vision_flag and not self.model_flag) else 0
            vision      = 1 if (self.vision_flag and not self.model_flag) else 0
            model       = 1 if self.model_flag else 0
            reduced     = 1 if self.reduced_flag else 0
            orientation = 1 if self.use_orientation else 0
            reset       = 1 if self.reset_flag else 0
            xyz         = 1 if self.xyz_flag else 0
            shooter     = 1 if self.use_shooter else 0
            redundant   = 1 if self.use_leader_and_sheath else 0
            self.flags_global[:] = [ascension, vision, model, reduced, orientation,
                                    reset, xyz, shooter, redundant]

     