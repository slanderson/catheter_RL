'''
initialize_control.py

begins a robot run by 
- initializing the robot communication
- starting threads for pose sensor, camera, opengl, gui
- initializing robot state
- starting control loop

J. Sganga 8/20/2016
'''


import sys
import time
import numpy as np
import multiprocessing
from robot.robot_state import robot_state
from robot.control.control_loop import run_loop, test_loop
from robot.private.initialize_robot_communication import initialize_system
from robot.devices import CameraInterface
from robot.devices.position_sensor import position_sensor_thread
from functions import openGL_interface

def beginCamThread(global_variables):
    save_images = False
    camThread = multiprocessing.Process(name='auris camera thread', 
                                        target=CameraInterface.CameraThread, 
                                        args = [global_variables, save_images])
    camThread.daemon = True
    camThread.start()

def beginOpenGLThread(global_variables):
    oglThread = multiprocessing.Process(name='OpenGL Render Thread', target=openGL_interface.OGLThread, args = [global_variables])
    oglThread.daemon = True
    oglThread.start()

def beginPositionThread(global_variables):
    posThread = multiprocessing.Process(name='Pose Tracker Thread', target=position_sensor_thread.PoseThread, args = [global_variables])
    posThread.daemon = True
    posThread.start()

def makeGlobalVariables():
    dx_global = multiprocessing.Array('d', 5)
    J_global  = multiprocessing.Array('d', 20)
    q_global  = multiprocessing.Array('d', 10)
    x_global  = multiprocessing.Array('d', 7) # 6 DOF pose, 1 quality
    flag_global = multiprocessing.Array('d', 9)
    x_desired_global = multiprocessing.Array('d', 5)
    force_global = multiprocessing.Array('d', 3)
    x_raw_global =  multiprocessing.Array('d', 15) # space for measurements from 2 sensors, and interaction flag
    return [dx_global, J_global, q_global, x_global, flag_global, x_desired_global, force_global, x_raw_global]

def control_robot(use_leader      = True,
                 use_sheath       = False,
                 use_orientation  = False,
                 use_lp_filter    = True, 
                 trajectory       = [],
                 model_estimation = 'mlc', # mlc, mbc, kf, r, ekf, ukf
                 motion_control   = 'jinv', # jinv, tension, tension_cvx, mpc, mpc2, mpc_cvx
                 dx_threshold     = 1.,
                 q_threshold      = 1e-5):
    global_variables = makeGlobalVariables()
    beginPositionThread(global_variables)
    beginCamThread(global_variables)
    # beginOpenGLThread(global_variables)

    robot_com, xbox  = initialize_system()
    robot = robot_state(robot_com           = robot_com,
                        xbox                = xbox, 
                        global_variables    = global_variables, 
                        use_leader          = use_leader, 
                        use_sheath          = use_sheath,
                        use_orientation     = use_orientation,
                        use_low_pass_filter = use_lp_filter,
                        model_estimation    = model_estimation,
                        motion_control      = motion_control,
                        dx_threshold        = dx_threshold,
                        q_threshold         = q_threshold,
                        trajectory          = trajectory)
    run_loop(robot)

def control_test():
    global_variables = makeGlobalVariables()
    beginPositionThread(global_variables)
    beginCamThread(global_variables)
    beginOpenGLThread(global_variables)
    
    robot_com, xbox  = initialize_system()
    robot = robot_state(robot_com, xbox, global_variables)
    test_loop(robot)



