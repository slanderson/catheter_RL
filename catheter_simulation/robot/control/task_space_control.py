'''
task_space.py

this file will manage the trajectory in task space defined by
a preset trajectory or real time control from the xbox or guide sensor 

J. Sganga 8/20/2016 

'''
import time, sys
import numpy as np
from robot.control.trajectories import get_trajectory

def set_dx(robot):
    # updates both x_desired and dx_desired
    if robot.trajectory:
        check_error_margin(robot)
        project_to_workspace(robot, robot.x_desired) 
    elif robot.use_guide_sensor:
        check_guide_sensor(robot)
    else:
        check_joystick(robot)

    if robot.global_variables:
        robot.x_desired_global[:] = robot.x_desired[:5]

def check_error_margin(robot):
    # trajectory controls desired x
    margin_xyz = 2. # mm
    margin_ae  = 5 # deg
    dx = robot.P_free.dot(robot.x_desired[robot.x_list] - robot.x_sensed[robot.x_list])
    error_xyz = max(abs(dx[:3])) # square error bound (what MPC uses)
    error_ae  = 0 
    if robot.use_orientation:
        error_ae = max(abs(dx[3:]))
    if error_xyz < margin_xyz and error_ae < margin_ae:
        robot.x_desired[robot.x_list] = np.asarray(robot.trajectory.pop(0))
        print(robot.x_desired)
        sys.stdout.flush()
        if not robot.trajectory: 
            # need to add last point twice, so it doesn't end prematurely
            end_trajectory(robot)
    robot.dx_desired = robot.x_desired[robot.x_list] - robot.x_sensed[robot.x_list]
    # robot.dx_desired = robot.updateDxWithForce(robot.dx_desired)

def set_trajectory(robot, name_traj, steps = 1, cycles = 1, steps_to_ins = 1, global_coordinates = False):
    '''
    sets up the list of position/orientation commands that the robot has to hit sequentially
    sets the first one as robot.x_desired
    '''
    robot.trajectory = get_trajectory(name_traj, steps, cycles, steps_to_ins)
    if not global_coordinates:
        robot.trajectory = np.asarray(robot.trajectory)
        robot.trajectory = list(robot.trajectory[:,robot.x_list]  + robot.x_sensed[robot.x_list])
    robot.x_desired[robot.x_list] =  np.asarray(robot.trajectory.pop(0))[robot.x_list]

def end_trajectory(robot):
    ''' 
    Allows the user to end a trajectory mid cycle with an xbox command, 
    returning control to the joy stick
    '''
    robot.end_trajectory = True
    robot.trajectory = []
    robot.x_desired = robot.x_sensed.copy()
    print('Finished Trajectory')
    sys.stdout.flush()

def check_guide_sensor(robot):
    robot.readSecondPoseSensor()
    robot.x_desired[robot.x_list] = robot.x_guide[robot.x_list].copy()
    robot.x_desired[0] -= 30 # avoid touching, slight offset
    robot.x_desired[2] += 30
    robot.dx_desired = robot.x_desired[robot.x_list] - robot.x_sensed[robot.x_list]
    robot.dx_desired = robot.updateDxWithForce(robot.dx_desired)

def check_joystick(robot):
    joy = robot.xbox.read_joystick()
    # joy stick controls desired dx
    robot.dx_desired = joy_to_dx(joy)[robot.x_list]
    robot.x_desired[robot.x_list] = robot.x_sensed[robot.x_list] + robot.dx_desired

def joy_to_dx(joy):
    # joy sticks to X, Y, Z commands -  according to X,Y,Z axes of Ascension
    # triggers - X (slide)
    # left joystick - vert = Z, horz = Y
    # right joystick- vert = el, horz = az
    joystick_gain = 0.2
    joyLY, joyLX, joyRY, joyRX, joyT = joy
    return joystick_gain * np.array([-joyT, joyLX, joyLY, joyRX, joyRY])
    
def project_to_workspace(robot, x):
    return x

def get_two_points(robot):
    '''
    grabs the next point waiting in trajectory queue 
    for MPC
    need to project both because as estimate of base angles 
    changes, the projection will too
    '''
    first_way_point = project_to_workspace(robot, robot.x_desired[robot.x_list])
    next_two_x = np.tile(first_way_point, (2,1)).T
    if robot.trajectory:
        next_way_point = np.asarray(robot.trajectory[0])[robot.x_list]
        next_two_x[:,1] = project_to_workspace(robot, next_way_point)
    return next_two_x