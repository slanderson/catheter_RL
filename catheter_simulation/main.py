import sys
import getopt
import numpy as np
'''
J.Sganga 8/19/16

main.py can initiate loops on the Auris robot or the simulated robot. 

The robot_model project tries to match the structure and functions of the
robot project as closely as reasonably possible. The goal is for seamless
integration of the control techniques developed on the simulation into 
real world implementation on the robot. When possible, the code will be 
flexible enough to allow for varying degrees of real-ness to improve 
speed/efficiency/ease of use. For example, ModelLowLevel is created so that
the commands from IDM_model match those of IDM. On the other hand, multiprocessing
is not used because handling the camera and opengl threads are ignored. 


Layers of abstraction  (robot parallels)                                                      
- main.py         (main)                                                          
    - run_model.py     (run_robot)                                                           
        - IDM_model.py        (IDM)                                                                
            - ModelLowLevel.py     (AurisLowLevel, commands to/from robot through AurisInterface.pyd)                                                     
                - ModelInterface.py (AurisInterface.pyd, mimics (bad) physics of real world)   

Key control points:
main - control options shown below
run_model - can change the trajectory (also in functions/trajectories.py), do printing
idm - change control functions
modellowlevel - expose variables to/from kinematics
modelinterface - change underlying simulation, noise, obstacles, etc
modelenvironment - made for lung task, defines the track of rotation matrices
                            
'''
def main():
    use_robot = False
    if use_robot:
        from robot.control.initialize_control import control_robot, control_test
        # control_test()
        # paramters can be changed in real time with the xbox controller
        control_robot(use_leader      = True,
                     use_sheath       = False,
                     use_orientation  = False,
                     use_lp_filter    = True, 
                     trajectory       = [],
                     model_estimation = 'ukf', # mlc, mbc, kf, r, ekf, ukf, abc
                     motion_control   = 'jinv', # jinv, tension, tension_cvx, mpc, mpc2, mpc_cvx, joy
                     dx_threshold     = 5.,
                     q_threshold      = 1e-5)   
    else:
        from robot.control.initialize_model_control import control_model
        control_model(use_leader      = True,
                     use_sheath       = False,
                     use_orientation  = False,
                     sensor_noise     = [0,0,0], # mm, rad, N
                     use_obstacle     = False,
                     use_lp_filter    = True, 
                     trajectory       = 'square',
                     touch_wall       = False, # remeber to turn off when running task!
                     bend_constraint  = np.pi/3,
                     model_estimation = 'mbc', # mlc, mbc, kf, r, ekf, ukf, angles
                     motion_control   = 'jinv', # jinv, tension, tension_cvx, mpc, mpc2, mpc_cvx
                     dx_threshold     = 25.,  # (delta movement in mm)^2
                     q_threshold      = 1e-6) # (delta motor in mm)^2

if __name__ == '__main__':
  main()
