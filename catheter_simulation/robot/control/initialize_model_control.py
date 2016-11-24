'''
initialize_model_control.py mimics initialize_control,
separated to skip  withouth the extra modules

J. Sganga 8/22/2016
'''
import numpy as np
from robot.model.model_communication import RobotModelInterface
from robot.robot_state import robot_state
from robot.control.control_loop import run_model_loop
from robot.control.motion_control import test_model_moves

def control_model(use_leader      = True,
                 use_sheath       = False,
                 use_orientation  = False,
                 sensor_noise     = [0,0,0], # standard dev for position, angle, force sensors
                 use_obstacle     = False,
                 use_heartbeat    = False,
                 use_lp_filter    = True,
                 trajectory       = 'lung',
                 touch_wall       = False,
                 bend_constraint  = np.pi,
                 model_estimation = 'mlc',
                 motion_control   = 'mpc',
                 dx_threshold     = 1.,
                 q_threshold      = 1e-5):
    # initializes the the low level kinematic model (formerly g_PyAuris)
    use_lung_task = True if trajectory == 'lung' else False
    robot_com = RobotModelInterface(use_leader      = use_leader, 
                                    use_sheath      = use_sheath,
                                    sensor_noise    = sensor_noise,
                                    use_obstacle    = use_obstacle,
                                    use_heartbeat   = use_heartbeat,
                                    use_lung_task   = use_lung_task,
                                    bend_constraint = bend_constraint,
                                    touch_wall      = touch_wall)
    robot = robot_state(robot_com           = robot_com,
                        xbox                = [], 
                        global_variables    = [], 
                        use_leader          = use_leader, 
                        use_sheath          = use_sheath,
                        use_orientation     = use_orientation,
                        use_low_pass_filter = use_lp_filter,
                        model_estimation    = model_estimation,
                        motion_control      = motion_control,
                        dx_threshold        = dx_threshold,
                        q_threshold         = q_threshold,
                        trajectory          = trajectory)

    run_model_loop(robot)    # runs normal task loop
    # test_model_moves(robot)    # test specific q moves
