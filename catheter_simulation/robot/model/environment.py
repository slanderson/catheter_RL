'''
Author: Jake Sganga
Date: 5/21/15

The ModelEnvironment is used to determine the effects of a simulated anatomy on 
the model (called from ModelInterface.py). To begin, this will define the 
lung/bronchoscopy task by assigning a global model rotation to the catheter
depending on it's state.

'''
import sys, time
import numpy as np
import functions.trig as trig

class environment(object):
    """environment that the catheter model exists in"""
    def __init__(self, bend_constraint = np.pi, transition_num = 7):
        self.transition_num = transition_num
        self.angle_list = np.array([
                        [0, np.pi / 8, np.pi / 6, np.pi / 4,  np.pi / 3, np.pi / 2, np.pi / 1.5], # yaw, z
                        [0,         0, np.pi / 10, np.pi / 8, np.pi / 6, np.pi / 5, np.pi / 4],   # pitch, y
                        [0,         0,         0,         0,          0,         0,         0]])
        self.transition_points = np.array([5, 10, 15, 25, 40, 45, 50])
        self.dt = self.transition_points.copy()
        self.dt[1:] -= self.transition_points[:-1] 
        self.bend_constraint = bend_constraint
        self.tube_start = 1
        self.tube_end = 40


    def getEnvRotation(self, q):
        '''
        Defines continuous function as a function of q_ins for rotation
        For R_zyz
        alpha (+z), beta (+y_relative), gamma (+z_relative_relative))
        '''
        q_ins = q[8] 
        # 2 sections making an S
        q_s1_start = 10
        q_s1_end   = 53 # specific value trying to match previous formulation
        q_s2_start = 60
        q_s2_end   = 100

        #before first bend:
        alpha = 0
        beta  = np.pi / 2
        gamma = 0

        # bends:
        angle_s1 = np.pi / 4
        angle_s2 = angle_s1

        # section 1:
        if q_ins > q_s1_start and q_ins <= q_s1_end:
            alpha = angle_s1 * ((q_ins - q_s1_start) / (q_s1_end - q_s1_start))**2
        elif q_ins > q_s1_end and q_ins <= q_s2_start:
            alpha = angle_s1
        # section 2:
        elif q_ins > q_s2_start and q_ins <= q_s2_end:
            alpha = angle_s2 - angle_s1 * (q_ins - q_s2_start) / (q_s2_end - q_s2_start)
        else:
            alpha = 0
        # gamma = -alpha
        angles = np.array([alpha, beta, gamma])
        return trig.R_zyz(angles), angles

        # # old trajectory the caused MLC to choke
        # if q_ins < self.transition_points[0]:
        #     return trig.R_zyx(self.angle_list[:,0]).dot(trig.R_zyz([0,np.pi/2,0])), self.angle_list[:,0]

        # for i in range(1, self.transition_num):
        #     if q_ins < self.transition_points[i]:
        #         alpha  = (self.transition_points[i] - q_ins) / self.dt[i]
        #         angles = alpha * self.angle_list[:,i-1] + (1 - alpha) * self.angle_list[:,i]
        #         return trig.R_zyx(angles).dot(trig.R_zyz([0,np.pi/2,0])), angles

        # return trig.R_zyx(self.angle_list[:,-1]).dot(trig.R_zyz([0,np.pi/2,0])), self.angle_list[:,-1]

        
    def getEnvTrajectory(self):
        '''
        Defines trajectory for lung task based on the the rotations 
        Don't have to be related, but it helps make the task more realistic

        trajectory points need to hit before the rotations because trajectory is 
        for tip position, while rotation is for the base of the articulating region
        '''
        #straight line
        goal = 30
        start_point = np.array([0, 0, 0, 0, 0])
        end_point = np.array([goal, goal, 0, 0, 0])
        trajectory = [start_point, end_point]

        # curve then jump
        trajectory = [
            np.array([10, 0, 0, 0, 0]), 
            # np.array([22.5, 5, 0, 0, 0]),
            # np.array([30, 10, 0, 0, 0]),
            # np.array([33, 15, 0, 0, 0]),
            # np.array([35, 20, 0, 0, 0]),
            # np.array([35, 25, 0, 0, 0]),
            np.array([35, 35, 0, 0, 0]),
            np.array([45, 35, 0, 0, 0])]

        # # original trajectory for lung
        # trajectory = []
        # traj_x     = 0
        # traj_y     = 0
        # traj_z     = 0
        # ins_gear_ratio = 1

        # for i, tp in enumerate(self.dt):
        #     traj_x += ins_gear_ratio * tp * np.cos(self.angle_list[0,i]) * np.cos(self.angle_list[1,i])
        #     traj_y -= ins_gear_ratio * tp * np.sin(self.angle_list[0,i]) * np.cos(self.angle_list[1,i]) #not sure why the neg
        #     traj_z += ins_gear_ratio * tp * np.sin(self.angle_list[1,i])
        #     trajectory.append(np.array([traj_x, traj_y, traj_z, self.angle_list[0,i], self.angle_list[1,i]]))

        return trajectory

    def constrainConfiguration(self, curve, q_ins, touch_wall = False):
        '''
        limits the motion of the cath depending on the configuration.
        Returns the constrained configuration, which will in turn dictate
        how the the tip position is read. The tendon positions are unaffected
        by the constraint, mimicking pulling into an unsensed wall.
        '''
        # return self.bend_constraint
        s, beta, phi = curve
        beta_open = self.bend_constraint
        beta_wall = 0

        if touch_wall and q_ins > 0:
            print('hitting wall...turn off for real runs')
            beta = beta_open
            phi  = np.pi / 2
            return s, beta, phi
            
        if q_ins < self.tube_start: # allow j initialization
            return s, beta, phi

        if phi > 0 and phi < np.pi:
            beta = min(beta, beta_open)
        elif q_ins < self.tube_end:
            beta = min(beta, beta_wall)

        return s, beta, phi

