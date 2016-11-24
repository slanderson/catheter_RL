'''
 Author: Jake Sganga
Date: 10/21/15

The ModelInterface script should be imported by ModelLowLevel to mimic AurisInterface 
This will allow for the most seamless implementation of model-tested algorithms on the robot itself. 
For that reason, the classes and functions should match the names and expected behavior of AurisInterface.


Fucntions to mimic:
GetDesiredPosition()
SetDesiredPosition(s1, s2, s3, s4, l1, l2, l3, l4, i1, i2)
GetActualPosition()
get_position(write to file)

'''
import sys, time
import numpy as np
from robot.model.environment import environment 
import functions.trig as trig

'''
File defines the settings for both the leader and sheath classes. Will be called by ModelInterface 
and maybe the plotting file to define the settings for the catheters.

Jake Sganga
10/27/15

note: made it one class that includes the sheath if option set
10/28

Here is where the simulated foward kinematics will be calculated
'''

class catheter():
    """leader of auris robot, defines sizes and other settings, everything in mm, rad, N and Nm!!!"""
    def __init__(self, use_leader       = True, 
                       use_sheath       = False,
                       sensor_noise     = [0,0,0], 
                       use_obstacle     = False,
                       use_heartbeat    = False,
                       use_lung_task    = False,
                       bend_constraint  = np.pi,
                       touch_wall       = False):
        
        self.use_leader = use_leader
        self.use_sheath = use_sheath
        # Default parameters
        # parameter tuples in [sheath, leader] format
        diameters     = [5.50, 4.25] # mm
        lengths       = [40, 35] # mm
        motor_scales  = np.array([0.4,   0.4,  0.4,  0.4, # sheath
                                  0.23,  0.28, 0.23,  0.28, # leader
                                  1.5,  1.0])              # linear motors
        #wire placement about cath axis (Z) from X = 0
        thetas   = np.array([-1.50, 0.05,  2.00,  3.25, #sheath (not tested yet)
                             -1.80,-0.15,  1.45,  3.10]) # leader
        # ground frame orientation (R_zyz, Euler angles)
        R_angles = np.array([0.075, np.pi/2 - 0.075, 0])

        self.setParameters(R_angles, thetas, motor_scales,  lengths, diameters)
        self.straight_length = 500
        self.tip_length = 9 #rigid length extending beyond leader

        self.q_num = 10 # 4 tendons + insertion for leader and sheath
        self.q     = np.zeros(self.q_num)
        self.q[-1] = self.length_leader if use_leader else 0
        self.q_desired = self.q.copy()

        self.x_num = 6
        self.x     = np.zeros(self.x_num) 
        self.x_desired = np.zeros(self.x_num)

        self.forces = np.zeros(6)
        self.K_env = 20 / 1000 # xyz stiffness, N/mm = mN/mm /1000, reguluate to ~40mN
        self.K = np.zeros((self.q_num, self.q_num)) # 4 tendons each, but doing full
        self.setStiffness()

        self.noise_position = sensor_noise[0]
        self.noise_angle    = sensor_noise[1]
        self.noise_force    = sensor_noise[2]

        self.use_obstacle   = use_obstacle  # adds virtual wall
        self.use_heartbeat  = use_heartbeat # not yet implemented
        self.use_lung_task  = use_lung_task
        self.lung_env       = environment(bend_constraint = bend_constraint)

        # Accounting for lung task
        self.bend_constraint = bend_constraint # [0, pi] limits beta for constrained task 
        self.base_env = self.x[:3].copy()
        self.base_env[0] -= self.length_sheath + self.q[-1] # setting base behind the starting tip pos
        self.q_ins_previous = 0 
        self.x_robot_frame = np.zeros(self.x_num)
        self.touch_wall = touch_wall # when marking position of inner wall

        # Sets position
        self.x_ground_frame          = self.x[:3].copy()
        self.setDesiredPosition(self.q) # should set correct self.x
        self.x_ground_frame_previous = self.x_ground_frame.copy()
        self.x_robot_frame_previous  = self.x_robot_frame.copy()

        
    def setParameters(self, 
                      angles,
                      thetas,
                      motor_scales, 
                      lengths,
                      diameters):
        ''' sets the parameters of the catheter '''
        self.R_angles     = angles
        self.R_env        = trig.R_zyz(angles)
        self.theta_sheath = thetas[:4]
        self.theta_leader = thetas[4:]
        self.motor_scales = motor_scales
        self.length_sheath, self.length_leader = lengths
        self.diameter_sheath, self.diameter_leader = diameters


    def getDesiredPosition(self):
        """ Returns the position we have set on each axis. """
        return self.q_desired.copy()

    def setDesiredPosition(self, q_desired):
        """ Sets the position we have set on each axis, and applies change to the model"""
        self.q_desired = q_desired.copy()
        self.q_ins_previous = self.q[8].copy()
        self.q = self.scaleGearRatios(self.q_desired.copy()) # will change orig wout copy
        self.updatePosition()
        if self.use_obstacle:
            self.updateForces()

    def scaleGearRatios(self, q):
        q *= self.motor_scales
        return q

    def getActualPosition(self):
        """ Returns the motor position of each axis. Using q_desired, so that self.q can be scaled according
        to experimental results."""
        return self.q.copy()

    def getCoordinatePosition(self):
        """ Returns the xyzaer positions. Controller is much less effective when in degrees
        Elevation = -pi/2 + beta
        """
        x = self.x.copy() # don't want to keep adding noise if check repeatedly
        if self.noise_position:
            x[:3] +=  np.random.normal(0, self.noise_position, 3)
        if self.noise_angle:
            x[3:] +=  np.random.normal(0, self.noise_angle, 3) 
        x[4] = -np.pi / 2 + x[4] # converts beta to elevation, which is what the Ascension gives
        return x

    def updatePosition(self):
        '''
        Finds the tip position and orientation in the ground frame.
        Trasforms robot frame by the enviroment-caused rotation matrices
        dx is rotated, otherwise it rotates the entire model about the origin
        R_env takes robot frame and brings it to ground frame
        '''
        # self.x_robot_frame_previous  = self.x_robot_frame.copy()
        self.x_ground_frame_previous = self.x_ground_frame.copy()
        self.x_robot_frame, T        = self.updateRobotFramePosition(self.q, self.use_lung_task)
        if self.use_lung_task:
            self.R_env, self.R_angles = self.lung_env.getEnvRotation(self.q.copy())
        self.x_ground_frame = self.R_env.dot(self.x_robot_frame[:3])
        dx = self.x_ground_frame - self.x_ground_frame_previous
        dx += self.R_env[:,2] * (self.q[8] - self.q_ins_previous) # insertion not accounted for in updateRobotFramePosition
        self.x[:3] += dx
        self.x[3:] = trig.getAnglesZYZ(self.R_env.dot(T[:3,:3]))

    def updateRobotFramePosition(self, q, use_lung_task):
        '''
        Based on kinematic model from Walker (2006)
        Adapted to 4 tendons

        Need to extend leader from tip of sheath, so want to make a transformation 
        matrix that is the product of T(sheath -> ground) and T(leader -> sheath)

        Remeber ordering of motors:
        s1, s2, s3, s4, l1, l2, l3, l4, i1, i2
        '''
        T = np.eye(4)
        if self.use_sheath:
            curve_sheath  = self.getCurvature(q[0:4], 
                                              self.theta_sheath, 
                                              self.diameter_sheath, 
                                              self.length_sheath)
            if use_lung_task:
                curve_sheath = self.lung_env.constrainConfiguration(curve_sheath, q[8], self.touch_wall)
            # straight length addressed in updatePosition
            x_sheath = self.getPositionFromCurve(curve_sheath, 0) 
            T        = self.getTfromX(x_sheath)

        if self.use_leader and q[-1] > 0: # leader is extended
            curve_leader = self.getCurvature(q[4:8], 
                                             self.theta_leader, 
                                             self.diameter_leader, 
                                             min(q[9], self.length_leader))
            if use_lung_task:
                curve_leader = self.lung_env.constrainConfiguration(curve_leader, q[8], self.touch_wall)
            x_leader  = self.getPositionFromCurve(curve_leader, max(0, q[9] - self.length_leader))
            T_leader  = self.getTfromX(x_leader)
            T_tip        = np.eye(4)
            T_tip[:3,3]  = [0, 0, self.tip_length] # 1cm rigid extension to pose sensor 
            T         = T.dot(T_leader.dot(T_tip))
            
        x_robot_frame = self.recoverXfromT(T)
        return x_robot_frame, T


    def getCurvature(self, q, theta, diameter, length):
        '''
        Assumes that q[8] = total insertion, q[9] = length that leader extends
        Original formulation assumed insertion along Z, phi = 0 along x-axis (rotation about +Z)
        Will formulate it that way then make the rotation later
        R = [0 0 1; 0 1 0; -1 0 0]
        so theta = 0 assumes +x, theta's rotate about Z marking the position of the wires
        '''
        dq_x = np.cos(theta).dot(q)
        dq_y = np.sin(theta).dot(q)
        s = float(length) 
        beta = (1 / diameter) *  np.sqrt(dq_x**2 + dq_y**2)
        # kappa = beta / float(s)
        phi = np.arctan2(dq_y, dq_x) 
        return [s, beta, phi]

    def get_leader_curvature(self, q_tendons):
        '''
        returns the curve parameters for the leader.
        allows direct access to the curvature parameters
        input = 4 displacements of the leader's tendons
        '''
        q_scaled = self.motor_scales[4:8] * q_tendons.copy()
        curve_leader = self.getCurvature(q_scaled, self.theta_leader, self.diameter_leader, self.length_leader)
        return curve_leader


    def getPositionFromCurve(self, curve,  straight_length = 0):
        '''
        Again assumes +Z is insertion!!, will fix in later function
        ignoring roll for now because it isn't an actuated DOF
        seems like straight length could be different to account for bends
        '''
        s, beta, phi = curve
        beta += 1e-9 # Avoids division by 0, never negative
        x = np.cos(phi) * (s / beta) * (1. - np.cos(beta))
        y = np.sin(phi) * (s / beta) * (1. - np.cos(beta))
        z = (s / beta) * np.sin(beta) + straight_length 
        alpha = phi 
        gamma = -alpha
        return np.asarray([x, y, z, alpha, beta, gamma])

    def getTfromX(self, X):
        '''
        The angles can be described by the rotation
        matrices Rz(alpha)Ry(beta)Rz(-azimuth) according to a intrinsic rotation
        (first about Z by azimuth, then about the resulting -Y axis by elevation, then correcting
        for the roll introduced) - called R_zyz(alhpa, beta, gamma) where gamma = -alpha 
        Note still using radians
        '''
        x, y, z, alpha, beta, gamma = X
        R = trig.R_zyz([alpha, beta, gamma])
        d = np.array([x, y, z])
        T = np.eye(4, dtype = float)
        T[:3,:3] = R[:]
        T[:3, 3] = d 
        return T

    def recoverXfromT(self, T):
        '''
        This function will extract the x, y, z, azimuth, elevation in the corrected
        coordinate frame (x = insertion) to match the ascension setup
        also makes the azimuth term more stable for most of the workspace
        '''
        x, y, z = T[:3,3]
        alpha, beta, gamma = trig.getAnglesZYZ(T[:3,:3])
        return np.array([x, y, z, alpha, beta, gamma])

    def getPointsAlongBody(self, q, num = 10):

        '''
        This function was added 3/7/2016, J. Sganga, to enable plotting of the cathter body after a simulation
        is run. It mimics update position, except the arc length, s, is allowed to take num equally spaced lengths.
        The function will return a 2*num x 6 array (x,y,z,a,b,c of each point for leader and sheath)

        Not plotting straight sections, yet
        '''
        q = self.scaleGearRatios(q)
        sheath_points   = np.zeros((num, self.x_num))
        arc_length = np.linspace(0, self.length, num)
        for i, al in enumerate(arc_length):
            curve   = self.getCurvature(q[0:4] * al / self.length, al)
            x       = self.getPositionFromCurve(curve, q[8])
            T       = self.getTfromX(x)
            # T[:3,:] = trig.R_reorient.dot(T[:3,:])
            x = self.recoverXfromT(T)
            sheath_points[i,:] = x
        body_points = sheath_points.copy()
        if q[9] > 0:
            leader_points   = np.zeros((num, self.x_num))
            arc        = min(q[9], self.length)
            arc_length = np.linspace(0, arc, num)
            for i, al in enumerate(arc_length):
                curve_leader = self.getCurvature(q[4:8]* al / arc, al, leader_flag = True) # don't want to assume it keeps articulating
                x_leader     = self.getPositionFromCurve(curve_leader, max(0, q[9] - self.length))
                T_leader     = self.getTfromX(x_leader)
                T_tip        = np.eye(4)
                T_tip[:3,3]  = [0, 0, 10] # 1cm rigid extension to pose sensor 
                T_leader     = T.dot(T_leader.dot(T_tip)) # last T from sheath
                x = self.recoverXfromT(T_leader)
                x[:3] = self.R_env.dot(x[:3])
                x[3:] = trig.getAnglesZYZ(self.R_env.dot(T_leader[:3,:3]))
                leader_points[i,:] = x
            body_points = np.vstack((sheath_points, leader_points))
        return body_points

    def updateForces(self):
        ''' Sets up a virtual wall with a stiffness based on the series combination 
        of the wall stiffness and the manipulator stiffness (guess). Only looking at
        xyz forces, not torques for now'''
        x_wall = -7
        ix = 2 
        dx = self.x[ix] - x_wall
        # not quite sure how to handle angle walls
        if dx / x_wall > 0: # contact for moving more negative than negative wall and more positive than positive wall
            self.forces[ix] = - self.K_env * dx  # N/mm * mm 
        else:
            self.forces[:3] *= 0  

    def getForces(self):
        ''' Returns the forces at the tip of the manipulator. xyz in N, torque xyz in Nm to 
        match the ATI Mini40. calculated based on an assumed stiffness of the enviromnemnt and 
        catheter (test against rigid heart anatomy means it will be very stiff). Going to need to 
        separate the actual configuration from the '''
        if self.noise_force:
            return self.forces.copy() + np.random.normal(0, self.noise_force, 6)
        else:
            return self.forces.copy()

    def setStiffness(self):
        '''
        Uses Camarillo's mech model for stiffness, arbitrary stiffnesses, not super stable...
        '''
        D = np.array([
            [-2, 0, 2, 0],
            [0, -2, 0, 2],
            [1,  1,  1, 1],
            [1,  1,  1, 1]
            ])
        Kb = 1e2
        Ka = 1e4
        Kt = 1e6
        Km = np.diag([Kb, Kb, Ka, Ka])
        Km_inv = np.diag([1/Kb, 1/Kb, 1/Ka, 1/Ka])
        lb = self.length_leader # mm, length of articulating region 
        la = lb
        L0 = np.diag([lb, lb, la, la])
        Cm = D.T.dot(L0.dot(Km_inv.dot(D))) + (1/Kt) * np.eye(4)
        Cm_inv = np.linalg.inv(Cm)
        self.K[:4,:4]   = Cm_inv
        self.K[4:8,4:8] = Cm_inv

        #skipping davids
        self.K = np.eye(self.q_num)
        primary = 2e-2
        pair    = 1e-3
        sides   = -1e-4
        K_leader = np.array([
                            [primary,   sides,    pair,   sides],
                            [  sides, primary,   sides,    pair],
                            [   pair,   sides, primary,   sides],
                            [  sides,    pair,   sides, primary]])
        self.K[:4,:4] = K_leader
        if self.q_num > 5:
            self.K[4:8,4:8] = K_leader

        self.K[-2:,-2:] *= 0# inserstions have no tension regardless of current reading


    def getStiffness(self):
        return self.K.copy()

    def get_leader_jacobian(self, q, curve = []):
        '''
        returns the 6 x 5 jacobian for the leader segment
        allows input of curve belief (s, theta (beta), phi)
        '''
        q_scaled = self.scaleGearRatios(q.copy())
        if not len(curve):
            curve = self.getCurvature(q_scaled[4:8], self.theta_leader, self.diameter_leader, self.length_leader)

        if q[9] < self.length_leader:
            ins_past_articulation = False
        else:
            ins_past_articulation = True

        q_x = np.cos(self.theta_leader).dot(q_scaled[4:8])
        q_y = np.sin(self.theta_leader).dot(q_scaled[4:8])

        J_scaled = self.getModelJacobian(self.theta_leader, q_x, q_y, curve, self.diameter_leader, ins_past_articulation)
        J = J_scaled * self.motor_scales[4:9]
        return J


    def getModelJacobian(self, theta_tendons, q_x, q_y, curve, diameter, ins_past_articulation = True):
        '''
        returns the model-based jacobian for a given q
        dx/dq = dx/dcurve * dcurve/dqr * dqr/dq
        '''
        dqxy_dq     = self.get_dqxy_dq(theta_tendons)
        dcurve_dqxy = self.get_dcurve_dqxy(q_x, q_y, curve, ins_past_articulation, diameter)
        dx_dcurve   = self.get_dx_dcurve(curve)
        return dx_dcurve.dot(dcurve_dqxy.dot(dqxy_dq))

    def get_dqxy_dq(self, theta_tendons):
        '''
        jacobian based on deriving dq_x and dq_y from the tendon displacements, q.
             dq[0] .... dq[4]
        dq_x
        dq_y
        dq_ins
         - valid for single segment only  
        '''
        dqxy_dq = np.zeros((3, 5))
        dqxy_dq[0,:4]  = np.cos(theta_tendons) # dqx
        dqxy_dq[1,:4]  = np.sin(theta_tendons) # dqy
        dqxy_dq[-1,-1] = 1.                    # dins
        return dqxy_dq

    def get_dcurve_dqxy(self, q_x, q_y, curve, ins_past_articulation, diameter):
        '''
        jacobian of change in curve parameters [s, theta (beta before), phi] 
        as a func of dq_x and dq_y
            dq_x, dq_y, d_ins
        d_s
        d_theta
        d_phi
        d_ins

        d_s row is all zeros if the leader is past the distal edge of the sheath, 
        always zeros for sheath
        '''
        epsilon = 1e-9 # to avoid division by zero
        s, theta, phi = curve
        theta += epsilon # theta never negative, prevents division by zero
        d_theta_denom = diameter**2 * theta
        d_phi_denom   = q_x**2 + q_y**2 + epsilon

        d_s = np.zeros(3)
        if not ins_past_articulation:
            d_s[-1] = 1.
        d_theta = np.array([q_x / d_theta_denom, q_y / d_theta_denom, 0])
        d_phi   = np.array([-q_y / d_phi_denom, q_x / d_phi_denom, 0])
        d_ins   = np.zeros(3)
        d_ins[-1] = 1.

        return np.array([d_s, d_theta, d_phi, d_ins])

    def get_dx_dcurve(self, curve):
        '''
        jacobian for d(x,y,z,a,b,c) as a func of d_curve
            d_s, d_theta, d_phi, d_ins
        d_x
        d_y
        ...
        d_c
        '''
        epsilon = 1e-6 # to avoid division by zero, not numerically stable with smaller perturbation
        s, theta, phi = curve
        theta += epsilon # theta never negative, prevents division by zero
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_phi   = np.cos(phi)
        s_phi   = np.sin(phi)

        dx_dcurve = np.zeros((6,4))
        # dx_dcurve row
        dx_dcurve[0,0] = c_phi * (1 - c_theta) / theta
        dx_dcurve[0,1] = c_phi * s * (theta * s_theta + c_theta - 1) / (theta**2)
        dx_dcurve[0,2] = -s_phi * s * (1 - c_theta) / theta
        # dy_dcurve row
        dx_dcurve[1,0] = s_phi * (1 - c_theta) / theta
        dx_dcurve[1,1] = s_phi * s * (theta * s_theta + c_theta - 1) / (theta**2)
        dx_dcurve[1,2] = c_phi * s * (1 - c_theta) / theta
        # dz_dcurve row
        dx_dcurve[2,0] = s_theta / theta
        dx_dcurve[2,1] = s * (theta * c_theta - s_theta) / (theta**2)
        dx_dcurve[2,3] = 1.
        # da_dcurve row
        dx_dcurve[3,2] = 1.
        # db_dcurve
        dx_dcurve[4,1] = 1.
        # dc_dcurve
        dx_dcurve[5,2] = -1.
        return dx_dcurve



    def estimateModelJacobian(self, q): # could speed up by only applying delta to indep dofs
        '''
        gets the model jacobian unaffected by noise, should be sufficiently close to the true jacobian 
        and serves as the check for any analytical form (the one above is not yet working)

        This needs to be done without being affected by the environmental rotation matrix...
        '''
        use_lung_task = False
        J = np.zeros((self.x_num, self.q_num))
        dq = 1e-5
        q_start = q.copy()
        q_start_scaled = self.scaleGearRatios(q.copy())
        x_start, _ = self.updateRobotFramePosition(q_start_scaled, use_lung_task)
        x_start[2] += q_start_scaled[8]
        for i, qi in enumerate(q):
            q[i] += dq
            q_scaled = self.scaleGearRatios(q.copy())
            x_robot_frame, _ = self.updateRobotFramePosition(q_scaled,use_lung_task)
            x_robot_frame[2] += q_scaled[8]
            dx = x_robot_frame - x_start
            J[:,i] = dx / dq
            q = q_start.copy()
        return J


    def getTensions(self):
        '''
        returns the tensions calculated using a simple heuristic
        stands in for current sensing and tension sensing in robot
        antagonistic cables have equal tensions, always positive
        heuristic =  average displacement between antagonistic cables
        '''
        return self.K.dot(self.q) 

    def getModelRotation(self):
        '''
        returns the rotation applied to the robot, determined
        by getEnvRotation. This function allows us to track the 
        true rotation for later comparisons
        3x3 numpy array
        '''
        return self.R_env.copy(), self.R_angles.copy()

    def getModelPose(self):
        ''' returns the azimuth and elevation angles wrt to the base model frame'''
        return self.x_robot_frame.copy()

    def testParameterFit(self, 
                         q, 
                         x_sensed,
                         start_index = 0,
                         parameters = [],
                         base_angles = []):
        ''' function accepts the recorded q's from the robot and the recorded
        tip pose(x_sensed), as well as the set of parameters that will be implemented
        for the run. The output x array will be return and the rms error between 
        the output and the reference trajectory in each dimension. 
        Tuples should be (leader, sheath)'''
        if parameters:
            angles, thetas, motor_scales, lengths, diameters = parameters
            self.setParameters(angles, thetas, motor_scales,  lengths, diameters)
        N, m = x_sensed.shape
        x = np.zeros((N,m))

        for i, qi in enumerate(q):
            if len(base_angles):
                self.R_env = trig.R_zyz(base_angles[i])
            self.setDesiredPosition(qi)
            x[i,:] = self.getCoordinatePosition()[:m]

        x_initial = x_sensed[start_index,:].copy()
        x_initial[3:] *= np.pi / 180
        x = x[:,:] - x[start_index,:] + x_initial

        error =  x_sensed - x
        error_rms = np.sqrt((1 / len(x)) * sum(error**2))

        offset = 1
        dx   = x[offset:,:] - x[:-offset,:]
        dx_sensed = x_sensed[offset:,:] - x_sensed[:-offset,:]
        error_dx =  dx_sensed - dx
        error_dx_rms = np.sqrt((1 / len(dx)) * sum(error_dx**2))

        return x, error_rms, error_dx_rms







