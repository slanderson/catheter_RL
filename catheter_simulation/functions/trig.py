'''
Collection of all the trig functions / rotation matrices that are used throughout
the simulation and control functions
'''
import numpy as np

############ used by ModelInterface ##################
# useful functions:
def Rx(alpha):
    return np.array([[1, 0, 0.],
                    [0,   np.cos(alpha), np.sin(alpha)],
                    [0.,   -np.sin(alpha), np.cos(alpha)]])

def Rz(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0.],
                    [np.sin(alpha),   np.cos(alpha), 0.],
                    [           0.,              0., 1.]])
def Ry(alpha):
    return np.array([[np.cos(alpha), 0., np.sin(alpha)],
                    [            0., 1., 0.],
                    [-np.sin(alpha), 0., np.cos(alpha)]])
# Faster to just make define the matrix and skip the matrix mult step
# Note that input is elevation, but aligning the Z-axis requires a 
#  positive Y-axis rotation by (90 deg - elevation). Remember radians!
# Need to rotate again by the z-axis to undo the inherent roll. Really is
#  Rz(azimuth)Ry(np.pi - el)Rz(-azimuth)

def RzRyRz(a, e):
    e = np.pi/2 - e
    return np.array([[np.cos(a)**2 * np.cos(e) + np.sin(a)**2, np.cos(a) * np.sin(a) * (np.cos(e) - 1), np.cos(a) * np.sin(e)],
                     [np.cos(a) * np.sin(a) * (np.cos(e) - 1), np.cos(a)**2 + np.sin(a)**2 * np.cos(e), np.sin(a) * np.sin(e)],
                     [                - np.cos(a) * np.sin(e),                  -np.sin(a) * np.sin(e),            np.cos(e)]])



# derivative of RzRyRz wrt azimuth
def dRda(a,e):
    e = np.pi/2 - e
    return np.array([
        [2 * (np.cos(a) * np.sin(a)) * (-np.cos(e) + 1), (np.cos(e) - 1) * (np.cos(a)**2 - np.sin(a)**2), -np.sin(a) * np.sin(e)],
        [(np.cos(e) - 1) * (np.cos(a)**2 - np.sin(a)**2), 2 * (np.cos(a) * np.sin(a)) * (np.cos(e) - 1), np.cos(a) * np.sin(e)],
        [np.sin(a) * np.sin(e), -np.cos(a) * np.sin(e), 0]
        ])
def dRde(a,e): 
    e = np.pi/2 - e #note becuase of this conversion, the matrix is multiplied by -1 to correct for the change of variables
    return -1 * np.array([
        [-np.cos(a)**2 * np.sin(e), -np.cos(a) * np.sin(a) * np.sin(e), np.cos(a) * np.cos(e)],
        [-np.cos(a) * np.sin(a) * np.sin(e), -np.sin(a)**2 * np.sin(e), np.sin(a) * np.cos(e)],
        [-np.cos(a) * np.cos(e), -np.sin(a) * np.cos(e), -np.sin(e)]
        ])

# flips to 
R_reorient = np.array([[0, 0, 1],
                       [0, 1, 0],
                       [-1,0, 0]])

def getAEfromR(R):
    Zx, Zy, Zz = R[:3,2] # extracts the Z column of the rotation matrix (all that's necessary for the az/el calculation)
    azimuth = np.arctan2(Zy, Zx) 
    elevation = np.arcsin(Zz)
    return np.asarray([azimuth, elevation])

# 





############# EKF rotations and derivatives #####################
'''
EKF relies on linearizing a non-linear function about the point
of the current state estimate. In this case, the non-linear function is
our expected dx (dx_expected), which will be compared to dx_observed 
in the measurement update step: res = dx_observed - h([a,b,c])
h([a,b,c]) = R_zyx(a,b,c) * J(q) * dq = R_zyx(a,b,c) * dx_model = dx_expected
where a, b, c are the Euler angles yaw, pitch, roll, respectively.
See ModelEnvironment.py for the matrix R_zyx, which is the transpose
of the matrix in Craig pg. 49.
To linearize, we want a jacobian matrix such that h([a,b,c]) ~ H [a, b, c]'
H = dh/d[a, b, c]
This gets a little tricky because we have a 3x3 matrix of non-linear functions 
mutiplied by dx_model to give us our output. 
Below I take the derivative of each row of R_zyx(a,b,c) with respect to a, b, c, 
this requires some transposing to make everything work out.
for example, in the x direction:
h([a,b,c]) ~= H [a, b, c]'
H = [
    [dh_x/da, dh_x/db, dh_x/dc],
    [dh_y/da, dh_y/db, dh_y/dc],
    [dh_z/da, dh_z/db, dh_z/dc]
]

[dh_x/da, dh_x/db, dh_x/dc] = (dRx_dabc * dx_model)'
'''
def R_zyx(angles):
    '''
    Forms rotation matrix based on azimuth elevation and roll anlges
    performing yaw (+z) first (around global z), which is typically called alpha = a
    then pitch (+y) (around relative y), aka beta = b, note that this is different that elevation, used elsewhere
    then roll  (+x) (around most relative x), aka gamma = c
    Z-Y-X Euler angles goes from rotated frame back to ground
    We want to multiply the ground into the rotated frame, so taking transpose of Rzyx 
    in page 49 of Craig
    '''
    a, b, c = angles
    return np.array([
        [np.cos(a) * np.cos(b), np.sin(a) * np.cos(b), -np.sin(b)],
        [np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c), np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c), np.cos(b) * np.sin(c)],
        [np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c), np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c), np.cos(b) * np.cos(c)]])

def getABCfromR(R):
    # assumes matrix in form of Rzyx from Craig pg 49
    a = np.arctan2(R[0, 1], R[1, 1])
    b = -np.arcsin(R[0, 2])
    c = np.arctan2(R[1, 2], R[2, 2])
    return np.array([a, b, c])

def dRx_dabc(angles):
    '''
    Derivative of the x row of R_zyx
    Note that a, b, c represent the Euler angles yaw, pitch, roll - axes(Z, Y, X)
    Rx = [np.cos(a) * np.cos(b), np.sin(a) * np.cos(b), -np.sin(b)]
    '''
    a, b, c = angles
    return np.array([
        [ -np.sin(a) * np.cos(b),    np.cos(a) * np.cos(b),          0], #da
        [np.cos(a) *(-np.sin(b)), np.sin(a) * (-np.sin(b)), -np.cos(b)], #db
        [                      0,                        0,          0]]) # dc

def dRy_dabc(angles):
    '''
    Derivative of the y row of R_zyx
    Note that a, b, c represent the Euler angles yaw, pitch, roll - axes(Z, Y, X)
    Ry = [np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c), np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c), np.cos(b) * np.sin(c)]
    '''
    a, b, c = angles
    return np.array([
        [(-np.sin(a)) * np.sin(b) * np.sin(c) - np.cos(a) * np.cos(c), np.cos(a) * np.sin(b) * np.sin(c) + (-np.sin(a)) * np.cos(c), 0], #da
        [np.cos(a) * np.cos(b) * np.sin(c),                            np.sin(a) * np.cos(b) * np.sin(c),                            (-np.sin(b)) * np.sin(c)], #db
        [np.cos(a) * np.sin(b) * np.cos(c) - np.sin(a) * (-np.sin(c)), np.sin(a) * np.sin(b) * np.cos(c) + np.cos(a) * (-np.sin(c)), np.cos(b) * np.cos(c)]]) # dc

def dRz_dabc(angles):
    '''
    Derivative of the z row of R_zyx
    Note that a, b, c represent the Euler angles yaw, pitch, roll - axes(Z, Y, X)
    Rz = [np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c), np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c), np.cos(b) * np.cos(c)]
    '''
    a, b, c = angles
    return np.array([
        [(-np.sin(a)) * np.sin(b) * np.cos(c) + np.cos(a) * np.sin(c), np.cos(a) * np.sin(b) * np.cos(c) - (-np.sin(a)) * np.sin(c), 0], #da
        [np.cos(a) * np.cos(b) * np.cos(c)                           , np.sin(a) * np.cos(b) * np.cos(c)                           , (-np.sin(b)) * np.cos(c)], #db
        [np.cos(a) * np.sin(b) * (-np.sin(c)) + np.sin(a) * np.cos(c), np.sin(a) * np.sin(b) * (-np.sin(c)) - np.cos(a) * np.cos(c), np.cos(b) * (-np.sin(c))]]) # dc


########## from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb
# used to handle angles as state variables
def normalize_angle(x):
    x = x % (2 * np.pi)    # force in range [0, 2 pi)
    if x > np.pi:          # move to [-pi, pi)
        x -= 2 * np.pi
    return x


def state_mean(sigmas, Wm):
    # Calculates mean of angles (handles the 359 and 3 deg mean = 1 deg)
    x = np.zeros(3)

    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x

def z_mean(sigmas, Wm):
    z_count = sigmas.shape[1]
    x = np.zeros(z_count)

    for z in range(0, z_count, 2):
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, z+1]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, z+1]), Wm))

        x[z] = np.sum(np.dot(sigmas[:,z], Wm))
        x[z+1] = atan2(sum_sin, sum_cos)
    return x


'''
Alternative to RzRyRz is to use the R_zyz from Craig pg 49. I thing this is what I should be trying to replicate in 
RzRyRz, which implicitly set c = -azimuth, and b = pi/2 + elevation. It also has the inverse of the rotation. 
a = alpha (+Z, ground), b = beta (+Y'), c = gamma (+Z")
ranges:
a: [-pi, pi]
b: [0, pi]
c: [-pi, pi]
arctan2 automatically enforces the -pi to pi bounds
the sqrt on the eqn for b (see getAnglesZYZ), enforces [0, pi]
note that care needs to be taken with ranges and delta angles

 Elevation = -pi/2 + beta *************
'''

def R_zyz(angles):
    if len(angles) == 3:
        a, b, c = angles
    elif len(angles) == 2: #often used for cath rotations, allows seemless replacement of RzRyRz
        a, b = angles
        c = -a
    return np.array([
        [np.cos(a) * np.cos(b) * np.cos(c) - np.sin(a) * np.sin(c), -np.cos(a) * np.cos(b) * np.sin(c) - np.sin(a) * np.cos(c), np.cos(a) * np.sin(b)],
        [np.sin(a) * np.cos(b) * np.cos(c) + np.cos(a) * np.sin(c), -np.sin(a) * np.cos(b) * np.sin(c) + np.cos(a) * np.cos(c), np.sin(a) * np.sin(b)],
        [                                   -np.sin(b) * np.cos(c),                                      np.sin(b) * np.sin(c),             np.cos(b)]])

def getAnglesZYZ(R):
    b = np.arctan2(np.sqrt(R[2,0]**2 + R[2,1]**2), R[2,2])
    if b == 0:
        a = 0
        c = np.arctan2(-R[0,1], R[0,0])
    elif b == np.pi:
        a = 0
        c = np.arctan2(R[0,1], -R[0,0])
    else:
        a = np.arctan2(R[1,2] / np.sin(b), R[0,2] / np.sin(b))
        c = np.arctan2(R[2,1] / np.sin(b), -R[2,0] / np.sin(b))
    return np.array([a, b, c])

def correct_delta_angles(angles):
    a, b, c = angles
    # a and c are put in [-pi, pi)
    a = a % (2 * np.pi)    # range [0, 2 pi)
    if a > np.pi:          # move to [-pi, pi)
        a -= 2 * np.pi

    b = b % (np.pi)    # range [0, 2 pi)
    if b > np.pi:          # move to [0, pi]
        b -= 2 * np.pi

    c = c % (2 * np.pi)    # range [0, 2 pi)
    if c > np.pi:          # move to [-pi, pi)
        c -= 2 * np.pi

def aer_to_abc(sensor_angles):
    # converting ascension angles (degress) to euler angles (radians)
    # R_zyz expects abc form
    # elevation = -pi/2 + beta
    a, el_rad, c = sensor_angles * np.pi / 180
    b =  el_rad + np.pi / 2
    return np.array([a, b, c])

def abc_to_aer(euler_angles):
    az, b_deg, roll = euler_angles * 180 / np.pi
    el = b_deg - 90.
    return np.array([az, el, roll])
    
    



