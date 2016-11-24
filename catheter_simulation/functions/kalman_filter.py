'''
Function applies the measurement update and time update on the estimated jacobian.
It creates its own catheter (ModelInterface) object to figure out the model jacobian.

J sganga 3/27/16
'''

import numpy as np
import scipy.linalg as sl
from robot.model.model_kinematics import catheter 
import functions.trig as trig

robot = catheter(use_leader     = True,
                 use_sheath     = True,
                 sensor_noise   = [0,0,0], 
                 use_obstacle   = False,
                 use_heartbeat  = False)  

def kalmanFilterUpdate(J, cov_J, dx, dq, q, q_old, sensor_noise, process_noise, model_scale):
    '''
    expects J as a matrix (m x n)
    cov_J as symmetric positive matrix (mn x mn)
    dx, 1d vector, size = number of x parameters being controlled
    dq, 1d vector, size = articulated q
    q,  1d vector, size = full q
    sensor_noise, 1d vector size of dx, needs to be supplied becuase changes when dx elements are from force sensor
    process_noise, 1d vec size of q, how much noise you expect in moving between J's, equivalent to MLC's J step_size
    model_scale, scalar (for now, could even by mn sized vec), how  much of the model you want to use 
    '''
    j = J.ravel() # stack row-wise, lower case j means vector form
    Q = np.kron(np.eye(len(dx)), dq) # makes matrix of diagonal blocks of dq row
    j, cov_J = measurementUpdate(j, cov_J, Q, dx, sensor_noise)
    J = j.reshape((len(dx), len(dq))) # back to matrix 
    J, cov_J = timeUpdate(J, cov_J, q, q_old, process_noise, model_scale)
    return J, cov_J

def measurementUpdate(j, cov_J, Q, dx, sensor_noise):
    '''
    mmse estimate of j (j is the mean of the gaussian vector)
    cov according to gaussian noise (see e207 notes)
    '''
    # print(np.diag(cov_J).dot(np.diag(cov_J)))
    s11 = cov_J
    s12 = cov_J.dot(Q.T)
    s21 = s12.T
    s22 = Q.dot(cov_J.dot(Q.T)) + sensor_noise * np.eye(len(dx))
    # s22_inv = np.linalg.inv(s22)
    j += s12.dot(np.linalg.inv(s22)).dot(dx - Q.dot(j))
    cov_J = s11 - s12.dot(np.linalg.inv(s22).dot(s21))
    return j, cov_J

def timeUpdate(J, cov_J, q, q_old, process_noise, model_scale):
    '''
    updates according to current q (measurement update is effectively the estimated J for the last q)
    get model J's in matrix form so might as well reshape here
    not sure what to do for cov of delta j
    '''
    if model_scale:
        m, n = J.shape
        J_model_new = robot.estimateModelJacobian(q)[:m, :n]
        J_model_old = robot.estimateModelJacobian(q_old)[:m, :n]
        J += model_scale * (J_model_new - J_model_old)
    
    cov_J += process_noise * np.eye(cov_J.shape[0])
    
    # if model_scale == 1: # doesn't account for weights..
    #     J = J_model_new
    return J, cov_J


################# EKF##########################
'''
EKF relies on linearizing a non-linear function about the point
of the current state estimate. In this case, the non-linear function is
our expected dx (dx_expected), which will be compared to dx_sensed 
in the measurement update step: res = dx_sensed - h([a,b,c])
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

def ekfUpdate(angles, dx_model, residual, sensor_noise = 0.1, cov_angle = np.eye(3)):
    '''
    mmse estimate of Euler angles
    Really just a measurement update step (non-linear recursive estimation)
    Linearize the rotation matrix R_zyx
    '''
    H = getH(angles, dx_model)
    s11 = cov_angle
    s12 = cov_angle.dot(H.T)
    s21 = s12.T
    s22 = H.dot(cov_angle.dot(H.T)) + sensor_noise * np.eye(3)
    angle_est = angles + s12.dot(np.linalg.inv(s22)).dot(residual)
    cov_angle = s11 - s12.dot(np.linalg.inv(s22).dot(s21))
    return angle_est, cov_angle

def getH(angles, dx_model):
    '''
    Forms H matrix based on the current estimate of the Euler angles and the dx_model because dx_expected = R dx_model
    '''
    H = np.zeros((3,3))
    H[0,:] = trig.dRx_dabc(angles).dot(dx_model)
    H[1,:] = trig.dRy_dabc(angles).dot(dx_model)
    H[2,:] = trig.dRz_dabc(angles).dot(dx_model)
    return H



'''
Unscented Kalman Filter - pick points around estimate and see...
https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb

Same as EKF, no prediction step
'''

def ukf_measurement_update(angles, 
                           curve,
                           dx_model, 
                           curve_model,
                           dx_sensed,
                           ab_sensed,
                           sensor_noise, 
                           cov_state,
                           use_curve):

    num_curve = 1 if use_curve else 0
    nx = len(angles)  + num_curve   # state variables we want to update
    nz = len(dx_sensed) + len(ab_sensed) + num_curve # sensor variables we model/observe
    if use_curve:
        state = np.hstack((angles, curve[1])) # only estimating the theta parameter
        sensor_sensed = np.hstack((dx_sensed, ab_sensed, curve_model[1]))
    else:
        state = angles
        sensor_sensed = np.hstack((dx_sensed, ab_sensed))
    # weights
    Wm, Wc, lam = get_weights(n = nx)
    # sigma points, 
    sigmas_f, sigma_num = get_sigma_points(lam, cov_state, state)
    # get mean and cov of sampled points
    # xp, Px = unscented_transform(sigmas_f, np.zeros((nx,nx)), Wm, Wc)
    xp, Px = state, cov_state
    # get expected sensor reading based on curvature
    angle_vec = []
    if len(ab_sensed):
        angle_vec = get_angles(curve, sigmas_f, use_curve, sigma_num)
    # transform sigma points to measurement space
    sigmas_h  = get_sigmas_h(sigmas_f, sigma_num, dx_model, angle_vec, use_curve)
    # mean and covariance of prediction based on sigma pts
    zp, Pz = unscented_transform(sigmas_h, sensor_noise * np.eye(nz), Wm, Wc)
    # compare to observed data and update angle estimation
    angles, P = update(sensor_sensed, sigmas_f, sigmas_h, xp, Px, zp, Pz, Wc)
    return angles, P

def get_weights(alpha = 0.05, beta = 2, kappa = 0.1, n = 3):
    # weights on the mean and covariance of the test (sigma) points
    lam = alpha**2 * (n + kappa) - n
    Wc = np.full(2*n + 1,  1. / (2*(n + lam)))
    Wm = np.full(2*n + 1,  1. / (2*(n + lam)))
    Wc[0] = lam / (n + lam) + (1. - alpha**2 + beta)
    Wm[0] = lam / (n + lam)
    return Wm, Wc, lam

def get_sigma_points(lam, P, state):
    # Pick sigma points based on Van der Merwe thesis
    n = len(state)
    sigmas = np.zeros((2*n+1, n))
    U = sl.cholesky((n+lam)*P) # sqrt
    # U = np.sqrt(n+lam) * np.eye(n) #faster, using identity for now.
    X = state.copy()
    sigmas[0] = X # first point is mean
    for k in range(n):
        sigmas[k+1]   = X + U[k]
        sigmas[n+k+1] = X - U[k]

    kmax, n = sigmas.shape
    return sigmas, kmax

def unscented_transform(sigmas, noise, Wm, Wc):
    # find weighted mean and cov
    x = np.dot(Wm, sigmas)
    kmax, n = sigmas.shape
    P = np.zeros((n, n))
    for k in range(kmax):
        y = sigmas[k] - x
        P += Wc[k] * np.outer(y, y) 
    P += noise
    return x, P

def get_angles(curve, sigmas_f, use_curve, sigma_num):
    # constructs a vector alpha, beta, gamma angles for each 
    # sigma, allows for varying curvature to affect the expected sensor reading
    angle_vec = np.zeros((sigma_num, 3))
    for i in range(sigma_num):
        curve_i = curve.copy()
        if use_curve:
            curve_i[1] = sigmas_f[i,3]
        angle_vec[i,:] = robot.getPositionFromCurve(curve_i)[3:] # extracting expected alpha, beta, gamma from curve
    return angle_vec


def get_sigmas_h(sigmas_f, sigma_num, dx_model, angle_vec, use_curve):
    # transform sigma points into measurement space
    # see what you'd get for each state
    n_dx = len(dx_model)
    n_ab = len(angle_vec)
    n_curve = 1 if use_curve else 0
    sigmas_dx = np.zeros((sigma_num, n_dx)) 
    sigmas_ab = np.zeros((sigma_num, n_ab)) 
    sigmas_curve = np.zeros((sigma_num, n_curve)) 
    
    for i in range(sigma_num):
        R_env_guess  = trig.R_zyz(sigmas_f[i,:3])
        if n_dx: # differential xyz
            sigmas_dx[i,:] = R_env_guess.dot(dx_model[:3])
        if n_ab: # instantaneous angles
            R_model        = trig.R_zyz(angle_vec[i,:]) 
            sigmas_ab[i,:] = trig.getAnglesZYZ(R_env_guess.dot(R_model))[:n_ab] # excludes roll if not included 
        if use_curve: # displacment based curvature
            sigmas_curve[i,:] = sigmas_f[i,3] 
    return np.hstack((sigmas_dx, sigmas_ab, sigmas_curve))
    
def update(sensors_sensed, sigmas_f, sigmas_h, xp, Px, zp, Pz, Wc):
    # compute cross variance of the state and the measurements
    sigma_num, nx = sigmas_f.shape
    _, nz         = sigmas_h.shape
    Pxz = np.zeros((nx, nz))
    for i in range(sigma_num):
        Pxz += Wc[i] * np.outer(sigmas_f[i] - xp, sigmas_h[i] - zp)
    K = np.dot(Pxz, np.linalg.inv(Pz)) # Kalman gain
    x = xp + np.dot(K, sensors_sensed - zp)
    P = Px - K.dot(Pz.dot(K.T))
    return x, P

def ukf_time_update(state, cov_state, q, process_noise = 0.1):
    # performs the UKF time update, updates state estimate and 
    # the covariance of the state
    # calculate sigma points for given mean and covariance
    nx = 1   # state variables we want to update, curve
    # weights
    Wm, Wc, lam = get_weights(n = nx)
    # sigma points of inputs 
    sigmas, sigma_num = get_sigma_points(lam, cov_state, q[4:8]) # grabs only leader tendons
    # transform sigma points thru fwd kin
    sigmas_curve = curve_transform(sigmas)
    # get mean and cov of sampled points
    u_curve, P_curve = unscented_transform(sigmas_curve[:,1], np.ones((nx,nx)) * process_noise, Wm, Wc)



def curve_transform(sigmas):
    # creates curve sigma points based on sigma points of the 
    sigmas_curve = np.zeros((len(sigmas), 3)) # 3 curve paramters returned, s, theta, phi
    for i, sig in sigmas:
        sigmas_curve[i,:] = robot.get_leader_curvature(sig)
    return sigmas_curve


'''
UKF update for curve parameters with base angles as dependents
'''

def ukf_curve_update(curve,
                     curve_indeces, 
                     curve_model,
                     tip_angles,
                     base_angles,
                     sensor_noise, 
                     cov_state):
    scale_angles  = 10
    state = curve[curve_indeces]
    nx = len(state)   # state variables we want to update (note, the angles will get updated dependent on curve)
    nz = nx + len(base_angles) # sensor variables we model (nx) / observe (angles)
    # weights
    Wm, Wc, lam = get_weights(alpha = 1e-3, n = nx)
    # sigma points
    sigmas_f, sigma_num = get_sigma_points(lam, cov_state, state)
    # get mean and cov of sampled points - not necessary to do the transform, bc the sigma points preserve mean and cov
    # xp, Px = unscented_transform(sigmas_f, np.zeros((nx,nx)), Wm, Wc)
    mu_state, P_state = state, cov_state
    # transform sigma points to measurement space
    sigmas_h  = get_sigmas_h_curve(sigmas_f, sigma_num, tip_angles, curve, curve_indeces, scale_angles)
    # mean and covariance of prediction based on sigma pts
    mu_meas, P_meas = unscented_transform(sigmas_h, sensor_noise * np.eye(nz), Wm, Wc)
    # define the 'sensed' parameters as modelled curve parameters and the previous base angles, might need scaling...
    
    sensor_sensed = np.hstack((curve_model[curve_indeces], base_angles * scale_angles))
    # compare to observed data and update angle estimation
    new_curve_parameters, P = update(sensor_sensed, sigmas_f, sigmas_h, mu_state, P_state, mu_meas, P_meas, Wc)
    # based on new curve, get corresponding base angles to match measurement
    new_curve = curve
    new_curve[curve_indeces] = new_curve_parameters 
    new_angles   = get_base_angles(new_curve, tip_angles) 
    return new_curve, new_angles, P

def get_sigmas_h_curve(sigmas_f, sigma_num, tip_angles, curve, curve_indeces, scale_angles):
    # transform sigma points into measurement space
    # here this means derive the necessary base angles to fit the curves (sigmas_f) with measured tip (tip_angles) 
    n_abc = 3 # 3 Euler angles, R_zyz
    sigmas_base = np.zeros((sigma_num, n_abc))
    
    for i in range(sigma_num):
        sampled_curve = curve.copy()
        sampled_curve[curve_indeces] = sigmas_f[i,:]
        sigmas_base[i,:] = get_base_angles(sampled_curve, tip_angles) * scale_angles
    return np.hstack((sigmas_f, sigmas_base))

def get_base_angles(curve, tip_angles):
    # finds the deterministic base angles to fit the curve parameters with the measured tip angles
    # R_tip_ground  = R_base_ground * R_tip_base
    # R_base_ground = R_tip_ground * R_tip_base^T
    angles_to_base = robot.getPositionFromCurve(curve)[3:] # extracting expected alpha, beta, gamma from curve
    R_tip_base     = trig.R_zyz(angles_to_base) 
    R_tip_ground   = trig.R_zyz(tip_angles)
    R_base_ground  = R_tip_ground.dot(R_tip_base.T)
    base_angles    = trig.getAnglesZYZ(R_base_ground)
    return base_angles

