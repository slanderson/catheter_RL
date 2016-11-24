# ConvexHelper.py
import cvxpy as cvx
import numpy as np
import time
import scipy.linalg as sl
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()

from lib import mpc_3_5
mpc_cvx = mpc_3_5.CVXWrapper()

def getDQTensionMin(J, dx, K, tension, tension_initial):
    m,n = J.shape
    dq = cvx.Variable(n)
    lamb = 0.1
    objective = cvx.Minimize(cvx.norm(tension + K*dq) + lamb * cvx.norm(dq))
    constraints = [J * dq == dx, tension + K*dq >= tension_initial]
    prob = cvx.Problem(objective, constraints)
    # tsolve = time.perf_counter()
    result = prob.solve(solver = 'SCS', warm_start = False)
    # print(1000 * (time.perf_counter() - tsolve))
    return np.array(dq.value).flatten()

def getMPCdq(J, dx, K, tension, tension_initial, steps = 3):
    # Form and solve control problem.
    # based on http://nbviewer.jupyter.org/github/cvxgrp/cvx_short_course/blob/master/intro/control.ipynb#Control
    m, n = J.shape
    x_desired = np.zeros(m) #puts in generic form
    x0 = -dx
    x = cvx.Variable(m, steps+1)
    u = cvx.Variable(n, steps)
    states = []
    Q = np.eye(m) * 0.5
    R = np.eye(n) * 0.1
    P = np.eye(m)
    # P = sl.solve_discrete_are(np.eye(m), J, Q, R)
    u_max = 1
    tension_scale = 0.1
    # loop costs
    for t in range(steps):
        cost = cvx.quad_form(x[:,t+1], Q) + cvx.quad_form(u[:,t], R) + tension_scale * cvx.norm(tension + K*u[:,t]) 
        if t == steps - 1: #final cost
            cost += cvx.quad_form(x[:,steps], P)
        constr = [x[:,t+1] == x[:,t] + J*u[:,t],
                  cvx.norm(u[:,t], 'inf') <= u_max,
                  tension + K*u[:,t] >= tension_initial]
        states.append(cvx.Problem(cvx.Minimize(cost), constr))
    # sums problem objectives and concatenates constraints.
    prob = sum(states)
    prob.constraints += [x[:,0] == x0]
    prob.solve(solver = 'SCS', warm_start = True)
    U = np.array(u.value)
    return U[:,0] #returns the first step

def getMPCdq2(J, x0, x_trajectory, K, tension, dq_last, steps = 3):
    # Form and solve control problem.
    # adding in trajectory look ahead
    m, n = J.shape
    u_max = 0.5
    bowl_radius = 0.5

    step_est = int(np.ceil(max(np.linalg.pinv(J).dot(x_trajectory[:,0] - x0)) / u_max))
    first_steps = min(step_est, steps)
    print(step_est, first_steps)

    I_select = np.zeros(steps + 2)
    x_traj = np.tile(x_trajectory[:,0], (steps + 2))
    if step_est < steps:
        # x_traj[n * (step_est+1):] =  np.tile(x_trajectory[:,1], (steps + 1 - step_est))
        I_select[step_est] = 1.

    x = cvx.Variable(m, steps+1)
    u = cvx.Variable(n, steps)
    tau = cvx.Variable(n, steps+1)

    states = []
    Q = np.eye(m) * 0.1
    R = np.eye(n) * 0.1
    R_delta = np.eye(n) * 0.2
    R_tension = np.eye(n) * 0.1
    P_mid = np.eye(m) * 1
    P = np.eye(m) * 1
    # P = sl.solve_discrete_are(np.eye(m), J, Q, R)
       
    # loop costs
    for t in range(first_steps):
        cost = cvx.quad_form(x[:,t+1] - x_trajectory[:,0], Q) + cvx.quad_form(tau[:,t], R_tension) 
        constr = [x[:,t+1] == x[:,t] + J*u[:,t],
                  tau[:,t+1] == tau[:,t] + K*u[:,t],
                  cvx.norm(u[:,t], 'inf') <= u_max,
                  tau[:,t] >= 0]
        if t == first_steps - 1 and first_steps == steps: #final constrt
            # constr += [cvx.norm(x[:,first_steps] - x_trajectory[:,0]) <= bowl_radius] # constraint worries me for feasibility
            cost += cvx.quad_form(x[:,first_steps] - x_trajectory[:,0], P_mid)
            cost += cvx.quad_form(tau[:,first_steps], R_tension) 
        if t > 0:
            cost +=  cvx.quad_form(u[:,t] - u[:,t-1], R_delta)
        else:
            cost += cvx.quad_form(u[:,0] - dq_last, R_delta)
        states.append(cvx.Problem(cvx.Minimize(cost), constr))
    # skips if no transition (first_steps = steps)
    for t in range(first_steps, steps): 
        cost = cvx.quad_form(x[:,t+1] - x_trajectory[:,1], Q) + cvx.quad_form(tau[:,t], R_tension) 
        cost +=  cvx.quad_form(u[:,t] - u[:,t-1], R_delta)
        constr = [x[:,t+1] == x[:,t] + J*u[:,t],
                  cvx.norm(u[:,t], 'inf') <= u_max,
                  tau[:,t] >= 0]
        if t == steps - 1: #final cost
            cost += cvx.quad_form(x[:,steps] - x_trajectory[:,1], P)  
            cost += cvx.quad_form(tau[:,steps], R_tension) 
        states.append(cvx.Problem(cvx.Minimize(cost), constr))
    # sums problem objectives and concatenates constraints.
    prob = sum(states)
    prob.constraints += [x[:,0] == x0, 
                        tau[:,0] == tension]
    for t in range(steps+1):
        prob.constraints += [cvx.abs(I_select[t] * (x[:,t] - x_trajectory[:,0])) <= bowl_radius]
    # if first_steps < steps:
    #     # prob.constraints += [cvx.norm(x[:,first_steps] - x_trajectory[:,0]) <= bowl_radius]
    #     prob.constraints += [cvx.abs(x[:,first_steps] - x_trajectory[:,0]) <= bowl_radius]

    prob.solve(solver = 'SCS', warm_start = True)
    U = np.array(u.value)
    return U[:,0] #returns the first step

def get_mpc_cvx(J, x0, x_trajectory, K, tension, u_prev):
    '''
    parameters
    J (n,m) # Jacobian (R * J * W), dx = Jdq.
    x[0] (n)  # initial state.
    x_traj[t] (n), t=0..T+1 # traj
    tension (m) # measured instantaneous tension - tension_initial
    K (m,m) # stiffness matrix d_tension = K * dq
    u_max nonnegative  # amplitude limit.
    Q (n,n) diagonal psd  # state cost.
    Q_final (n,n) diagonal psd  # final state cost.
    R_tension (m,m) diagonal psd # tension cost.
    R_du (m,m) diagonal psd  # change in control cost.
    I_select[t] nonnegative, t=1..T+1 # index for the traj switch, 0 otherwise
    bowl_radius positive  # bowl constraint

    note n, m flip
    '''
    n, m = J.shape

    T = 15 # set in cvxgen source
    Q = np.eye(n) * 0.001
    Q_final   = np.eye(n) * 0.1
    Q_tension = np.eye(m) * 1e2
    R_du      = np.eye(m) * 1.
    R_du[-1,-1] = 0
    u_max     = np.array([1.])
    epsilon   = np.array([0.4]) #0.5 in idm
    I_select  = np.zeros(T + 2)
    x_traj    = np.tile(x_trajectory[:,0], (T + 2))
    tension_min = np.zeros(n)
    delta = 0

    step_est = int(np.ceil(max(abs(np.linalg.pinv(J).dot(x_trajectory[:,0] - x0))) / u_max))
    step_est += delta
    if step_est < 0:
        print(x_traj.shape,np.tile(x_trajectory[:,1], (T + 1 - step_est)).shape, step_est)
        print(J)
        print(x_trajectory[:,0] - x0)
        print(np.linalg.pinv(J).dot(x_trajectory[:,0] - x0))
        print(max(abs(np.linalg.pinv(J).dot(x_trajectory[:,0] - x0))))
    if step_est < T:
        x_traj[n * (step_est+1):] =  np.tile(x_trajectory[:,1], (T + 1 - step_est))
        # x_traj[n * step_est:] =  np.tile(x_trajectory[:,1], (T + 2 - step_est))
        I_select[step_est] = 1.

    mpc_cvx.runSolver(J.flatten('F'), 
                      x0, 
                      x_traj, 
                      tension, 
                      K.flatten('F'), 
                      u_max,
                      u_prev, 
                      np.diag(Q), 
                      Q_final.flatten('F'), 
                      np.diag(Q_tension), 
                      np.diag(R_du), 
                      I_select, 
                      epsilon,
                      tension_min)
    dq_desired  = np.asarray([mpc_cvx.getData(i) for i in range(m)])
    return dq_desired

def get_mpc_prediction():
    ''' must be called after get_mpc_cvx, provides an array of the 15 x 3 xyz positions
    that were predicted at the given time'''
    T = 15 # set in cvxgen source
    x_num = 3
    dx_predicted = np.zeros([T-1, x_num])
    for time_pt in range(1,T):
        dx_predicted[time_pt-1,:] = np.asarray([mpc_cvx.getX(time_pt,i) for i in range(x_num)])
    return dx_predicted

def get_tension_min(J, dx, K, tension, u_prev):
    m,n = J.shape
    dq = cvx.Variable(n)
    Q_tension = 0.
    R_du = 0.
    Q = 10.
    u_max = 0.1
    objective = cvx.Minimize(Q_tension * cvx.norm(tension + K*dq) + R_du * cvx.norm(dq - u_prev) + Q * cvx.norm(J * dq - dx))
    constraints = [tension + K*dq >= 0]
    prob = cvx.Problem(objective, constraints)
    # tsolve = time.perf_counter()
    result = prob.solve(solver = 'ECOS', warm_start = False)
    # print(1000 * (time.perf_counter() - tsolve))
    return np.array(dq.value).flatten()