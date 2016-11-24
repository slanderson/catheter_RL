'''
modelless_helper.py contains the functions specific to updating
the jacobian according to Yip's MLC papers

J. Sganga
8/21/2016
'''

import sys, time
import numpy as np


'''
Mike's MLC is actually a linear upadate. This is a least norms
with a linear constraint format (See Boyd's EE263 lecture on least norms)
'''
def linear_jacobian_update(J, dq, dx, check_rank = False):
    # DQ should be given as W * dq
    # Expects inputs are numpy arrays, J is 2D, DQ is length of actuated q,
    x_num, q_num = J.shape 
    J_num = x_num * q_num
    # Linear constraint that dx = J dq
    Aeq = np.kron(np.eye(x_num), dq)
    # full matrix with [I Aeq'; Aeq 0]
    A = np.zeros((J_num + x_num, J_num + x_num))
    A[:J_num, :J_num] = np.eye(J_num)
    A[J_num:, :J_num] = Aeq
    A[:J_num, J_num:] = Aeq.T
    # setting equal to [Jold, dx]
    b = np.zeros(J_num + x_num)
    b[:J_num] = J.flatten()
    b[J_num:] = dx
    # solving jacobian. matrix should always be square and full rank unless all dq are 0
    new_J_vec = np.linalg.inv(A).dot(b)
    #  new_J_vec is in array of shape (x_num * q_num + x_num, 1)
    #  need to slice out the lambda terms that represents a rough idea of how differen these J are
    #  J to 2D array
    J = np.reshape(new_J_vec[:J_num], (x_num, q_num))
    change = np.ravel(new_J_vec[J_num:])
    if check_rank:
        if not svd_rank_check(J):
            return (Jacobian, 0)
    return (J, change)

def svd_rank_check(J):
     # # # # # #  Condition number  check  # # # # # # 
    U, s, V = np.linalg.svd(J, full_matrices=True)
    conditionNumber = s[2]/s[1]
    if (conditionNumber < 0.05): # somewhat arbitrary threshold
        print ("****************** BAD JACOBIAN ***************")
        sys.stdout.flush()
        ts = time.perf_counter()
        return False
    else:
        return True

########################## robustification ####################################
''' Not working, yet, needs fix '''
def robustify(robot):
    x0 = robot.x_sensed[robot.x_list]
    # going backwards through list, maintaining index in original order
    for i, index in reversed(list(enumerate(robot.robust_index))):
        dx = x0 - robot.history_array[index][1][robot.x_list] # x_sensed in 2nd position
        if dx.dot(dx) > robot.dx_threshold:
            break
        if (i == 0): return (0, 0, False)

    dq = robot.q_desired - robot.history_array[index][5] 
    if dq.dot(dq)  < 1e-5: 
        return (0, 0, False)
        
    del robot.robust_index[:i] # list before this point is unnecessary
    return(dx, dq, True)

'''
Gets a Jacobian estimate by moving each set of independent actuators and measuring
the position displacement. Antagonistic motor pairs are actuated together because 
they are currently forced to be equal and opposite as tension feedback is not 
possible.
'''
def initialize_jacobian_estimate(robot): 
    robot.aurisUpdate()
    J = np.zeros((robot.x_num, robot.q_num))
    K = np.eye(robot.q_num_full)
    dq_list = robot.getInitialDqList()
    # dq_list = 3 * np.eye(robot.q_num)
    q_start = robot.q.copy()
    robot.goToQ_loop(q_start + robot.q_initial)
    robot.spinWait(1)
    for i, dq in enumerate(dq_list):
        robot.spinWait(1)
        robot.sensorUpdate()
        robot.aurisUpdate()
        q_desired = robot.expand_q.dot(dq) + q_start + robot.q_initial
        x_start   = robot.x_sensed[:robot.x_num].copy()
        q_amps_start = robot.q_amps.copy()
        robot.goToQ_loop(q_desired)
        robot.spinWait(2)
        robot.sensorUpdate()
        robot.aurisUpdate()
        dx = robot.x_sensed[:robot.x_num] - x_start
        amps_delta = robot.q_amps - q_amps_start
        for j, dq_j in enumerate(dq):
            if dq_j != 0:
                # if j < 8: #tendons
                #     dq_j*=2
                J[:, j] = dx / dq_j # if use dq=0 here, it becomes a singular matrix...
                K[:, j] = amps_delta / dq_j # using abs to use heuristic of mirroring the antagonistic stiffness
        robot.goToQ_loop(q_start + robot.q_initial)
        sys.stdout.flush()
    # Fill out the weighting matrix! to now take the form: dx = J W dq
    robot.W = np.diag([np.linalg.norm(J[:, i]) for i in range(robot.q_num)])
    J = np.transpose(np.asarray([J[:,i] / robot.W[i,i] for i in range(robot.q_num)]))
    robot.J = J
    print('J ', robot.J)
    # print('K ', robot.K)
    sys.stdout.flush()
    robot.spinWait(2)
    # set up for next time
    robot.x_past_J_update = robot.x_sensed[:robot.x_num].copy()
    robot.q_past_J_update = robot.q_desired.copy()
    robot.force_past_J_update = robot.force_sensed.copy()
    robot.dx_expected *= 0

    if not robot.model_estimation.lower() == 'mlc': 
        robot.modelUpdate()
        robot.W = np.eye(robot.q_num)

'''makes list of q commands for initial J'''
def getInitialDqList(robot):
    dq = 2 if not robot.vision_flag else 0.5
    if not robot.use_leader_and_sheath:
        dq_list = dq * np.eye(3)
    else:
        dq_list = dq * np.eye(5)
        dq_list[:2] *= 1.5 #sheath tendons 
    dq_list[-1] *= 3 #insertion axis
    if not robot.reduced_flag:
        dq_list = [robot.expandQPairs(this_dq) for this_dq in dq_list]
    return dq_list
'''tries to make writing out dqs easier'''
def expandQPairs(robot, q):
    q_expanded = np.zeros(robot.q_num)
    if not robot.use_leader_and_sheath:
        q_expanded[0] =  q[0]
        q_expanded[1] =  q[1]
        q_expanded[2] = -q[0]
        q_expanded[3] = -q[1]
        q_expanded[4] =  q[2]
    else:
        q_expanded[0] =  q[0]
        q_expanded[1] =  q[1]
        q_expanded[2] = -q[0]
        q_expanded[3] = -q[1]
        q_expanded[4] =  q[2]
        q_expanded[5] =  q[3]
        q_expanded[6] = -q[2]
        q_expanded[7] = -q[3]
        q_expanded[8] =  q[4]
    return q_expanded

'''
Gets Jacobian for the leader. Used to test the empirical Jacobian under different constraints. 
'''
def getLeaderJacobian(robot): 
    robot.aurisUpdate()
    J = np.zeros((robot.x_num_full, robot.q_num)) # all x_coords, 4 tendons + ins (includes sheath, but unused)
    dq = 1
    dq_list = dq * np.eye(5)[2:,:]
    dq_list = np.vstack((dq_list, -dq_list))
    dq_list = [robot.expandQPairs(this_dq) for this_dq in dq_list]
    q_start = robot.q.copy()
    q_amps_start = robot.q_amps.copy()
    for i, dq in enumerate(dq_list):
        robot.spinWait(1)
        robot.sensorUpdate()
        q_desired = robot.expand_q.dot(dq) + q_start + robot.q_initial
        x_start   = robot.x_sensed[:robot.x_num].copy()
        robot.goToQ_loop(q_desired)
        robot.spinWait(2)
        robot.sensorUpdate()
        robot.aurisUpdate()
        dx = robot.x_sensed[:robot.x_num] - x_start
        for j, dq_j in enumerate(dq):
            if dq_j > 0: # taking only positive values
                if j < 8: #tendons
                    dq_j*=2
                J[:, j] = dx / dq_j # if use dq=0 here, it becomes a singular matrix...
        robot.goToQ_loop(q_start + robot.q_initial)
        sys.stdout.flush()
    # Fill out the weighting matrix! to now take the form: dx = J W dq
    # robot.W = np.diag([np.linalg.norm(J[:, i]) for i in range(robot.q_num)])
    # J = np.transpose(np.asarray([J[:,i] / robot.W[i,i] for i in range(robot.q_num)]))
    robot.J = J
    print(robot.J)
    sys.stdout.flush()
    robot.spinWait(2)



def right_inv(J, dx):
    # Computes the RIGHT inverse only (for redundant manipulators)
    # the linalg.pinv call will be slower than the direct solution
    # also, this throws an error when J*Jt is singular, which is good
    return J.T.dot(np.linalg.inv(J.dot(J.T)).dot(dx))

def inv_equal_opposite(J, dx):
    # Assumes J is not singular
    # Also assumes we are using 5 acutated dq, (q1 = -q3, q2 = -q4)
    m, n = J.shape
    if n == 5:
        J_add_on  = np.array([[1., 0., 1., 0., 0.],
                             [0., 1., 0., 1., 0.]])
        tendon_pairs = 2
        
    elif n == 9:
        J_add_on  = np.array([[1., 0., 1., 0., 0., 0., 0., 0., 0.],
                             [0., 1., 0., 1., 0., 0., 0., 0., 0.],
                             [0., 0., 0., 0., 1., 0., 1., 0., 0.],
                             [0., 0., 0., 0., 0., 1., 0., 1., 0.]])
        tendon_pairs = 4
    J_full = np.zeros((m + tendon_pairs, n))
    J_full[:m, :] = J
    J_full[m:, :] = J_add_on
    dx_full = np.zeros(m + tendon_pairs)
    dx_full[:m] = dx
    if m + tendon_pairs < n:
        return right_inv(J_full, dx_full)
    else:
        return np.linalg.inv(J_full).dot(dx_full)