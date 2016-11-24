'''
Jake Sganga
10/27/15

Function is used to calculate the closed form solution of the jacobian update

takes as input the old jacobian, Wdq, and dx
as vectors. This function finds the least norms solution to the 
minimization of the squared norm of
delta J given the linear constraint dx = Jdq





FILE DEPRICATED!!!!!!!!!!!!!!!!!!!!!!! See modelless_helper.py

'''
import sys, time
import numpy as np
import scipy.linalg as sl # apparently this solver is slightly faster than np.linalg



def LinearJacobianUpdate(Jacobian, DQ, DX):
    # DQ should be given as W * dq
    # Expects inputs are numpy arrays, J is 2D, DQ is length of actuated q,
    Jacobian = Jacobian.flatten()
    # general to different sizes
    x_num = len(DX)
    q_num = len(DQ)
    J_num = len(Jacobian)
    # Linear constraint that dx = J dq
    Aeq = np.kron(np.eye(x_num), DQ)
    # full matrix with [I Aeq'; Aeq 0
    fullA = np.bmat([[np.eye(J_num), np.transpose(Aeq)],
                     [Aeq, np.zeros((x_num, x_num))]])
    # setting equal to [Jold, dx]
    b = np.bmat([Jacobian, DX])
    # solving jacobian. matrix should always be square and full rank unless all dq are 0
    newJacobian = sl.solve(fullA, np.transpose(b))
    #  newJacobian is in array of shape (x_num * q_num + x_num, 1)
    #  need to slice out the lambda terms that represents a rough idea of how differen these J are
    #  J to 2D array
    J = np.reshape(newJacobian[:J_num], (x_num, q_num))
    change = np.ravel(newJacobian[J_num:])

    # # # # # #  Condition number  check  # # # # # # 
    U, s, Vh = np.linalg.svd(J, full_matrices=True)
    conditionNumber = s[2]/s[1]
    # print("Condition Number", conditionNumber, "\n")
    if (conditionNumber < 0.05):
        print ("****************** BAD JACOBIAN ***************")
        sys.stdout.flush()
        ts = time.perf_counter()
        while(time.perf_counter() - ts < 1000):
            pass
        orig_J = np.reshape(Jacobian[:J_num], (x_num, q_num))
        return(orig_J, 0)

    return (J, change)

def GetDQfromPsuedoInv(Jacobian, DX):
    # Computes the RIGHT inverse only (for redundant manipulators)
    # the linalg.pinv call will be slower than the direct solution
    # also, this throws an error when J*Jt is singular, which is good
    Jt = np.transpose(Jacobian)
    JJt = np.dot(Jacobian, Jt)
    return np.dot(Jt, sl.solve(JJt, DX))

def GetDQfromPsuedoInv_EqualOpposite(Jacobian, DX):
    # Assumes J is not singular
    # Also assumes we are using 5 acutated dq, (q1 = -q3, q2 = -q4)
    m, n = Jacobian.shape
    if n == 5:
        J_add_on = np.array([[1., 0., 1., 0., 0.],
                             [0., 1., 0., 1., 0.]])
        dx_add_on = np.zeros(2)
    elif n == 9:
        J_add_on = np.array([[1., 0., 1., 0., 0., 0., 0., 0., 0.],
                             [0., 1., 0., 1., 0., 0., 0., 0., 0.],
                             [0., 0., 0., 0., 1., 0., 1., 0., 0.],
                             [0., 0., 0., 0., 0., 1., 0., 1., 0.]])
        dx_add_on = np.zeros(4)
    else: print("full case not yet implemented!!!!!!!!!!!!!!!!!!!!")
    J = np.vstack((Jacobian, J_add_on)) # Jacobian is a 2D array
    dx = np.hstack((DX, dx_add_on)) # DX is a 1D array
    m, n = J.shape
    if m < n:
        return GetDQfromPsuedoInv(J, dx)
    else:
        return sl.solve(J, dx)


# rotates frame based on the azimuth and elevation recording (ignoring roll for now)
# returns the rotation matrix
# RzRy written according to positive axis displacements, but elevation is about -Y', so 
# automatically flipping the sign 
# Needs radians, but receives degrees from Ascension
def RzRy(azimuth, elevation, radians = False):
    scalar = np.pi / 180 if not radians else 1
    elevation = -elevation * scalar
    azimuth = azimuth * scalar
    return np.array([[np.cos(azimuth) * np.cos(elevation), -np.sin(azimuth), np.cos(azimuth) * np.sin(elevation)],
                     [np.sin(azimuth) * np.cos(elevation),  np.cos(azimuth), np.sin(azimuth) * np.sin(elevation)],
                     [                 -np.sin(elevation),               0., np.cos(elevation)]])

# Note that input is elevation, but aligning the Z-axis requires a 
#  positive Y-axis rotation by (90 deg - elevation). Remember radians!
# Need to rotate again by the z-axis to undo the inherent roll. Really is
#  Rz(azimuth)Ry(np.pi - el)Rz(-azimuth)
def RzRyRz(a, e, radians = True):
    scalar = np.pi / 180 if not radians else 1
    e = np.pi/2 - scalar * e
    a *= scalar
    return np.array([[np.cos(a)**2 * np.cos(e) + np.sin(a)**2, np.cos(a) * np.sin(a) * (np.cos(e) - 1), np.cos(a) * np.sin(e)],
                     [np.cos(a) * np.sin(a) * (np.cos(e) - 1), np.cos(a)**2 + np.sin(a)**2 * np.cos(e), np.sin(a) * np.sin(e)],
                     [                - np.cos(a) * np.sin(e),                  -np.sin(a) * np.sin(e),            np.cos(e)]])

def LinearJacobianUpdate_NoSafety(Jacobian, DQ, DX):
    # DQ should be given as W * dq
    # Expects inputs are numpy arrays, J is 2D, DQ is length of actuated q,
    Jacobian = Jacobian.flatten()
    # general to different sizes
    x_num = len(DX)
    q_num = len(DQ)
    J_num = len(Jacobian)
    # Linear constraint that dx = J dq
    Aeq = np.kron(np.eye(x_num), DQ)
    # full matrix with [I Aeq'; Aeq 0
    fullA = np.bmat([[np.eye(J_num), np.transpose(Aeq)],
                     [Aeq, np.zeros((x_num, x_num))]])
    # setting equal to [Jold, dx]
    b = np.hstack((Jacobian, DX))
    # solving jacobian. matrix should always be square and full rank unless all dq are 0
    newJacobian = sl.solve(fullA, np.transpose(b))
    #  newJacobian is in array of shape (x_num * q_num + x_num, 1)
    #  need to slice out the lambda terms that represents a rough idea of how differen these J are
    #  J to 2D array
    J = np.reshape(newJacobian[:J_num], (x_num, q_num))
    change = np.ravel(newJacobian[J_num:])
    return (J, change)

def MultiObjectiveLSUpdate(J, H, dq, dfv, dx):
    # DQ should be given as W * dq
    # Expects inputs are numpy arrays, J is 2D, DQ is length of actuated q,
    # general to different sizes
    bias = 50
    J = J.ravel()
    H = bias * H.ravel()
    JH  = np.hstack((J, H))

    JH_num = len(JH)
    J_num  = len(J)
    H_num  = len(H)
    q_num  = len(dq)
    fv_num = len(dfv)
    x_num  = len(dx)

    # Linear constraint that dx = J dq
    A_q  = np.kron(np.eye(x_num), dq)
    A_fv = np.kron(np.eye(x_num), dfv)
    Aeq = np.hstack((A_q, A_fv))
    # Multiobjective skew
    I_j = np.ones(x_num * q_num)
    I_h = bias * np.ones(x_num * fv_num)
    I   = np.diag(np.hstack((I_j, I_h)))  
    # I = np.eye(JH_num)

    fullA = np.bmat([[I, np.transpose(Aeq)],
                     [Aeq, np.zeros((x_num, x_num))]])
    # setting equal to [JHold, dx]
    b = np.hstack((JH, dx))
    # solving jacobian. matrix should always be square and full rank unless all dq are 0
    newJH = sl.solve(fullA, np.transpose(b))
    #  newJacobian is in array of shape (x_num * q_num + x_num, 1)
    #  need to slice out the lambda terms that represents a rough idea of how differen these J are
    #  J to 2D array
    newJ   = newJH[:J_num]
    newH   = newJH[J_num:JH_num]
    change = newJH[JH_num:]

    J = np.reshape(newJ, (x_num, q_num))
    H = np.reshape(newH, (x_num, fv_num))

    return (J, H, change)


def DeltaJacobianUpdate(Jacobian, DQ, DX):
    # DQ should be given as W * dq
    # Expects inputs are numpy arrays, J is 2D, DQ is length of actuated q,
    x_num = len(DX)
    q_num = len(DQ)
    # Linear constraint that dx = J dq
    Q = np.kron(np.eye(x_num), DQ)
    b = DX - Jacobian.dot(DQ)
    # solving jacobian. matrix should always be square and full rank unless all dq are 0
    DJ = Q.T.dot(sl.solve(Q.dot(Q.T), b))
    #  newJacobian is in array of shape (x_num * q_num + x_num, 1)
    #  need to slice out the lambda terms that represents a rough idea of how differen these J are
    #  J to 2D array
    DJ = np.reshape(DJ, (x_num, q_num)) 
    J = DJ + Jacobian
    # # # # # #  Condition number  check  # # # # # # 
    U, s, Vh = np.linalg.svd(J, full_matrices=True)
    conditionNumber = s[2]/s[1]
    # print("Condition Number", conditionNumber, "\n")
    if (conditionNumber < 0.05):
        print ("****************** BAD JACOBIAN ***************")
        return Jacobian

    return DJ
