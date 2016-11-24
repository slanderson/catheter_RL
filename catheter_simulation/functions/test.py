'''
Jake Sganga
Function is used to calculate the closed form solution of the jacobian update

takes as input the old jacobian, Wdq, and dx
as vectors. This function finds the least norms solution to the 
minimization of the squared norm of
delta J given the linear constraint dx = Jdq

'''
import time
import LinearJacobianUpdate as ljq

import numpy as np 
import scipy.linalg as sl

J = np.array([[1, 2, 3], [3, 4, 4]])
J = np.ones((3,5))
dq = np.array([1, 1, 1, 1, 1])
dx = np.array([6.5, 10, 1])

newJ, change = ljq.LinearJacobianUpdate(J, dq, dx)
print(newJ)



now = time.perf_counter()
print(now)

# while time.perf_counter() - now < 2: pass

print(time.perf_counter())

t1 = np.ones(10)


print(t1/2.)


# print(change)

# J = np.array([[1, 2, 3], [1, 2, 3]])
# print(np.linalg.matrix_rank(J))
# desiredX = np.array([1, 1])

# now = time.time()
# for i in range(20000):
#     dq1 = np.dot(sl.pinv(J), desiredX)
# print(time.time() - now)

# now = time.time()
# for i in range(20000):
#     dq2 = np.dot(sl.pinv2(J), desiredX)
# print(time.time() - now)

# now = time.time()
# for i in range(20000):
#     dq3 = ljq.GetDQfromPsuedoInv(J, desiredX)
# print(time.time() - now)

now = time.perf_counter()
for i in range(1000):
    dq3, change = ljq.LinearJacobianUpdate(J, dq, dx)
print('here ', 1000 * (time.perf_counter() - now))


# print(dq1, dq2, dq3)
t = list(range(1,10))
print(t[4:9])
# deltaActModel = np.asarray([1:10])
q_num = 5

dq_test = np.asarray([deltaActModel[ix] for ix in [0,1,2,3,8] if q_num == 5])
print(dq_test)