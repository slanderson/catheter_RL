'''
Estimates the Rotation matrix (scaling and tranformation) of the model-based jacobian 
based on the observed outputs, dx. This is an alternative to the full jacobian 
update, making the estimation smaller (3x3 vs 3xm), and preserving some information
on the constrained dimensions. Limited to the xyz control because azimuth and elevation
angles do not obey the simple transformation. 

Formula below based on the function:
min ||R_t+1 - R_t||
s.t. dx_observed = R_t+1 dx_expected

Letting dx_expected = sum(J(q_i) * dq_i) i = 0 -> end, note without R_t

'''
import numpy as np
import scipy.linalg as sl

def rotationEstimation(R, dx_observed, dx_expected, step_size = 0.25):
	'''
	uses the right inverse of the kron(dx_expected) to solve the equation above
	uses trick to R.flatten() to stack the rows vertically into a 1D vector
	'''
	x_num = 3
	assert(len(dx_expected) == x_num)
	K  = np.kron(np.eye(x_num), dx_expected)
	dR = K.T.dot(np.linalg.inv(K.dot(K.T))).dot(dx_observed - R.dot(dx_expected))
	R_new =  R.flatten() + step_size * dR
	return np.reshape(R_new, (x_num, x_num))

