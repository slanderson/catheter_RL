
import sys
import time
import scipy.linalg as sl
import matplotlib.pyplot as plt
# import cvxpy
import numpy as np
import pickle
import FeatureVector_signal as fvs
import FeatureVector_local as fvl
# sys.path.append("C:\\Users\\Auris\\Box Sync\\Research\\Auris\\PyAuris\\Functions")
sys.path.append("../../Functions")
import LinearJacobianUpdate as lju 


with open( "heartbeat.p", "rb" ) as input_file:
    heart = pickle.load(input_file)
    heart_cath = heart[0]
    heart_stage = heart[1]
    delta_heart = heart[2]


# def featureVector(i):
#     fv = []
#     # fv.append(i%120)
#     # fv.append((i%120)**2)
#     # fv.append(i%130)
#     # fv.append((i%130)**2)
#     # fv.append(np.sin(i / 20))
#     # fv.append(np.sin(i / 10))
#     # fv.append(np.cos(i / 20))
#     # fv.append(np.cos(i / 10))
#     [fv.append(np.sin(i / (1*j))) for j in range(9, 20)]
#     [fv.append(np.cos(i / (1*j))) for j in range(9, 20)]
#     # [fv.append(np.sin(i / (.25*j))**2) for j in range(1, 80)]
#     # [fv.append(np.cos(i / (.25*j))**2) for j in range(1, 80)]
#     fv.append(1)
#     return fv

# heart_cath = heart_cath[:800]
# featureMatrix = np.asarray([featureVector(i) for i in range(len(heart_cath))])

# # print(featureMatrix.shape)
# AtA = np.dot(np.transpose(featureMatrix), featureMatrix)
# Atb = np.dot(np.transpose(featureMatrix), heart_cath[:,1])
# print(AtA.shape)
# print(Atb.shape)
# # wx = sl.pinv(featureMatrix, heart_cath[:,0])
# wx = sl.solve(AtA, Atb)
# print(sl.norm(np.dot(featureMatrix, wx) - heart_cath[:,1]))

# print(wx)
# plt.plot(np.dot(featureMatrix, wx), '.-')
# plt.plot(heart_cath[:,1],'--.')
# plt.show()


i = 100
raw = np.asarray([np.cos(j * (2 * np.pi/i)) for j in range(i)])
# plt.figure()
plt.plot(raw)
raw = np.reshape(raw, (i, 1))

featureMatrix = np.asarray([fvs.featureVector(j) for j in range(i)])
print(featureMatrix[:10,:], raw[:10])

# print(featureMatrix.shape)
AtA = np.dot(np.transpose(featureMatrix), featureMatrix)
Atb = np.dot(np.transpose(featureMatrix), raw)
print(AtA.shape)
print(Atb.shape)
# wx = sl.pinv(featureMatrix, heart_cath[:,0])
wx = sl.solve(AtA, Atb)
print(featureMatrix.shape, raw.shape)
w, res, rank, s = sl.lstsq(featureMatrix, raw)
print(w.shape)
print(res)
print(sl.norm(np.dot(featureMatrix, w) - raw))
# print('w', wx)
plt.plot(np.dot(featureMatrix, w), '--')

W = w.ravel()
# W = np.asarray([1.5,-.08, 8.06064343e-04,   6.94411678e-07, 1.00374759e-10,   4.99951781e-13])
# print(W)
# frac = 0.01
# for j in range(1000):
#     ix = j%i
#     fv = fvs.featureVector(ix)
#     # W = np.asarray([W])
#     print(W.shape, fv.shape, raw[ix].shape)
#     newW, change = lju.LinearJacobianUpdate_NoSafety(W, fv, raw[ix])
#     W = (1 - frac)*W + frac*newW.ravel()

# print(sl.norm(np.dot(featureMatrix, W.ravel()) - raw))
# plt.plot(np.dot(featureMatrix, W), '.')
# W = W.ravel()
# fv = fvs.featureVector(2)


# W = np.ones(len(fv))

fv = fvs.featureVector(2)
E = np.diag(W)
W = np.ones(len(fv))
frac = 0.0001
for j in range(1000):
    ix = j%i
    fv = np.dot(E, fvs.featureVector(ix))
    # print(raw[ix] - np.dot(W, fv))
    # print(raw[ix], np.dot(W,fv),  W, frac*(raw[ix] - np.dot(W, fv))*fv)
    W = W + frac*(raw[ix] - np.dot(W, fv))*fv
    # time.sleep(1)

# print(sl.norm(np.dot(featureMatrix, W.ravel()) - raw))
plt.figure()
featureMatrix = np.asarray([fvs.featureVector(j%100) for j in range(1000)])
plt.plot(np.dot(featureMatrix, np.dot(E,W)), '.')
 





### with heart beat
length = 150
# plt.figure()
raw = heart_cath[:length, 0]
plt.figure()
plt.plot(raw)
# raw = np.reshape(raw, (i, 1))

featureMatrix = np.asarray([fvl.featureVector(j) for j in range(length)])

w, res, rank, s = sl.lstsq(featureMatrix, raw)
# print(w.shape)
# print(res)
print('here', sl.norm(np.dot(featureMatrix, w) - raw))
print('w', w)
plt.plot(np.dot(featureMatrix, w), '--')

# W = w.ravel()

window = 2
new_beat = []
for j in range(window,len(heart_cath[:,0]) - window):
    frac  = .9
    prior = frac*heart_cath[j-1, 0] + (1-frac)*heart_cath[j-2, 0]
    post  = frac*heart_cath[j+1, 0] + (1-frac)*heart_cath[j+2, 0]
    if heart_cath[j, 0] > 1 and prior < heart_cath[j, 0] > post:
        new_beat.append(j)
print(new_beat)
W = w
length = 7000
frac = 0.25
ix = 0
estimate = []
EKG = new_beat[:]
for j in range(length):
    if j == EKG[0]:
        EKG.pop(0)
        ix = 0
    fv = np.asarray(fvl.featureVector(ix))
    # print(heart_cath[j, 0], heart_cath[ix,0], np.dot(W, fv))
    # print(raw[ix], np.dot(W,fv),  W, frac*(raw[ix] - np.dot(W, fv))*fv)
    estimate.append([j, np.dot(W, fv)])
    W = W + frac*(heart_cath[j, 0] - np.dot(W, fv))*fv
    # time.sleep(1)
    ix += 1

# print(sl.norm(np.dot(featureMatrix, W.ravel()) - raw))
plt.figure()
featureMatrix = np.asarray([fvl.featureVector(j%length) for j in range(length)])
plt.plot(heart_cath[:length, 0], '.')
# plt.plot(np.dot(featureMatrix, W), '.')
estimate = np.asarray(estimate)
plt.plot(estimate[:,0], estimate[:,1])
print(sl.norm(estimate[:,1] - heart_cath[:length,0]))


# print(W)

plt.show()