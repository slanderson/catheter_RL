

import numpy as np
import pickle
import matplotlib.pyplot as plt
import scipy.linalg as sl
from FeatureVector import *

# data is x,y,z,a,e,r
# 2 sets, first data point is the one on the catheter, 
# 2nd is the one on the beating heart stage
heart_full = []
with open('soft_incontact_stationary.txt') as heart_txt:
    for line in heart_txt:
        line_list = line.split()
        line_floats = list(map(float, line_list))
        heart_full.append(line_floats)

scale_factor = 1./6
heart = np.asarray(heart_full)
pos_avg = np.mean(heart, axis = 0)

heart_zeroed = [scale_factor*(this_pos - pos_avg) for this_pos in heart]

heart_cath  = np.asarray([pt[:6] for pt in heart_zeroed])
heart_stage = np.asarray([pt[6:] for pt in heart_zeroed])

delta_heart = np.asarray([heart_cath[i] - heart_cath[i-1] for i in range(1, len(heart_cath))])

length = heart_cath.shape[0]
print(length)
original_heart_cath = heart_cath[:]
heart_cath = heart_cath[:length - 100,:]
heart_cath[:,1] = original_heart_cath[100:,1]

heart_list = [heart_cath, heart_stage, delta_heart]



# heart_cath_train = heart_cath[:800]
# initial_weights = []
# for j in range(6):
#     featureMatrix = np.asarray([featureVector(i) for i in range(len(heart_cath_train))])

#     # print(featureMatrix.shape)
#     AtA = np.dot(np.transpose(featureMatrix), featureMatrix)
#     Atb = np.dot(np.transpose(featureMatrix), heart_cath_train[:,j])
#     # print(AtA.shape)
#     # print(Atb.shape)
#     # wx = sl.pinv(featureMatrix, heart_cath[:,0])
#     w = sl.solve(AtA, Atb)
#     print(sl.norm(np.dot(featureMatrix, w) - heart_cath_train[:,j]))
#     initial_weights.append(w)

# weights = np.asarray(initial_weights)


# # print(wx)
# w = weights[0,:]
# plt.plot(np.dot(featureMatrix, w), '-')
# plt.plot(heart_cath_train[:,0],'--')
# plt.show()

# normalize_factors = [np.linalg.norm(weights[:, i]) for i in range(weights.shape[1])]
# weights = np.transpose(np.asarray([weights[:,i] / normalize_factors[i] for i in range(weights.shape[1])]))
# heart_list.append(weights)
# heart_list.append(normalize_factors)
# print(normalize_factors)
# print(weights)




window = 2
new_beat = [0]
for j in range(window,len(heart_cath[:,0]) - window):
    frac  = .9
    prior = frac*heart_cath[j-1, 0] + (1-frac)*heart_cath[j-2, 0]
    post  = frac*heart_cath[j+1, 0] + (1-frac)*heart_cath[j+2, 0]
    if heart_cath[j, 0] > 1 and prior < heart_cath[j, 0] > post:
        new_beat.append(j)
print(new_beat)

heart_list.append(new_beat)



with open( "heartbeat.p", "wb" ) as output:
    pickle.dump(heart_list, output)
