import numpy as np
def featureVector(i):
    fv = []
    [fv.append(np.sin(i / (1*j))) for j in range(9, 20)]
    [fv.append(np.cos(i / (1*j))) for j in range(9, 20)]
    fv.append(1)
    return np.asarray(fv)