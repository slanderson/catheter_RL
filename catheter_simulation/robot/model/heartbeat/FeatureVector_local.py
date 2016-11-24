import numpy as np
def featureVector(i):
    fv = []
    # [fv.append(i**j) for j in range(5)]
    [fv.append(1 if i==j else 0) for j in range(150)]
    return np.asarray(fv)