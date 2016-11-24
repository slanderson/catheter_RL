
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing
import random

# from functions import LinearJacobianUpdate as lju
from functions import low_pass_filter as f
# # from functions import ConvexHelper as ch
import functions.trig as trig
import functions.state_estimation as se

t = []
h = t.insert(0, 't')
print(h)

import pickle
file = '~/Google Drive/Research/data/soft_robot/robot_data/lung_runs/history_abc_sparse_traj.p'
file = '/Users/jakesganga/Google Drive/Research/data/soft_robot/robot_data/lung_runs'
with open(file, "rb" ) as input_file:
        history = pickle.load(input_file)