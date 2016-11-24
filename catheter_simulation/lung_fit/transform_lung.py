'''
transform_stl is used to fit the orientation and position of the STL in the 
global frame of the position sensor.
The file history_guide_along_model2.p has the guide information 
being run over the actual model. 

Jake Sganga
8/5/16

rename, edit and move to control_robot 8/25/2016

to look at plots, use common_plots.py

expects to be run from data_analysis folder's common_plots.py
'''
import sys, time
import matplotlib.pyplot as plt
# import seaborn as sns # importing this here makes changing plot settings in jupyter hard
import numpy as np
import pickle

from data_analysis.load_history import history_data
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()

def get_transformed_lung():
    data = history_data(file = 'history_guide_fit_sparse.p', folder = computer_paths.lung_folder)
    with open(computer_paths.lung_folder + 'zunu_airtree.p', "rb" ) as input_file:
        lung = pickle.load(input_file)

    x_offset = 40
    y_offset = 52
    z_offset = -230

    offset = np.array([x_offset, y_offset, z_offset])
    R_lung = np.array([[0, 0, 1],
                       [-1, 0, 0],
                       [0, -1, 0]])

    lung_transformed = np.array([R_lung.dot(l) + offset  for l in lung])
    return lung_transformed, data.x_guide

