'''
load_history.py
the module will be called by the data analysis tools to load history.p pickled arrays
after simulations or robot runs.

Will make a class that will hold all the useful variables.
'''
import sys
import pickle
import numpy as np
sys.path.append('..')
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()



class history_data(object):
    """docstring for Data"""
    def __init__(self, 
                 file = 'history.p', 
                 folder = computer_paths.data_folder):
        self.file = file
        self.folder = folder
        self.unpack_history()

    def unpack_history(self):
        with open(self.folder + self.file, "rb" ) as input_file:
            history = pickle.load(input_file)
        try:
            (time_pts,
            x_sensed,
            x_raw,
            x_desired,
            q,
            q_desired,
            q_desired_raw,
            J, 
            W,
            R,
            angles_sensed,
            angles_model,
            amps,
            amps_raw,
            force_sensed,
            force_raw,
            x_guide,
            dx_predicted) = history.T
            
            self.angles_model = np.asarray(list(angles_model))
        except ValueError:
            (time_pts, x_sensed, x_raw, x_desired, q, q_desired, q_desired_raw, 
                J,  W, R, angles_sensed, amps, amps_raw, force_sensed, force_raw, 
                x_guide, dx_predicted) = history.T
        # robot specific variables:
        self.q_desired_raw = np.asarray(list(q_desired_raw))
        self.amps_raw = np.asarray(list(amps_raw))
                

        self.time_pts      = np.asarray(list(time_pts))
        self.x_sensed      = np.asarray(list(x_sensed))
        self.x_raw         = np.asarray(list(x_raw))
        self.x_desired     = np.asarray(list(x_desired))
        self.q             = np.asarray(list(q))
        self.q_desired     = np.asarray(list(q_desired))
        self.J             = np.asarray(list(J))
        self.W             = np.asarray(list(W))
        self.amps          = np.asarray(list(amps))
        self.force_sensed  = np.asarray(list(force_sensed))
        self.force_raw     = np.asarray(list(force_raw))
        self.R             = np.asarray(list(R))
        self.angles_sensed = np.asarray(list(angles_sensed))
        self.x_guide       = np.asarray(list(x_guide))
        self.dx_predicted  = np.asarray(list(dx_predicted))
        
        self.x = self.x_sensed[:,0] 
        self.y = self.x_sensed[:,1]
        self.z = self.x_sensed[:,2]
        self.a = self.x_sensed[:,3]
        self.e = self.x_sensed[:,4]
        # self.r = self.x_sensed[:,5]



                