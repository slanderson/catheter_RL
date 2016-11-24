'''
process_square_runs.py

processes a set of pickled files consisting of repeated runs
in a given configuration for later plotting.

J. Sganga 8/31/2016
'''

import sys
import numpy as np
from scipy.interpolate import interp1d
from load_history import history_data


class multiple_runs(object):
    '''class holds data of each loaded run, processes it. child processes will plot'''
    def __init__(self, file_names, complete_run = True):
        self.run_list  = []
        self.file_names = file_names
        for file in file_names:
            self.run_list.append(history_data(file = file))
        self.find_way_point_indeces()
        self.start_pts = self.indeces[1,:]
        self.end_pts   = self.indeces[-1,:]
        self.center_data()
        # self.get_average_trace()
        if complete_run:
            self.get_radial_traces()
        else:
            self.end_pts = [len(run.time_pts) for run in self.run_list]
            self.get_partial_traces()

    def center_data(self):
        for i, run in enumerate(self.run_list):
            offsets = run.x_desired[self.start_pts[i],:]
            run.x_desired = run.x_desired - offsets
            run.x_sensed  = run.x_sensed  - offsets

    def find_way_point_indeces(self):
        self.indeces = []
        for i, run in enumerate(self.run_list):
            index_list = []
            for data_pt in range(len(run.time_pts) - 1):
                if self.is_transition_point(run, data_pt):
                    index_list.append(data_pt)
            self.indeces.append(index_list)
        self.indeces = np.asarray(self.indeces).T # columns = runs

    def is_transition_point(self, run, data_pt):
        dx_desired = run.x_desired[data_pt + 1, :] - run.x_desired[data_pt, :]
        if max(abs(dx_desired)) > 0.5:
            return True
        else:
            return False


    def get_average_trace(self):
        # 0 - fixing initial pose
        # 1 - current position
        points_per_gap = 50
        self.interp_trace = np.zeros((points_per_gap * (len(self.indeces) - 3), len(self.run_list), 3))
        ix_start = 2
        for i, indeces in enumerate(self.indeces[ix_start:-1,:]):
            t_vec = np.linspace(0, 1, num=points_per_gap, endpoint=True)
            # dx will be same for each run
            for run_ix, run in enumerate(self.run_list):
                time_length = self.indeces[i+1+ix_start,run_ix] - self.indeces[i+ix_start,run_ix]
                t_train  =  np.linspace(0, 1, num=time_length, endpoint=True)
                x        = run.x_sensed[self.indeces[i+ix_start, run_ix]:self.indeces[i+1+ix_start, run_ix], 0]
                y        = run.x_sensed[self.indeces[i+ix_start, run_ix]:self.indeces[i+1+ix_start, run_ix], 1]
                z        = run.x_sensed[self.indeces[i+ix_start    , run_ix]:self.indeces[i+1+ix_start, run_ix], 2]
                f_x = interp1d(t_train, x, kind='linear')
                f_y = interp1d(t_train, y, kind='linear')
                f_z = interp1d(t_train, z, kind='linear')
                self.interp_trace[points_per_gap * i:points_per_gap * (i + 1), run_ix, 0] = f_x(t_vec)
                self.interp_trace[points_per_gap * i:points_per_gap * (i + 1), run_ix, 1] = f_y(t_vec)
                self.interp_trace[points_per_gap * i:points_per_gap * (i + 1), run_ix, 2] = f_z(t_vec)

        self.avg_trace = np.mean(self.interp_trace, axis = 1)
        self.avg_trace -= self.avg_trace[0,:]
        for i, run in enumerate(self.run_list):
            offsets = run.x_desired[self.start_pts[i],:3]
            self.interp_trace[:,i,:]  -= offsets

    def get_polar_coordinates(self, origin = np.array([0, 0, 0])):
        runs_polar = []
        for i_run, run in enumerate(self.run_list):
            x     = run.x_sensed[:,:3] - origin
            theta = np.arctan2(x[:,2], x[:,1])
            phi   = np.arctan2(x[:,0], np.sqrt(x[:,2]**2 + x[:,1]**2))
            r     = np.sqrt(x[:,2]**2 + x[:,1]**2 + x[:,0]**2)
            x_polar = np.vstack((theta, phi, r)).T
            runs_polar.append(x_polar)
        self.runs_polar = np.asarray(runs_polar)

    def get_radial_traces(self):
        bin_num = 360 * 0.5
        # along square
        self.get_polar_coordinates(origin = np.array([0, 0, 0]))
        square_avg, square_min, square_max = self.get_radial_bins(bin_num, start_index = 11, end_index = -1)
        square_avg = self.reorder_array(square_avg)
        square_min = self.reorder_array(square_min)
        square_max = self.reorder_array(square_max)
        # along insertion
        # bin_num = 360 * 0.2
        self.get_polar_coordinates(origin = np.array([0, 0, 20]))
        ins_avg, ins_min, ins_max = self.get_radial_bins(bin_num, start_index = 2, end_index = 12)
        self.radial_avg = np.vstack((ins_avg[:-3,:], square_avg))
        self.radial_min = np.vstack((ins_min[:-3,:], square_min)) # notice max min flip, dependent on origin location
        self.radial_max = np.vstack((ins_max[:-3,:], square_max))

    def get_partial_traces(self):
        bin_num = 360 * 0.5
        self.get_polar_coordinates(origin = np.array([0, -4, 3]))
        i_end = len(self.indeces)
        partial_avg, partial_min, partial_max = self.get_radial_bins(bin_num, start_index = 0, end_index = [])
        self.radial_avg = self.reorder_array(partial_avg, angle_start = -np.pi / 2) 
        self.radial_min = self.reorder_array(partial_min, angle_start = -np.pi / 2)
        self.radial_max = self.reorder_array(partial_max, angle_start = -np.pi / 2)


    def get_radial_bins(self,
                        bin_num,
                        start_index = 11, # index of x_desired in self.indeces
                        end_index   = -1):
        self.theta_vec   = np.linspace(-np.pi, np.pi, num = bin_num + 1)
        self.polar_trace = np.zeros((int(bin_num), len(self.run_list), 3))
        d_angle = 2 * np.pi / bin_num
        radial_max = []
        radial_min = []
        radial_avg = []
        for i_bin, theta in enumerate(self.theta_vec):
            min_theta = theta
            max_theta = theta + d_angle
            bin_vec_avg = []
            bin_vec_max = []
            bin_vec_min = []
            for i_run, run in enumerate(self.run_list):
                r_start = self.indeces[start_index, i_run]
                r_end   = self.indeces[end_index, i_run] if end_index else len(run.time_pts)
                polar_range = self.runs_polar[i_run][r_start:r_end, :]
                x_range     = run.x_sensed[r_start:r_end,:3]
                within_gap  = []
                within_gap  = x_range[(polar_range[:,0] >= min_theta) & (polar_range[:,0] < max_theta)]
                # import pdb; pdb.set_trace() 
                if len(within_gap):
                    radii = np.sqrt(within_gap[:,1]**2 + within_gap[:,2]**2)
                    i_min = np.argmin(radii)
                    i_max = np.argmax(radii)
                    mean_gap = np.mean(within_gap, axis = 0)
                    self.polar_trace[i_bin, i_run, :] = mean_gap
                    mean_radius = np.sqrt(mean_gap[1]**2 + mean_gap[2]**2)
                    mean_gap = np.hstack((mean_gap, mean_radius))
                    # import pdb; pdb.set_trace() 
                    bin_vec_avg.append(mean_gap)
                    bin_vec_min.append(np.hstack((within_gap[i_min,:], radii[i_min])))
                    bin_vec_max.append(np.hstack((within_gap[i_max,:], radii[i_max])))
            if bin_vec_avg:
                bin_vec_avg = np.asarray(bin_vec_avg)
                radial_avg.append(np.mean(bin_vec_avg, axis = 0)[:3])
            if bin_vec_min:
                bin_vec_min= np.asarray(bin_vec_min)
                radial_min.append(bin_vec_min[np.argmin(bin_vec_min[:,3]), :3])  
            if bin_vec_max:
                bin_vec_max = np.asarray(bin_vec_max)
                radial_max.append(bin_vec_max[np.argmax(bin_vec_max[:,3]), :3]) # finds max index for radius value, stores xyz associated
        radial_avg = np.asarray(radial_avg)
        radial_min = np.asarray(radial_min)
        radial_max = np.asarray(radial_max)
        return radial_avg, radial_min, radial_max


    def reorder_array(self, array, angle_start = np.pi/4 - 0.05):
        i_theta_start  = np.argmin(abs(self.theta_vec - angle_start))
        new_array  = array.copy()
        new_array[:len(array) - i_theta_start, :] = array[i_theta_start:,:]
        new_array[len(array) - i_theta_start:, :] = array[:i_theta_start,:]
        return new_array

