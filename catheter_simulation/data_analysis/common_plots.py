'''
common_plots.py

keeps the code for commonly used plot forms to be used by jupyter notebooks
or other files...

note that seaborn is an ipython widget, which is giving problems in these stand 
alone files, so the plots will all have color arguments which can be filled with 
seaborn colors.

J. Sganga 8/24/16
'''

# coding: utf-8

# # Plotting History of Robot States
# 
# This script should extract the data history.p in the ./data folder
# 
# Data:
# see below and IDM.py for what arrays are included
# 
# Jake Sganga
# 2/4/2016
# 

import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from load_history import history_data
from make_lung_task_fig import make_plot_boundaries

sys.path.append('..')
from robot.model.model_kinematics import catheter
from robot.model.environment import environment
from lung_fit.transform_lung import get_transformed_lung
from functions.computer_specific_paths import computer_paths

computer_paths = computer_paths()
robot_model = catheter()
lung  = environment()

# seaborn's default colors
default_colors = [(0.2980392156862745, 0.4470588235294118, 0.6901960784313725),
                  (0.3333333333333333, 0.6588235294117647, 0.40784313725490196),
                  (0.7686274509803922, 0.3058823529411765, 0.3215686274509804),
                  (0.5058823529411764, 0.4470588235294118, 0.6980392156862745)]
blue, green, red, purple = default_colors
blues_cmap  = 'Blues'# 0x71a3a58

sheath_list = [0, 1, 2, 3]
leader_list = [4, 5, 6, 7]
axis_names  = ['X (mm)', 'Y (mm)', 'Z (mm)', 'Azimuth (deg)', 'Elevation (deg)', 'Roll (deg)', 'Time (s)']

class plot_history(object):
    """class to hold the information for plotting"""
    def __init__(self, history_file = 'history.p', colors = default_colors):
        super(plot_history, self).__init__()
        self.history_file = history_file
        self.colors = colors
        self.data   = history_data(history_file)
    
    def save_fig(self, fig, name = 'fig_test', folder = computer_paths.data_folder + '/figs/', file_type = 'png', dpi = 300):
        fig.savefig(folder + name + '.' + file_type, dpi = dpi,  format = file_type, bbox_inches='tight', pad_inches = 0.01)
        # fig.savefig(folder + name, format = 'pdf', bbox_inches='tight')

    def plot_position(self, 
                      xy = True, 
                      yz = False, 
                      xz = False, 
                      xt = False,
                      yt = False,
                      include_desired = True,
                      include_raw = False, 
                      start_pt = 0, 
                      end_pt = -1,
                      alphas = [1., 1., 0.1]):
        figs = []
        if xy: 
            figs.append(self.plot_xyz(1, 0, start_pt, end_pt, include_desired, include_raw, alphas = alphas))
        if yz:
            figs.append(self.plot_xyz(1, 2, start_pt, end_pt, include_desired, include_raw, alphas = alphas))
        if xz:
            figs.append(self.plot_xyz(2, 0, start_pt, end_pt, include_desired, include_raw, alphas = alphas))
        if xt:
            figs.append(self.plot_xyz(6, 0, start_pt, end_pt, include_desired, include_raw, alphas = alphas, faded = False))
        if yt:
            figs.append(self.plot_xyz(6, 1, start_pt, end_pt, include_desired, include_raw, alphas = alphas, faded = False))
        return figs

    def plot_magnetic_trace(self, fig, x, y):
        plt.figure(fig.number)

        t = np.linspace(2, 10, len(x))
        points = np.array([x,y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        lc = LineCollection(segments, cmap=blues_cmap,
                            norm=plt.Normalize(0, 10))
        lc.set_array(t)
        lc.set_linewidth(3)
        lc.set_label('sensed position')
        plt.gca().add_collection(lc)
        return fig

    def plot_xyz(self, 
                 x_index, # 3 is for time
                 y_index,
                 start_pt = 0, 
                 end_pt = -1, 
                 include_desired = True, 
                 include_raw = False, 
                 ax = plt.subplot(111),
                 colors = [],
                 axis_limits = [],
                 alphas = [1., 1., 0.1],
                 faded = True):

        if not colors:
            colors = self.colors

        plt.sca(ax)

        x = self.data.x_sensed[start_pt:end_pt, x_index] if x_index < 6 else self.data.time_pts[start_pt:end_pt]
        y = self.data.x_sensed[start_pt:end_pt, y_index]

        if faded:
            fig = self.plot_magnetic_trace(fig, x, y)
            plt.figure(fig.number)
        else:
            ax.plot(x, y, '-', color = colors[0], label = 'sensed', alpha = alphas[0], linewidth = 4.)

        if include_desired: 
            # start_pt = 5000
            # end_pt   = 18000
            x = self.data.x_desired[start_pt:end_pt, x_index] if x_index < 6 else self.data.time_pts[start_pt:end_pt]
            y = self.data.x_desired[start_pt:end_pt, y_index]
            ax.plot(x, y, '.', color = colors[1], label = 'desired', alpha = alphas[1], markersize = 15)

        if include_raw:
            x = self.data.x_raw[start_pt:end_pt, x_index] if x_index < 6 else self.data.time_pts[start_pt:end_pt]
            y = self.data.x_raw[start_pt:end_pt, y_index]
            ax.plot(x, y, '.', color = colors[0], label = 'raw', alpha = alphas[2])

        # plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        
        ax.set_xlabel(axis_names[x_index])
        ax.set_ylabel(axis_names[y_index])

        if axis_limits:
            ax.set_axis(axis_limits)

        return ax

    def plot_leader(self):
        return self.plot_q(leader_list), self.plot_amps(leader_list, include_raw = True)

    def plot_q(self, q_list):
        fig = plt.figure()
        for i, q_i in enumerate(q_list):
            plt.plot(self.data.time_pts, self.data.q_desired[:,q_i], color = self.colors[i], label = 'tendon ' + str(q_i))
        plt.xlabel('Time (s)')
        plt.ylabel('Displacement (mm)')
        plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        return fig

    def plot_amps(self, q_list, include_raw = False):
        fig = plt.figure()
        for i, q_i in enumerate(q_list):
            plt.plot(self.data.time_pts, self.data.amps[:,q_i], color = self.colors[i], label = 'tendon ' + str(q_i))
            if include_raw:
                plt.plot(self.data.time_pts, self.data.amps_raw[:,q_i], '.', color = self.colors[i], label = 'tendon ' + str(q_i), alpha = 0.05)

        plt.xlabel('Time (s)')
        plt.ylabel('Current (A)')
        plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        return fig
    

    def plot_angles(self, colors = default_colors):
        angle_names = [r'$\alpha$', r'$\beta$', r'$\gamma$']  
        fig = plt.figure()
        for i in range(3):
            plt.plot(self.data.time_pts, self.data.angles_sensed[:,i] * 180/np.pi, color = colors[i], label = r'sensed '+angle_names[i])
        for i in range(3):
            plt.plot(self.data.time_pts, self.data.angles_model[:,i] * 180/np.pi, '--', label = 'actual ' + angle_names[i])
        plt.xlabel('Time')
        plt.ylabel('angles (deg)')
        plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        return fig

    def plot_tension(self):
        fig = plt.figure()
        for i in range(4):
            plt.plot(self.data.time_pts, self.data.amps[:,4+i], lw = 3, label = 'Tendon ' + str(i+1))
        plt.xlabel('time')
        plt.ylabel('current')
        plt.title(' Tendon Currents')
        plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        plt.grid(True)
        return fig


######################## COMPARE WITH MODEL ##########################

    def plot_model_comparison(self, 
                             xy = True, 
                             yz = False, 
                             xz = False, 
                             a  = False,
                             e  = False,
                             r  = False,
                             include_desired = True,
                             include_raw = False,
                             start_pt = 0, 
                             end_pt = -1,
                             alphas = [0.005, 0.5, 0.1],
                             adjust_paramters = False):
        self.get_model_prediction(adjust_paramters = adjust_paramters,start_pt = start_pt)
        figs = []
        colors = [red, green]
        faded = False
        if xy: 
            fig_xy = self.plot_xyz_model(1, 0, start_pt, end_pt)
            fig_xy = self.plot_xyz(1, 0, start_pt, end_pt, include_desired, include_raw, fig_xy, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_xy)
        if yz:
            fig_yz = self.plot_xyz_model(1, 2, start_pt, end_pt)
            fig_yz = self.plot_xyz(1, 2, start_pt, end_pt, include_desired, include_raw, fig_yz, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_yz)        
        if xz:
            fig_xz = self.plot_xyz_model(2, 0, start_pt, end_pt)
            fig_xz = self.plot_xyz(2, 0, start_pt, end_pt, include_desired, include_raw, fig_xz, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_xz)
        if a: 
            fig_a = self.plot_xyz_model(6, 3, start_pt, end_pt)
            fig_a = self.plot_xyz(6, 3, start_pt, end_pt, include_desired, include_raw, fig_a, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_xy)
        if e:
            fig_e = self.plot_xyz_model(6, 4, start_pt, end_pt)
            fig_e = self.plot_xyz(6, 4, start_pt, end_pt, include_desired, include_raw, fig_e, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_yz)        
        if r:
            fig_r = self.plot_xyz_model(6, 5, start_pt, end_pt)
            fig_r = self.plot_xyz(6, 5, start_pt, end_pt, include_desired, include_raw, fig_r, colors = colors, faded = faded, alphas = alphas)
            figs.append(fig_xz)

        return figs


    def get_model_prediction(self, start_pt = 0, with_body = False, adjust_paramters = False):
        # parameters = angles, thetas, motor_scales, lengths, diameters 
        q_model = self.data.q_desired.copy() - self.data.q_desired[start_pt,:]
        q_model[:,-1] = robot_model.length_leader
        if adjust_paramters:
            parameters = self.adjust_model_paramters(start_pt)
        else:
            parameters = []
        angles_reduced_roll = self.data.angles_model.copy()
        # angles_reduced_roll[:,2] *= 0.15
        self.model_x, self.rms_x, self.rms_dx = robot_model.testParameterFit(q_model, self.data.x_sensed, start_pt, parameters, angles_reduced_roll)
        self.model_x[:,3:] *= 180/np.pi # degrees for ascension
        if with_body:
            num_points = 10
            self.model_body = np.asarray([robot_model.GetPointsAlongBody(qi, num_points) for qi in q_model])

    def adjust_model_paramters(self, start_pt = 0):
        diameters     = [5.50, 4.25] # mm
        lengths       = [40, 35] # mm
        motor_scales  = np.array([0.4,   0.4,  0.4,  0.4, # sheath
                                  0.27,  0.33, 0.28,  0.33, # leader
                                  1.5,  1.0])              # linear motors
        #wire placement about cath axis (Z) from X = 0
        thetas   = np.array([-1.50, 0.05,  2.00,  3.25, #sheath (not tested yet)
                             -1.80,-0.15,  1.45,  3.10]) # leader
        # ground frame orientation (R_zyz, Euler angles)
        # R_angles = np.array([0.075, np.pi/2 - 0.075, 0])
        R_angles = self.data.angles_model[start_pt, :]
        return [R_angles, thetas, motor_scales, lengths, diameters]

    def plot_xyz_model(self, 
                     x_index, # 3 is for time
                     y_index,
                     start_pt = 0, 
                     end_pt = -1, 
                     fig = [],
                     colors = [purple],
                     axis_limits = [],
                     alphas = [1.]):
        if not fig:
            fig = plt.figure()
        else:
            plt.figure(fig.number)

        x = self.model_x[start_pt:end_pt, x_index] if x_index < 6 else self.data.time_pts[start_pt:end_pt]
        y = self.model_x[start_pt:end_pt, y_index]

        plt.plot(x, y, '.', color = colors[0], label = 'model prediction', alpha = alphas[0])
        plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        
        plt.xlabel(axis_names[x_index])
        plt.ylabel(axis_names[y_index])

        if axis_limits:
            plt.axis(axis_limits)

        return fig

        

