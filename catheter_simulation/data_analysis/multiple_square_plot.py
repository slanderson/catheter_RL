'''
averaged square plot averages at least 3 runs on the robot on 
the square task. this will generate a figure with an average and stdev shaded
region around it.

relies on common_plots

J. Sganga 8/31/16
'''
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib import rc, rcParams
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from process_runs import multiple_runs # parent class

# seaborn's default colors
default_colors = [(0.2980392156862745, 0.4470588235294118, 0.6901960784313725),
                  (0.3333333333333333, 0.6588235294117647, 0.40784313725490196),
                  (0.7686274509803922, 0.3058823529411765, 0.3215686274509804),
                  (0.5058823529411764, 0.4470588235294118, 0.6980392156862745)]
blue, green, red, purple = default_colors
grey = (0.6588235294117647, 0.6431372549019608, 0.5843137254901961) # xkcd greyish
axis_font_size = 14

class square_plot(multiple_runs):
    """docstring for average_square_plot"""
    def __init__(self, file_names, complete_run = True):
        super(square_plot, self).__init__(file_names, complete_run)

    def plot_all_traces(self, ax = [], color = default_colors):
        if not ax:
            ax = plt.subplot(111)
        for i_run, run in enumerate(self.run_list):
            ax.plot(run.x_sensed[self.start_pts[i_run]:self.end_pts[i_run],1], 
                     run.x_sensed[self.start_pts[i_run]:self.end_pts[i_run],2], 
                     '-', 
                     color = color[i_run], 
                     alpha = 0.25, 
                     linewidth = 1,
                     label = 'Run ' + str(i_run))
        # plt.figure(fig.number)
        # # plt.plot(self.radial_avg[:,1], 
        # #          self.radial_avg[:,2], 
        # #          linewidth = 4,
        # #          alpha = 0.9,
        # #          label = 'radial avg')
        # plt.xlabel('Y (mm)')
        # plt.ylabel('Z (mm)')
        # plt.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        return ax

    def plot_shades(self, ax = [], color = blue, include_traces = True, include_labels = True):
        if not ax:
            ax = plt.subplot(111)
        ax.plot(self.run_list[0].x_desired[self.indeces[2:, 0],1], 
                 self.run_list[0].x_desired[self.indeces[2:, 0],2], 
                 '.-', 
                 color = green, 
                 alpha = 0.3, 
                 markersize = 3,
                 marker = 's',
                 label = r'\textbf{Way Points}')

        if include_traces:
            self.plot_all_traces(ax, color = [color] * len(self.run_list))

        xs = np.vstack((self.radial_min[:,1], self.radial_max[:,1]))
        ys = np.vstack((self.radial_min[:,2], self.radial_max[:,2]))
        xs[1,:] = xs[1,::-1]
        ys[1,:] = ys[1,::-1]
        ax.fill(np.ravel(xs), np.ravel(ys), color = color, edgecolor = color, alpha = 0.3, label = r'\textbf{Range}')
        ax.plot(self.radial_avg[:-1,1], self.radial_avg[:-1,2], color = color, label = r'\textbf{Average}')
        ax.set_aspect('equal')
        if include_labels: 
            self.add_labels(ax)
        return ax

    def add_labels(self, ax):
        ax.set_xlabel(r'\textbf{Y (mm)}', fontsize = axis_font_size)
        ax.set_ylabel(r'\textbf{Z (mm)}', fontsize = axis_font_size)
        ax.legend(loc = 'upper left', bbox_to_anchor = [0.2,0.45])
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)

       
    def plot_angles(self, ax = plt.subplot(111)):
        self.angles = np.zeros((min(self.end_pts - self.start_pts), len(self.run_list), 3))
        for run_ix, run in enumerate(self.run_list):
            self.angles[:,run_ix,:] = run.angles_sensed[self.start_pts[run_ix]:self.start_pts[run_ix] + len(self.angles)].copy()
        self.avg_angles = np.mean(self.angles, axis = 1) * 180/np.pi
        self.max_angles = max(self.angles, axis = 1) * 180/np.pi
        self.min_angles = min(self.angles, axis = 1) * 180/np.pi
        t = run.time_pts[:min(self.end_pts - self.start_pts)].flatten()
        t -= t[0]
        for i in range(3):
            ax.fill_between(t, self.max_angles[:,i], self.min_angles[:,i], alpha = 0.5, color = default_colors[i])
        angle_list = [r'$\alpha$', r'$\beta$', r'$\gamma$']
        model_offset = np.array([1, 1, 1.]) * 180/np.pi
        for i in range(3):
            ax.plot(t, self.avg_angles[:,i], color = default_colors[i], label = angle_list[i] + ' sensed')
            ax.plot(t, run.angles_model[self.start_pts[run_ix]:self.start_pts[run_ix] + len(self.angles),i] * model_offset[i], 
                     '--', color = default_colors[i], label = angle_list[i] + ' true')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Base Orientation (degrees)')
        ax.legend(loc = 'upper left', bbox_to_anchor = [1,1])
        return ax





        

