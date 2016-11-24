'''
lung_plots.py 
handles plotting trajectories on the lung

J.Sganga 9/2/2016
'''

import sys
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib import gridspec
import matplotlib.lines as mlines

from mpl_toolkits.axes_grid1.inset_locator import inset_axes, zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import seaborn.apionly as sns # doesn't change the rcparams


from load_history import history_data
from figure_settings import icra
icra = icra()

sys.path.append('..')
from lung_fit.transform_lung import get_transformed_lung
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()

blue, green, red, purple, yellow, cyan = sns.color_palette("deep")[:]
default_colors = [blue, green, red, purple, yellow, cyan]
grey = sns.xkcd_palette(["greyish"])[0]

blues_cmap  = 'Blues'# 0x71a3a58

sheath_list = [0, 1, 2, 3]
leader_list = [4, 5, 6, 7]




class plot_lung(object):
    """holds history data from a lung run and plots it on the lung stl"""
    def __init__(self, history_files, run_names):
        self.run_list  = []
        for file in history_files:
            self.run_list.append(history_data(file = file))
        self.run_names = run_names
        self.get_lung()
        self.offset = np.array([44, -78, -110]) # mm, positions lung nicely relative to plotting axes
        self.lung -= self.offset 
        self.x_sensed  = [] # lists contain each run's array's
        self.x_desired = []
        for run in self.run_list:
            self.x_sensed.append(run.x_sensed[:,:3] - self.offset)
            self.x_desired.append(run.x_desired[:,:3] - self.offset)
        self.find_way_point_indeces()


    def get_lung(self): 
            self.lung, self.guide_trace = get_transformed_lung()
        
    def check_lung_fit(self):
        ax_xy = self.plot_lung_cloud(1, 0)
        ax_xy.plot(self.guide_trace[:,1], self.guide_trace[:,0], 'k.', alpha = 0.05)

        ax_xz = self.plot_lung_cloud(2, 0, with_trace = True)
        ax_xz.plot(self.guide_trace[:,2], self.guide_trace[:,0], 'k.', alpha = 0.05)
        return ax_xy, ax_xz

    def find_way_point_indeces(self):
        self.indeces = []
        for i_run, run in enumerate(self.run_list):
            index_list = []
            for data_pt in range(len(run.time_pts) - 1):
                if self.is_transition_point(i_run, data_pt):
                    index_list.append(data_pt)
            index_list.append(len(run.time_pts))
            self.indeces.append(index_list)
        self.indeces = np.asarray(self.indeces).T # columns = runs


    def is_transition_point(self, i_run, data_pt):
        dx_desired = self.x_desired[i_run][data_pt + 1, :] - self.x_desired[i_run][data_pt, :]
        if max(abs(dx_desired)) > 0.5:
            return True
        else:
            return False

    def plot_lung_cloud(self, 
                        x_index,
                        y_index,
                        ax = plt.subplot(111),
                        color = blue,
                        alpha = 0.002,
                        slice_index = 2,
                        slice_max = 1e3,
                        slice_min = -1e3,
                        do_slice = False,# pass in reduced lung if wanted
                        skip_n = 1): 
        if do_slice:
            lung = self.lung[(self.lung[:,slice_index] < slice_max) & (self.lung[:,slice_index] > slice_min)]
        else:
            lung = self.lung

        ax.plot(lung[::skip_n, x_index], lung[::skip_n, y_index], '.', color = color, alpha = alpha, markersize = 3)            
        ax.set_aspect('equal')
        ax.invert_yaxis()
        ax.invert_xaxis()
        sns.despine(ax = ax,  top=True, right=True, left=False, bottom=False)
        return ax

    def add_on_lung(self, 
                     ax,
                     i_run,
                     view = 'xy',
                     color = red,
                     alpha = 0.5,
                     linewidth = 2):
        if view.lower() == 'xy':
            x_index, y_index = 1, 0
        elif view.lower() == 'yz':
            x_index, y_index = 1, 2
        else:
            x_index, y_index = 2, 0
        start_pt = self.indeces[i_run][1]
        end_pt   = self.indeces[i_run][-1] #-1 is end of recording, includes after trajec..

        x = self.x_sensed[i_run][start_pt:end_pt, x_index]
        y = self.x_sensed[i_run][start_pt:end_pt, y_index]
        line, = ax.plot(x, y, '-', color = color, alpha = alpha, linewidth = linewidth)
        return line

    def add_desired_on_lung(self,
                            ax,
                            i_run,
                            view = 'xy',
                            color = green,
                            alpha = 0.1,
                            markersize = 5,
                            marker = 's'):
        if view.lower() == 'xy':
            x_index, y_index = 1, 0
        elif view.lower() == 'yz':
            x_index, y_index = 1, 2
        else:
            x_index, y_index = 2, 0
        start_pt = self.indeces[i_run][2]
        end_pt   = self.indeces[i_run][-2] #-1 is end of recording, includes after trajec..
        x = self.x_desired[i_run][start_pt:end_pt, x_index]
        y = self.x_desired[i_run][start_pt:end_pt, y_index]
        ax.plot(x, y, '.', marker = marker, color = color, alpha = alpha, markersize = markersize)


    def make_lung_figure(self, 
                         fig_width  = icra.col_width,
                         fig_height = icra.golden_height,
                         subplot_left  = 0.2,  # the left side of the subplots of the figure
                         subplot_right = 0.9,    # the right side of the subplots of the figure
                         subplot_bottom = 0.2,   # the bottom of the subplots of the figure
                         subplot_top = 0.9,      # the top of the subplots of the figure
                         subplot_wspace = -.3,   # the amount of width reserved for blank space between subplots
                         subplot_hspace = -.25,   # the amount of height reserved for white space between subplots
                         skip_n = 1, # skip n - 1 points when plotting lung dots
                         lung_color = blue,
                         lung_alpha = 0.0058,
                         lung_markersize = 2,
                         rasterize_lung = True, # allows pdfs to not save every single point 
                         rasterize_order = 0, # no idea what this does
                         run_colors = [blue, red, grey],
                         run_alphas = [0.7, 0.7, 0.7],
                         linewidth  = 2,
                         wp_color   = green,
                         wp_alpha   = 0.8,
                         wp_markersize = 4,
                         zoom_factor = 1.5,
                         bbox_tuple = (0.52, 0.6),
                         zoom_limits = (-73, -18, 120, 200),
                         legend_bbox = (1.65,1)):

        # make figure with subplots
        self.lung_fig = plt.figure(figsize=(fig_width, fig_height)) # w, h in inches
        gs = gridspec.GridSpec(1, 5) 
        split = 2#3
        ax_xy = plt.subplot(gs[:, split:])
        ax_xz = plt.subplot(gs[:, :split])
        plt.subplots_adjust(subplot_left, subplot_bottom, subplot_right, subplot_top, subplot_wspace, subplot_hspace)
        
        # XZ subplot
        lung_slice = self.lung[self.lung[:,1] < 0]
        self.plot_lung_on_subplot(ax_xz, 
                                  x_index = 2, 
                                  y_index = 0, 
                                  lung = lung_slice, 
                                  skip_n = skip_n, 
                                  lung_color = lung_color, 
                                  lung_alpha = lung_alpha, 
                                  lung_markersize = lung_markersize, 
                                  rasterize_lung = rasterize_lung, 
                                  rasterize_order = rasterize_order,
                                  hide_axes = [True, True, False, False])# [top, right, left, bottom] 
        axes_ticks = [-50, 0, 50]
        axes_ticks_names = ['-50','0','50']
        plt.setp(ax_xz, 
                 xticks = axes_ticks, 
                 xticklabels= axes_ticks_names)
        ax_xz.invert_xaxis()
        ax_xz.set_xlabel(icra.axis_labels[2], fontsize = icra.label_fontsize)
        ax_xz.set_ylabel(icra.axis_labels[0], fontsize = icra.label_fontsize)


        # XY subplot
        self.plot_lung_on_subplot(ax_xy, 
                                  x_index = 1, 
                                  y_index = 0, 
                                  lung = self.lung, 
                                  skip_n = skip_n, 
                                  lung_color = lung_color, 
                                  lung_alpha = lung_alpha, 
                                  lung_markersize = lung_markersize, 
                                  rasterize_lung = rasterize_lung, 
                                  rasterize_order = rasterize_order,
                                  hide_axes = [True, True, True, False])# [top, right, left, bottom] 
        ax_xy.invert_xaxis()
        ax_xy.set_xlabel(icra.axis_labels[1], fontsize = icra.label_fontsize)
        ax_xy.yaxis.set_ticks([])


        for i_run, run in enumerate(self.run_list):
            self.add_on_lung(ax_xy, i_run, view = 'xy',  color = run_colors[i_run], alpha = run_alphas[i_run], linewidth = linewidth)
            self.add_on_lung(ax_xz, i_run, view = 'xz', color = run_colors[i_run], alpha = run_alphas[i_run],  linewidth = linewidth)
        # only plottign the first run's desired points. make sure it reached all of them
        self.add_desired_on_lung(ax_xy, i_run = 0, view = 'xy', color = wp_color, alpha = wp_alpha, markersize = wp_markersize)
        self.add_desired_on_lung(ax_xz, i_run = 0, view = 'xz', color = wp_color, alpha = wp_alpha, markersize = wp_markersize)

        # zoomed axis
        ax_in = zoomed_inset_axes(ax_xy, 
                                  zoom_factor, # zoom-factor
                                  loc = 3, # not really sure, picks some location on the fig
                                  bbox_to_anchor = bbox_tuple, #allows fine tuned position control but not sure what they correspond to...
                                  bbox_transform = ax_xy.figure.transFigure) # strange command needed for bbox...
        ax_in.invert_xaxis()
        self.plot_lung_on_subplot(ax_in, 
                                  x_index = 1, 
                                  y_index = 0, 
                                  lung = self.lung, 
                                  skip_n = skip_n, 
                                  lung_color = lung_color, 
                                  lung_alpha = lung_alpha, 
                                  lung_markersize = lung_markersize, 
                                  rasterize_lung = rasterize_lung, 
                                  rasterize_order = rasterize_order,
                                  hide_axes = [False, False, False, False])# [top, right, left, bottom] 
        for i_run, run in enumerate(self.run_list):
            self.add_on_lung(ax_in, i_run, view = 'xy',  color = run_colors[i_run], alpha = run_alphas[i_run])
        self.add_desired_on_lung(ax_in, i_run = 0, view = 'xy', color = wp_color, alpha = wp_alpha)

        x1, x2, y1, y2 = zoom_limits # specify the limits
        ax_in.set_xlim(x1, x2) # apply the x-limits
        ax_in.set_ylim(y1, y2) # apply the y-limits
        ax_in.invert_yaxis()
        ax_in.invert_xaxis()
        ax_in.xaxis.set_ticks([])
        ax_in.yaxis.set_ticks([]) 
        # lines from parent to inset
        mark_inset(ax_xy, ax_in, loc1=2, loc2=4, fc="none", ec="0.5")

        self.line_labels = ['Way Points'] + self.run_names # for legend
        self.line_handles = [mlines.Line2D([0,0],[0,0],  marker = 's', color = wp_color, markersize = wp_markersize, alpha = wp_alpha, linestyle = '')]
        for i_run, run in enumerate(self.run_list):
            line = mlines.Line2D([0,0],[0,0], color = run_colors[i_run], linewidth = linewidth, alpha = run_alphas[i_run])
            self.line_handles.append(line)

        plt.legend(handles = self.line_handles, 
                    labels = self.line_labels,
                    loc = 'upper left', 
                    bbox_to_anchor = legend_bbox,
                    fontsize = icra.label_fontsize)

        return self.lung_fig



    def plot_lung_on_subplot(self, 
                             ax, 
                             x_index, 
                             y_index, 
                             lung,
                             skip_n = 1, # skip n - 1 points when plotting lung dots
                             lung_color = blue,
                             lung_alpha = 0.0058,
                             lung_markersize = 2,
                             rasterize_lung = True, # allows pdfs to not save every single point 
                             rasterize_order = 0,# no idea what this does
                             hide_axes = [True, True, False, False]): # [top, right, left, bottom] 
        ax.plot(lung[::skip_n, x_index], 
                lung[::skip_n, y_index], 
                '.', 
                color = lung_color, 
                alpha = lung_alpha, 
                markersize = lung_markersize)
        ax.set_rasterized(rasterize_lung)
        ax.set_rasterization_zorder(rasterize_order)
        ax.tick_params(labelsize=icra.label_fontsize)
        ax.set_xlim(min(lung[:,x_index]), max(lung[:,x_index]))
        ax.set_ylim(min(lung[:,y_index]), max(lung[:,y_index])) 
        ax.set_aspect('equal')
        ax.invert_yaxis()
        top, right, left, bottom = hide_axes
        sns.despine(ax = ax, top = top, right = right, left = left, bottom = bottom)


    def savefig(self, 
                dpi = 300, 
                file_format = 'pdf', 
                file_name = 'test'):
        self.lung_fig.savefig(icra.fig_folder + file_name + '.' + file_format, dpi = dpi)

   

        
            