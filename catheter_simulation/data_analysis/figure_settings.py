'''
figure_settings.py
holds the settings and other useful parameters for making 
figures ready for publication

J.Sganga 9/2/2016
'''

import numpy as np
import sys, time
sys.path.append('..')
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()

class icra(object):
    """holds icra/ieee settings"""
    def __init__(self):
        super(icra, self).__init__()

    col_width = 3.5 # in
    col_double_width = 7 + 1/16 # in  

    golden_height  = col_width / 1.618 # golden ratio

    label_fontsize = 8 # 8 pt

    dpi_high = 1200
    dpi_mid  = 300

    fig_folder = computer_paths.fig_folder + 'icra_2017/'

    x_label  = r'\textbf{X (mm)}'
    y_label  = r'\textbf{Y (mm)}'
    z_label  = r'\textbf{Z (mm)}'
    axis_labels = [x_label, y_label, z_label]
                