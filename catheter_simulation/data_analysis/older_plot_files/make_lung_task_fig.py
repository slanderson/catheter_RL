
# ## Lung Environment Data
# Collect data from the insertion tasks that define the environments boundaries. 
# Need to specify which edge case matches the bend constraint.
# Unfortunately hard coded now, should be fixed later. 

import sys, time
import numpy as np
import matplotlib.pyplot as plt
from load_history import history_data


folder = '../data/model_data/lung_data/'
grey = [0.7, 0.725, 0.725]

def make_plot_boundaries(constraint):
    fig = plt.figure()
    file   = 'history_edge_0.p'
    data_outer = history_data(file, folder)
    
    file   = 'history_edge_pi_' + str(constraint) + '.p'
    data_inner = history_data(file, folder)
    # isolate the confined locations
    start_index = 0
    stop_index  = 0
    for i, q_i in enumerate(data_outer.q[:,8]):
        if q_i >= lung.tube_start:
            start_index = i
            break
    for i, q_i in enumerate(data_outer.q[:,8]):
        if q_i >= lung.tube_end:
            stop_index = i
            break
            
    stop_index -= 0        
    top    = np.ones(stop_index - start_index) * 40
    bottom = np.linspace(-10,0.025,10)
    
    plt.fill_between(data_outer.y[start_index:stop_index], 
                     data_outer.x[start_index:stop_index],
                     top, color = grey, alpha = 1, 
                     label = 'Lung Wall')
    plt.fill_between(bottom, 
                     40 * np.ones(len(bottom)),
                     color = grey, alpha = 1)
    start_index += 20
    stop_index += 70 
    plt.fill_between(data_inner.y[start_index:stop_index],
                     data_inner.x[start_index:stop_index], 
                     color = grey, alpha = 1)
    plt.xlabel('Y(mm)')
    plt.ylabel('X(mm)')
    plt.axis('equal')
    plt.axis((-10,40,-10,50))

    return fig
    