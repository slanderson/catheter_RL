import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt


def butter_lowpass(cutoff_frequency, signal_frequency, order=5):
    '''provides the coefficients for a butterworth low pass filter given the parameters'''
    nyquist = 0.5 * signal_frequency
    normal_cutoff_frequency = cutoff_frequency / nyquist
    b, a = butter(order, normal_cutoff_frequency, btype='lowpass', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff_frequency, signal_frequency, order=5):
    b, a = butter_lowpass(cutoff_frequency, signal_frequency, order=order)
    y = lfilter(b, a, data)
    return y

def filter_segment(raw_data, filtered_data, cutoff_frequency, signal_frequency, order = 2):
    '''real-time implementation of the low pass filter
    for IDM expect two sets of data in form of 100 Hz signals and 10 Hz signals, want same cutoff and order'''
    if cutoff_frequency:
        b, a = butter_lowpass(cutoff_frequency, signal_frequency, order=order)
    elif signal_frequency == 100: # default order = 2, cutoff = 5, signal = 100
        b = np.array([ 0.02008337,  0.04016673,  0.02008337]) 
        a = np.array([ 1.        , -1.56101808,  0.64135154])
        
    elif signal_frequency == 15:# default order = 2, cutoff = 5, signal = 15
        # b = np.array([ 0.46515308,  0.93030615,  0.46515308])
        # a = np.array([ 1.        ,  0.6202041 ,  0.24040821])
        b = np.array([ 0.10844744,  0.21689488,  0.10844744]) # cutoff 2
        a = np.array([ 1.        , -0.87727063,  0.31106039])
        

    # import pdb; pdb.set_trace()
    assert(len(raw_data) == order + 1)
    assert(len(filtered_data) == order)
    # [::-1] reverses the order of an arrray [start:stop:step]
    # a[0] is for the filtered_data[n], which is being returned, a[0] = 1
    # return b.dot(data_raw[::-1]) - a[1:].dot(filtered_data[::-1])
    return b[::-1].dot(raw_data) - a[:0:-1].dot(filtered_data)
        
'''filters the sensor inputs and commanded outputs to keep 
the time lag induced by the filter constant across the input/outputs'''
def filter_data(data_point, filter_array,  sensor_freq = [], cutoff_freq = [], filter_order = 2):
    '''
    low pass filters the data using a butterworth filter with 
    a cutoff freq, sensor frequency, and filter order defined in the 
    idm class. the filter array stores the necessary number of filtered
    and raw data points, zero pads  in the beginning. Calls the func 
    filter_segment in the FilterHelper.py and returns the filtered data point.
    '''
    raw_data, filtered_data = filter_array
    # order = min(self.history_index, self.filter_order)
    raw_data[-1] = data_point
    filtered_data[-1] = filter_segment(raw_data, filtered_data[:-1], cutoff_freq, 
                               sensor_freq, filter_order)
    # shuffling array to put most recent points in place
    raw_data[:-1] = raw_data[1:]
    filtered_data[:-1] = filtered_data[1:]
    return filtered_data[-1]