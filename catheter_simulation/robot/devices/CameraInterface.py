'''
This file runs the camera loop using the functions exposed by the EssentiallyFujikura.pyd, which is a 
boosted version of the FujikuraWrapper. This takes video from the leader tip, processes it according to
the feature tracking algorithms and displays the image at roughly 30 Hz. It is run on a separate core 
than the robot IDM using the multiprocessing module.  Through global variables, this script outputs the 
delta x and y (camera frame) and takes as input to plot the Jacobian and q values.

Jake Sganga
11/5/15
'''
import sys
import time
import numpy as np
import pickle
from functions.computer_specific_paths import computer_paths
computer_paths = computer_paths()
sys.path.append(computer_paths.boost_folder + 'EssentiallyFujikura/x64/Debug')
import EssentiallyFujikura

def CameraThread(global_variables, save_images = False):
    print('Started camera thread')
    sys.stdout.flush()

    dx = global_variables[0]
    J  = global_variables[1]
    q  = global_variables[2]
    x  = global_variables[3]
    flags = global_variables[4]
    force = global_variables[6]

    print(J[:])

    fuji = EssentiallyFujikura.FujikuraWrapper()
    fuji.Initialize()                 #Initialize the Camera
    fuji.CaptureAndDetect()            #captures an image, grabs features, saves the list internally.
    fuji.Cleanup()                   #pushes most recent features to previous feature.
    image_counter = 0
    image_threshold = 700
    wait_time = 0.1 # seconds between captures
    image_pose = np.zeros((image_threshold, 8))

    while True:
        now = time.perf_counter()
        fuji.CaptureAndDetect()        #captures an image, grabs features, saves the list internally.
        fuji.MatchFeaturesAndEstimateMotion() #match the features between newest frame and the previous feature set, return matched keys and overall displacement estimated.
        if save_images:
            # record the pose sent to the camera thread
            if image_counter < image_threshold:
                fuji.saveFrame()
                image_pose[image_counter,0]  = image_counter
                image_pose[image_counter,1:] = x[:]
                t_wait = time.perf_counter()
                while((time.perf_counter() - t_wait) < wait_time):
                    pass
            image_counter += 1
            if image_counter == image_threshold:
                with open( "data/camera_pose.p", "wb" ) as output:
                    pickle.dump(image_pose, output) 
        #plot the results
        key_press = fuji.PlotCurrentFeatures(np.asarray(J[:]), np.asarray(q[:]), np.asarray(x[:]), np.asarray(flags[:]), np.asarray(force[:]))
        
        dx[:] = [fuji.GetDx(), fuji.GetDy(), fuji.GetDz(), 0, 0]
        if flags[3]:
            fuji.Cleanup()             #pushes most recent features to previous feature.
            flags[3] = 0
        if(key_press == 27): break             #Check keyboard for quitting (ESC)
        

    #close camera
    fuji.TearDown();
    