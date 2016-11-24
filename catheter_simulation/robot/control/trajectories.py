'''
Builds the trajectories for the robot to follow. They dictate the position/orientation
of the tip of the robot. 

All units in mm and degrees!

J. Sganga
3/19/16
'''

import numpy as np
from robot.model.environment import environment 

def get_trajectory(name_traj, steps = [], cycles = [], steps_to_ins = []):
    if name_traj.lower() == 'square':
        trajectory = make_square(steps, cycles, steps_to_ins)
    if name_traj.lower() == 'circle':
        trajectory = make_circle(steps, cycles, steps_to_ins)
    if name_traj.lower() == 'line':
        trajectory = make_line(steps)
    if name_traj.lower() == 'lung':
        trajectory = make_lung(steps)
    if name_traj.lower() == 'real lung':
        trajectory = make_real_lung(steps)
    return trajectory


def make_square(steps = 10, cycles = 1, steps_to_ins = 10):

    depth = 10
    side_length_y = 15
    side_length_z = 15
    az_angle = 0
    el_angle = np.pi/4

    trajectory = [
        np.array([depth,  side_length_y, side_length_z, np.pi / 3, 0]),
        np.array([depth, -side_length_y, side_length_z, 0, -np.pi / 6]),
        np.array([depth, -side_length_y,-side_length_z, -np.pi / 3, np.pi / 6]),
        np.array([depth,  side_length_y,-side_length_z, 0, -np.pi / 6]),
        np.array([depth,  side_length_y, side_length_z, 0, 0])
        ]
    trajectory = addStepsBetween(trajectory, steps)
    trajectory.append(trajectory[-1]) # last point is doubled to make sure it's hit
    trajectory *= (cycles)
    trajectory = addInsertion(trajectory, steps_to_ins)
    return trajectory

def make_circle(steps = 10, cycles = 2, steps_to_ins = 10):

    # trajectory circle
    radius = 10
    depth = 10
    az_angle = 0
    el_angle = 0

    phi = np.linspace(0, 2 * np.pi, steps)
    phi = np.append(phi, phi[-1]) # repeating last trajectory point so control loop completes it
    z_points = radius * np.cos(phi) 
    y_points = radius * np.sin(phi)
    trajectory = [np.array([depth, y, z, az_angle, el_angle]) for y, z in zip(y_points, z_points)]
    # y = 0
    # z = 0
    # az_points = np.linspace(-np.pi/4, np.pi/4, steps)
    # trajectory = [np.array([depth, y, z, az, el_angle]) for az in az_points]
    trajectory *= (cycles)
    trajectory = addInsertion(trajectory, steps_to_ins)
    return trajectory

def make_line(steps = 2):
    depth = 20
    trajectory  = [np.array([0, 0, 0, 0, 0]),
                   np.array([0, depth, 0, 0, 0]),
                   np.array([0, depth, 0, 0, 0])]
    trajectory = addStepsBetween(trajectory, steps)
    return trajectory

def make_lung(steps = 3):
    env = environment()
    trajectory = env.getEnvTrajectory()
    trajectory = addStepsBetween(trajectory, steps)
    trajectory.append(trajectory[-1]) # last point is doubled to make sure it's hit
    return trajectory

def make_real_lung(steps = 1): # in global frame!!!
    trajectory  = [np.array([135, -75, -100, 0, 0]),
                   # np.array([155, -85, -100, 0, 0]),
                   # np.array([168, -90, -102, 0, 0]),
                   np.array([178, -100,-110, 0, 0]),
                   # np.array([183, -110,-100, 0, 0]),
                   # np.array([190, -112, -109, 0, 0]),
                   np.array([200, -118, -106, 0, 0]),
                   np.array([220, -125, -102, 0, 0]),
                   np.array([240, -127, -95, 0, 0]),
                   np.array([240, -127, -95, 0, 0])]
    trajectory = addStepsBetween(trajectory, steps)
    return trajectory


def addStepsBetween(trajectory, steps):
    gaps_to_fill = len(trajectory)
    trajectory.append(trajectory[-1]) # leaves out last point if ignored
    trajectory_btwn = list(np.zeros(steps * gaps_to_fill))
    ix_counter = 0
    for pt in range(gaps_to_fill):
        for i in range(steps):
            frac = (steps - i) / steps
            trajectory_btwn[ix_counter] = frac * trajectory[pt] + (1 - frac) * trajectory[pt+1]
            ix_counter += 1
    return trajectory_btwn


def addInsertion(trajectory, steps_to_ins):
    trajectory_ins = list(np.zeros(steps_to_ins))
    initial_point = np.zeros(5)
    for i in range(steps_to_ins):
        frac = (steps_to_ins - i) / steps_to_ins
        trajectory_ins[i] = frac * initial_point + (1 - frac) * trajectory[0]

    return trajectory_ins + trajectory






