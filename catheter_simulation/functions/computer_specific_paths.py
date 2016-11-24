'''
computer specific paths
this file allows each computer to specify the 
key folder locations e.g. Google Drive
'''
import os, getpass
class computer_paths(object):
    """paths for each of the computers I use"""
    def __init__(self):
        self.username = getpass.getuser()
        self.get_folders()

    def get_folders(self):
        if self.username == 'Auris':
            self.google_drive = 'C:/Users/Auris/Google Drive/'
        elif self.username == 'sganga' or self.username == 'WIN+sganga': # WIN from jupyter notebooks
            self.google_drive = 'C:/Users/Sganga/Google Drive/'
        else:
            self.google_drive = '/Users/jakesganga/Google Drive/'

        self.data_folder  = self.google_drive + 'Research/data/soft_robot/'
        self.boost_folder = self.google_drive + 'Research/code/boost_projects/'
        self.lung_folder  = self.google_drive + 'Research/data/lung_fit/'
        self.fig_folder   = self.google_drive + 'Research/figures/'