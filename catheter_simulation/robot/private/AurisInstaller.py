##############################################################################
# Auris Surgical Robotics, Inc.
# (C) Copyright 2014. All Rights Reserved.
#
#
# \file       AurisInstaller.py
# \brief
#     Check we are running the right python with the right standard packages.
#     Then test and install any custom Auris packages.
#
# \date       Jan 2015
#
# \author     Ben Fredrickson
##############################################################################

import sys
import os
import subprocess
import shutil
from winreg import *

# auris directories
ROOT_DIR = os.getcwd() + "\\"
PACKAGE_DIR = ROOT_DIR + 'AurisPackages\\'
AURISLIB_DIR = ROOT_DIR + 'Libs\\' 
PRIVATE_DIR = AURISLIB_DIR + 'private\\'

# python directories
if (sys.prefix != "C:\\Anaconda3"):
    print("This is not Anaconda3, I don't know how to configure (%s)." % (sys.prefix))
    if (not force): raise Exception("Package Error")
PYLIB_DIR = sys.prefix + "\\Lib\\" 
SITE_PACKAGE = PYLIB_DIR + "site-packages\\"

# Auris files
PYAURIS_PYD = "AurisInterface.pyd"
PYAURIS_EGG = "AurisInterface-0.0.0-py3.4.egg-info"
PYAURIS_LOW = "AurisLowLevel.py"

def RegisterPy():
    # tweak as necessary
    version = sys.version[:3]
    installpath = sys.prefix

    regpath = "SOFTWARE\\Python\\PythonCore\\%s\\" % (version)
    installkey = "InstallPath"
    pythonkey = "PythonPath"
    pythonpath = "%s\\Lib;%s\\DLLs" % (
        installpath, installpath)
    auriskey = "AurisPath"
    helpkey = "%sHELP\\Main Python Documentation" % (regpath)
    helppath = "%s\\Doc\\python342.chm" % (installpath)

    try:
        reg = OpenKey(HKEY_LOCAL_MACHINE, regpath)
    except EnvironmentError:
        try:
            reg = CreateKey(HKEY_LOCAL_MACHINE, regpath)
        except:
            print("*** Unable to register Python. Please run as administrator.")
            raise Exception("Registry Error")
    try:
        SetValue(reg, installkey, REG_SZ, installpath)
        SetValue(reg, pythonkey, REG_SZ, pythonpath)
        SetValue(reg, auriskey, REG_SZ, AURISLIB_DIR)
    except:
        print("*** Unable to change registry keys. Please run as administrator.")
        CloseKey(reg)
        raise Exception("Registry Error")
    CloseKey(reg)


def CheckStandard(force):

    try:
        print("\tRegister: ... ", end="")
        RegisterPy()
        print("OK.")

        print("\tNumpy: ... ", end="")
        import numpy as np
        print("OK.")

        print("\tScipy: ... ", end="")
        import scipy as sp
        print("OK.")

        print("\tPylab: ... ", end="")
        import pylab as pl
        print("OK.")

        #print("\tPyGame: ... ", end="")
        #import pygame as pg
        #print("OK.")
    except:
        print("FAILED.")
        print("Standard package not properly configured, please download and install.", sys.exc_info()[0])
        if (not force): raise Exception("Package Error")


def CheckAuris(force):
    try:
        print("\tPyInterface: ... ", end="")
        if (not os.path.exists(PRIVATE_DIR + PYAURIS_PYD)):
            if (not force): raise Exception("No pyd in Auris Package.")
        if (not os.path.exists(SITE_PACKAGE + PYAURIS_PYD)):
            shutil.copy(PRIVATE_DIR + PYAURIS_PYD, SITE_PACKAGE)
            print("new pyd ... ", end="")
        if (os.path.getmtime(PRIVATE_DIR + PYAURIS_PYD) > os.path.getmtime(SITE_PACKAGE + PYAURIS_PYD)):
            os.remove(SITE_PACKAGE + PYAURIS_PYD)
            shutil.copy(PRIVATE_DIR + PYAURIS_PYD, SITE_PACKAGE)
            print("modified pyd ... ", end="")
        
        if (not os.path.exists(PRIVATE_DIR + PYAURIS_EGG)):
            if (not force): raise Exception("No egg in Auris Package.")
        if (not os.path.exists(SITE_PACKAGE + PYAURIS_EGG)):
            shutil.copy(PRIVATE_DIR + PYAURIS_EGG, SITE_PACKAGE)
            print("new egg ... ", end="")
        if (os.path.getmtime(PRIVATE_DIR + PYAURIS_EGG) > os.path.getmtime(SITE_PACKAGE + PYAURIS_EGG)):
            os.remove(SITE_PACKAGE + PYAURIS_EGG)
            shutil.copy(PRIVATE_DIR + PYAURIS_EGG, SITE_PACKAGE)
            print("modified egg ... ", end="")

        if (not os.path.exists(SITE_PACKAGE + PYAURIS_EGG) or
            not os.path.exists(SITE_PACKAGE + PYAURIS_PYD)):
            raise Exception("No pyd in Auris Package.")
        print("OK.")
        
    except:
        print("FAILED.")
        print(sys.exc_info())
        if (not force): raise Exception("No pyd in Auris Package.")

    try:
        print("\tAurisLowLevel: ... ", end="")
        if (not os.path.exists(PRIVATE_DIR + PYAURIS_LOW)):
            if (not force): raise Exception("No LowLevel in Auris Package.")
        if (not os.path.exists(PYLIB_DIR + PYAURIS_LOW)):
            shutil.copy(PRIVATE_DIR + PYAURIS_LOW, PYLIB_DIR)
            print("new low level ...", end="")
        if (os.path.getmtime(PRIVATE_DIR + PYAURIS_LOW) > os.path.getmtime(PYLIB_DIR + PYAURIS_LOW)):
            os.remove(PYLIB_DIR + PYAURIS_LOW)
            shutil.copy(PRIVATE_DIR + PYAURIS_LOW, PYLIB_DIR)
            print("modified low level ...", end="")
        print(sys.path)
        import AurisLowLevel
        print("OK.")
    except:
        print("FAILED.")
        print(sys.exc_info())
        print(PRIVATE_DIR + PYAURIS_PYD)
        print(os.path.exists(PRIVATE_DIR + PYAURIS_PYD))
        if (not force): raise Exception("No LowLevel in Auris Package.")
    
    try:
        print("\tPyAuris: ... ", end="")
        import IDM
        print("OK.")
    except:
        print("FAILED.")
        print(sys.exc_info())
        if (not force): raise Exception("Package Error")



if __name__ == "__main__":
    try:
        print("Testing Standard Packages.")
        CheckStandard(len(sys.argv) >= 2 and sys.argv[1] == "-f")
        print("Standard Packages Verified.")
    except:
        print("\nFailed to verify Standard Packages.");
        sys.stdout.flush()
        sys.exit(-1)

    try:
        print("Test Auris Packages:")
        CheckAuris(False)
        print("Auris Packages Verified.")
    except:
        print("\nFailed to verify Auris Packages.");
        print(sys.exc_info())
        sys.stdout.flush()
        sys.exit(-1)

