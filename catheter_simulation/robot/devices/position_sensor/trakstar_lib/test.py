import sys
import ctypes

from atc3dg_types import *

dll_name = "C:\\Ascension\\3D Guidance driveBAY2 (Rev D)\\3DG API Developer\\3DG API\\ATC3DG64.DLL"
_api = ctypes.CDLL(dll_name)
_api.InitializeBIRDSystem