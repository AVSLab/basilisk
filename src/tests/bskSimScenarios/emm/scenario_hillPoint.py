# Import some architectural stuff that we will probably always use
import sys
import os
import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# This part definitely needs work.  Need to detect Basilisk somehow.

print "local_path= ", path
sys.path.append(path + "/../../../../../" + "EMM/EMMModules")
import EMMDynModels
import EMMFSWModels