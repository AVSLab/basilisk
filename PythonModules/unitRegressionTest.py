#
#   Main Unit Regression Script
#

import sys
import os
import inspect
import subprocess


totalError = 0

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


sys.path.append(path + '/../ADCSAlgorithms/attControl/MRP_Steering/UnitTest/')
import MRP_SteeringUnitTest
totalError += MRP_SteeringUnitTest.runUnitTest()

#sys.path.append(path + '/../ moduleTemplate/ subModuleTemplate/UnitTest/')
#import subModuleTemplate
#totalError += subModuleTemplate.runUnitTest()





print totalError