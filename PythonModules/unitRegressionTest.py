#
#   Main Unit Regression Script
#

import sys
import os
import inspect
import math



#   initialize the unit test error counter
totalError = 0


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


sys.path.append(path + '/../ADCSAlgorithms/attControl/MRP_Steering/UnitTest/')
import MRP_SteeringUnitTest                         # module python unitTest name
totalError += MRP_SteeringUnitTest.runUnitTest()

sys.path.append(path + '/../ADCSAlgorithms/attControl/LowPassFilterTorqueCommand/UnitTest//')
import LowPassFilterTorqueCommandUnitTest           # module python unitTest name
totalError += LowPassFilterTorqueCommandUnitTest.runUnitTest()


sys.path.append(path + '/../ADCSAlgorithms/ moduleTemplate/ subModuleTemplate/UnitTest/')
import subModuleTemplateUnitTest                    # module python unitTest name
totalError += subModuleTemplateUnitTest.runUnitTest()


#
#   Summarize the regression test results
#
print "---------------------------------------------------"
if  totalError:
    print "FAILED: " + str(totalError) + " Errors were found"
else:
    print "SUCCESS: all unit tests were completed successfully."
print "---------------------------------------------------"
