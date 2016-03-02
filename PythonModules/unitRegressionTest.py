'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
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
print "-------------------------------------------------------"
if  totalError:
    print "FAILED: " + str(totalError) + " Errors were found"
else:
    print "SUCCESS: all unit tests were completed successfully."
print "-------------------------------------------------------"
