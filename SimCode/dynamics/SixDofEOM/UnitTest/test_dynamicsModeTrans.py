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
#   Integrated Unit Test Script
#   Purpose:  Run a test of the unit dynamics modes
#   Author:  Hanspeter Schaub
#   Creation Date:  May 22, 2016
#


import sys, os, inspect
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import csv
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import spice_interface
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import unitTestSupport                  # general support file with common unit test functions




# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
def test_unitDynamicsModes(show_plots):     # update "subModule" in this function name to reflect the module name
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitDynamicsModesTestFunction(show_plots)
    assert testResults < 1, testMessage



def unitDynamicsModesTestFunction(show_plots):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)


    scSim = SimulationBaseClass.SimBaseClass()
    DynUnitTestProc = scSim.CreateNewProcess("DynUnitTestProcess")
    DynUnitTestProc.addTask(scSim.CreateNewTask("sixDynTestTask", unitTestSupport.sec2nano(10.)))
    
    VehDynObject = six_dof_eom.SixDofEOM()
    spiceObject = spice_interface.SpiceInterface()
    
    #Initialize the ephemeris module
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = splitPath[0] + '/Basilisk/External/EphemerisData/'
    spiceObject.UTCCalInit = "2014 March 27, 14:00:00.0"
    spiceObject.OutputBufferCount = 2
    spiceObject.PlanetNames = spice_interface.StringVector(
        ["earth"])
    spiceObject.zeroBase = "earth"
    
    mu_earth = 0.3986004415E+15 # [m^3/s^2]
    reference_radius_earth = 0.6378136300E+07 # [m]
    EarthGravBody = six_dof_eom.GravityBodyData()
    EarthGravBody.BodyMsgName = "earth_planet_data"
    EarthGravBody.outputMsgName = "earth_display_frame_data"
    EarthGravBody.IsCentralBody = True
    EarthGravBody.mu = mu_earth
    
    VehDynObject.ModelTag = "VehicleDynamicsData"
    VehDynObject.PositionInit = six_dof_eom.DoubleVector([-4020338.690396649,	7490566.741852513,	5248299.211589362])
    VehDynObject.VelocityInit = six_dof_eom.DoubleVector([-5199.77710904224,	-3436.681645356935,	1041.576797498721])
    #Note that the above position/velocity get overwritten by the ICs from the target ephemeris
    VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.1, 0.2, -.3])
    VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.001, -0.01, 0.03])
    VehDynObject.baseMass = 1500.0 - 812.3
    VehDynObject.baseInertiaInit = six_dof_eom.DoubleVector([900, 0.0, 0.0,
                                                         0.0, 800.0, 0.0,
                                                         0.0, 0.0, 600.0])
    VehDynObject.T_Str2BdyInit = six_dof_eom.DoubleVector([1.0, 0.0, 0.0,
                                                           0.0, 1.0, 0.0,
                                                           0.0, 0.0, 1.0])
    VehDynObject.baseCoMInit = six_dof_eom.DoubleVector([0.0, 0.0, 1.0])
    
    VehDynObject.AddGravityBody(EarthGravBody)
    
    VehDynObject.useTranslation = True;        # turn on translational mode 
    VehDynObject.useRotation = False;          # turn on rotational mode 
    
    scSim.AddModelToTask("sixDynTestTask", spiceObject)
    scSim.AddModelToTask("sixDynTestTask", VehDynObject)

    scSim.TotalSim.logThisMessage("inertial_state_output", unitTestSupport.sec2nano(120.))
    
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(unitTestSupport.sec2nano(10*60.)) #Just a simple run to get initial conditions from ephem
    scSim.ExecuteSimulation()
    

    dataSigma = scSim.pullMessageLogData("inertial_state_output.sigma", range(3))
    # set the  output truth states
    trueVector = [
            [0.0, 0.0, 0.0]
            ,[0.0, 0.0, 0.0]
            ,[ 0.0, 0.0, 0.0]
            ,[  0.0, 0.0, 0.0]
            ,[ 0.0, 0.0, 0.0]
            ,[0.0, 0.0, 0.0]
               ]    
    # compare the module results to the truth values
    accuracy = 1e-9
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataSigma[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Dynamics Mode (Trans) failed  attitude unit test at t=" + str(dataSigma[i,0]*unitTestSupport.NANO2SEC) + "sec\n")
    
    scPos = scSim.pullMessageLogData("inertial_state_output.r_N", range(3))
    # set the  output truth states
    trueVector = [
            [-4.02033869e+06,   7.49056674e+06,   5.24829921e+06]
            ,[ -4.63215860e+06,   7.05702991e+06,   5.35808355e+06]
            ,[ -5.21739172e+06,   6.58298551e+06,   5.43711328e+06]
            ,[ -5.77274669e+06,   6.07124005e+06,   5.48500543e+06]
            ,[ -6.29511743e+06,   5.52480250e+06,   5.50155642e+06]
            ,[ -6.78159911e+06,   4.94686541e+06,   5.48674159e+06]
               ]    
    # compare the module results to the truth values
    accuracy = 1e-2
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(scPos[i],trueVector[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Dynamics Mode (Trans) failed  pos unit test at t=" + str(scPos[i,0]*unitTestSupport.NANO2SEC) + "sec\n")
    
    #   print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED " 

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]
                                                
                                                    
#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitDynamicsModes(False)           # update "subModule" in function name

