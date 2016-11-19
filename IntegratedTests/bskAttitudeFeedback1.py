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
#   Purpose:  Integrated test of the spacecraftPlus(), extForceTorque, simpleNav() and
#             MRP_Feedback() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#   Author:  Hanspeter Schaub
#   Creation Date:  Nov. 19, 2016
#

import pytest
import sys, os, inspect
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import ctypes
import math
import csv
import logging


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import macros

# import simulation related support
import spacecraftPlus
import sim_model
import ExtForceTorque
import simIncludeGravity

# import FSW Algorithm related support
import MRP_Steering
import vehicleConfigData






# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useTranslation, useRotation", [
     (True, True)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedback(show_plots, useTranslation, useRotation):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = bskAttitudeFeedback(
            show_plots, useTranslation, useRotation)
    assert testResults < 1, testMessage



def bskAttitudeFeedback(show_plots, useTranslation, useRotation):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()


    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))


    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]] # kg*m^2
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)


    # setup Earth Gravity Body
    earthGravBody = simIncludeGravity.addEarth()
    earthGravBody.isCentralBody = True          # ensure this is the central gravitational body
    earthEphemData = simIncludeGravity.addEarthEphemData()

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([earthGravBody])


    # setup extForceTorque module
    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[-1], [1], [-1]]
    extFTObject.extForce_B = [[1], [2], [3]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)








    #
    #   Setup data logging before the simulation is initialized
    #
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, macros.minute2nano(2.))


    # create simulation messages
    scSim.TotalSim.CreateNewMessage(simProcessName,
                                          earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 2)
    scSim.TotalSim.WriteMessageData(earthGravBody.bodyMsgName, 8+8*3+8*3+8*9+8*9+8+64, 0,
                                          earthEphemData)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])  # m - r_BN_N
    velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])  # m/s - v_BN_N
    sigmaRef.setState([[0.1], [0.2], [-0.3]])       # sigma_BN_B
    omegaRef.setState([[0.001], [-0.01], [0.03]])   # rad/s - omega_BN_B










    # #
    # #   Setup FSW process with attitude control module
    # #
    #
    # fswProcess = scSim.CreateNewProcess(fswProcessName)
    # # create the FSW task and specify the update time
    # fswProcess.addTask(scSim.CreateNewTask(fswTaskName, macros.sec2nano(0.1)))
    #
    # # Construct algorithm and associated C++ container
    # controlModuleConfig = MRP_Steering.MRP_SteeringConfig()
    # controlModuleWrap = scSim.setModelDataWrap(controlModuleConfig)
    # controlModuleWrap.ModelTag = "MRP_Steering"
    #
    # # Add test module to runtime call list
    # scSim.AddModelToTask(fswTaskName, controlModuleWrap, controlModuleConfig)
    #
    # # Initialize the test module configuration data
    # controlModuleConfig.inputGuidName = "inputGuidName"
    # controlModuleConfig.vehConfigInMsgName = "vehicleConfigName"
    # controlModuleConfig.outputDataName = "outputName"
    #
    # controlModuleConfig.K1 = 0.15
    # controlModuleConfig.K3 = 1.0
    # controlModuleConfig.Ki = 0.01
    # controlModuleConfig.P = 150.0
    # controlModuleConfig.omega_max = 1.5*macros.D2R
    # controlModuleConfig.integralLimit = 2./controlModuleConfig.Ki*0.1
    #
    # # vehicleConfigData Message:
    # inputMessageSize = 18*8+8                           # 18 doubles + 1 32bit integer
    # scSim.TotalSim.CreateNewMessage(fswProcessName, controlModuleConfig.vehConfigInMsgName,
    #                                       inputMessageSize, 2)
    # vehicleConfigOut = vehicleConfigData.vehicleConfigData()
    # I = [1000., 0., 0.,
    #      0., 800., 0.,
    #      0., 0., 800.]
    # vehicleConfigOut.ISCPntB_B = I
    # scSim.TotalSim.WriteMessageData(controlModuleConfig.vehConfigInMsgName,
    #                                 inputMessageSize,
    #                                 0, vehicleConfigOut)




    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(macros.minute2nano(10.))
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    dataSigma = scSim.pullMessageLogData(scObject.scStateOutMsgName+".sigma_BN", range(3))
    dataPos = scSim.pullMessageLogData(scObject.scStateOutMsgName+".r_BN_N", range(3))


    np.set_printoptions(precision=16)

    # Remove time zero from list
    dataPos = dataPos[1:len(dataPos),:]
    dataSigma = dataSigma[1:len(dataSigma),:]

    useExtForceTorque = 1
    if useTranslation==True:
        if useRotation == True and useExtForceTorque == 1:
            truePos = [
                          [-4.63213916e+06, 7.05701699e+06, 5.35807994e+06]
                        , [-5.21733880e+06, 6.58291016e+06, 5.43708908e+06]
                        , [-5.77264774e+06, 6.07105086e+06, 5.48494463e+06]
                        , [-6.29495833e+06, 5.52444552e+06, 5.50144157e+06]
                        , [-6.78136423e+06, 4.94628599e+06, 5.48655395e+06]
                    ]

        elif useRotation == True and useExtForceTorque == 2:
            truePos = [
                          [-4.63216819e+06, 7.05702511e+06, 5.35808835e+06]
                        , [-5.21743009e+06, 6.58296641e+06, 5.43713249e+06]
                        , [-5.77283297e+06, 6.07119737e+06, 5.48504872e+06]
                        , [-6.29527082e+06, 5.52472739e+06, 5.50163359e+06]
                        , [-6.78183900e+06, 4.94674963e+06, 5.48686274e+06]
                    ]
        else: # natural translation
            truePos = [
                        [-4.63215860e+06,   7.05702991e+06,   5.35808355e+06]
                        ,[-5.21739172e+06,   6.58298551e+06,   5.43711328e+06]
                        ,[-5.77274669e+06,   6.07124005e+06,   5.48500543e+06]
                        ,[-6.29511743e+06,   5.52480250e+06,   5.50155642e+06]
                        ,[-6.78159911e+06,   4.94686541e+06,   5.48674159e+06]
                        ]

    if useRotation==True:
        if useExtForceTorque > 0:
            trueSigma = [
                          [1.35231621e-01,  3.41155419e-01, -6.64963888e-01]
                        , [1.45248859e-01, -6.05831978e-01,  5.45084605e-01]
                        , [2.96168976e-01,  3.52365350e-01, -2.29346879e-01]
                        , [2.57514759e-01, -5.93356106e-01,  5.54508640e-01]
                        , [4.91025978e-01, -4.21586707e-01,  3.61459503e-01]
                        ]

        else: # natural dynamics without RW or thrusters
            trueSigma = [
                        [-3.38921912e-02,  -3.38798472e-01,   5.85609015e-01]
                        ,[ 2.61447681e-01,   6.34635269e-02,   4.70247303e-02]
                        ,[ 2.04899804e-01,   1.61548495e-01,  -7.03973359e-01]
                        ,[ 2.92545849e-01,  -2.01501819e-01,   3.73256986e-01]
                        ,[ 1.31464989e-01,   3.85929801e-02,  -2.48566440e-01]
                        ]

    # compare the module results to the truth values
    accuracy = 1e-8
    if useRotation == True:
        if (len(trueSigma) != len(dataSigma)):
            testFailCount += 1
            testMessages.append("FAILED: unequal sigma data array sizes\n")
        else:
            for i in range(0,len(trueSigma)):
                # check a vector values
                if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
                    testFailCount += 1
                    testMessages.append("FAILED:  Dynamics Mode failed attitude unit test at t=" + str(dataSigma[i,0]*macros.NANO2SEC) + "sec\n")

    if useTranslation==True:
        if (len(truePos) != len(dataPos)):
            testFailCount += 1
            testMessages.append("FAILED: unequal position data array sizes\n")
        else:
            for i in range(0,len(truePos)):
                # check a vector values
                if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
                    testFailCount += 1
                    testMessages.append("FAILED:  Dynamics Mode failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")


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
    test_bskAttitudeFeedback(False,       # show_plots
                             True,        # useTranslation
                             True         # useRotation
                           )

