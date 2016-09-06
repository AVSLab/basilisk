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
#   Author:  John Alcorn
#   Creation Date:  August 31, 2016
#

import pytest
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
sys.path.append(splitPath[0]+'/Basilisk/modules')
sys.path.append(splitPath[0]+'/Basilisk/PythonModules')

import spice_interface
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import unitTestSupport  # general support file with common unit test functions
import macros
import vehicleConfigData
import simDynEffectorTemplate


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag", [
    (False),
    (True),
])
# provide a unique test method name, starting with test_
def test_unitSimDynEffectorTemplate(show_plots, useFlag):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimDynEffectorTemplate(show_plots, useFlag)
    assert testResults < 1, testMessage


def unitSimDynEffectorTemplate(show_plots, useFlag):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    DynUnitTestProc = scSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(scSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    VehDynObject = six_dof_eom.SixDofEOM()
    spiceObject = spice_interface.SpiceInterface()

    # Initialize the ephemeris module
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = splitPath[0]+'/Basilisk/External/EphemerisData/'
    spiceObject.UTCCalInit = "2014 March 27, 14:00:00.0"
    spiceObject.OutputBufferCount = 2
    spiceObject.PlanetNames = spice_interface.StringVector(
        ["earth"])
    spiceObject.zeroBase = "earth"

    mu_earth = 0.3986004415E+15  # [m^3/s^2]
    reference_radius_earth = 0.6378136300E+07  # [m]
    EarthGravBody = six_dof_eom.GravityBodyData()
    EarthGravBody.BodyMsgName = "earth_planet_data"
    EarthGravBody.outputMsgName = "earth_display_frame_data"
    EarthGravBody.IsCentralBody = True
    EarthGravBody.mu = mu_earth

    # Initialize Spacecraft Data
    VehDynObject.ModelTag = "VehicleDynamicsData"
    VehDynObject.PositionInit = six_dof_eom.DoubleVector([-4020338.690396649, 7490566.741852513, 5248299.211589362])
    VehDynObject.VelocityInit = six_dof_eom.DoubleVector([-5199.77710904224, -3436.681645356935, 1041.576797498721])
    # Note that the above position/velocity get overwritten by the ICs from the target ephemeris
    VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.1, 0.2, -.3])
    VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.001, -0.01, 0.03])
    VehDynObject.baseMass = 750.0
    VehDynObject.baseInertiaInit = six_dof_eom.DoubleVector([900, 0.0, 0.0,
                                                             0.0, 800.0, 0.0,
                                                             0.0, 0.0, 600.0])
    VehDynObject.baseCoMInit = six_dof_eom.DoubleVector([0.0, 0.0, 1.0])

    VehDynObject.AddGravityBody(EarthGravBody)

    useTranslation = True
    useRotation = True

    VehDynObject.useTranslation = useTranslation
    VehDynObject.useRotation = useRotation


    testModule = simDynEffectorTemplate.simDynEffectorTemplate()
    testModule.ModelTag = "externalDisturbance"
    testModule.inputVehProps = "spacecraft_mass_props"
    if useFlag:
        SimulationBaseClass.SetCArray([1, 2, 3], 'double', testModule.extForce_B)
        SimulationBaseClass.SetCArray([-1, 1, -1], 'double', testModule.extTorquePntB_B)
    else:
        SimulationBaseClass.SetCArray([0, 0, 0], 'double', testModule.extForce_B)
        SimulationBaseClass.SetCArray([0, 0, 0], 'double', testModule.extTorquePntB_B)
    VehDynObject.addBodyEffector(testModule)


    # add objects to the task process
    scSim.AddModelToTask(unitTaskName, spiceObject)
    scSim.AddModelToTask(unitTaskName, VehDynObject)

    scSim.AddVariableForLogging("VehicleDynamicsData.totScOrbitalEnergy", macros.sec2nano(120.))
    scSim.AddVariableForLogging("VehicleDynamicsData.totScRotEnergy", macros.sec2nano(120.))
    scSim.AddVectorForLogging('VehicleDynamicsData.totScOrbitalAngMom_N', 'double', 0, 2, macros.sec2nano(120.))
    # scSim.AddVariableForLogging("VehicleDynamicsData.totScOrbitalAngMomMag", macros.sec2nano(120.))
    scSim.AddVectorForLogging('VehicleDynamicsData.totScRotAngMom_N', 'double', 0, 2, macros.sec2nano(120.))
    # scSim.AddVariableForLogging("VehicleDynamicsData.totScRotAngMomMag", macros.sec2nano(120.))
    # scSim.AddVariableForLogging("VehicleDynamicsData.scRotEnergyRate", macros.sec2nano(0.001))
    # scSim.AddVariableForLogging("VehicleDynamicsData.scRotPower", macros.sec2nano(0.001))

    scSim.TotalSim.logThisMessage("inertial_state_output", macros.sec2nano(120.))

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(60*10.))  # Just a simple run to get initial conditions from ephem
    scSim.ExecuteSimulation()

    # log the data
    dataSigma = scSim.pullMessageLogData("inertial_state_output.sigma", range(3))
    dataPos = scSim.pullMessageLogData("inertial_state_output.r_N", range(3))

    dataOrbitalEnergy = scSim.GetLogVariableData("VehicleDynamicsData.totScOrbitalEnergy")
    dataRotEnergy = scSim.GetLogVariableData("VehicleDynamicsData.totScRotEnergy")
    dataOrbitalAngMom_N = scSim.GetLogVariableData("VehicleDynamicsData.totScOrbitalAngMom_N")
    # dataOrbitalAngMomMag = scSim.GetLogVariableData("VehicleDynamicsData.totScOrbitalAngMomMag")
    dataRotAngMom_N = scSim.GetLogVariableData("VehicleDynamicsData.totScRotAngMom_N")
    # dataRotAngMomMag = scSim.GetLogVariableData("VehicleDynamicsData.totScRotAngMomMag")
    # dataRotEnergyRate = scSim.GetLogVariableData("VehicleDynamicsData.scRotEnergyRate")
    # dataRotPower = scSim.GetLogVariableData("VehicleDynamicsData.scRotPower")

    numpy.set_printoptions(precision=16)
    # plt.figure(1)
    # plt.plot(dataRotEnergy[:,0]*1.0E-9, dataRotEnergy[:,1]-dataRotEnergy[0, 1], 'b')
    # plt.figure(2)
    # plt.plot(dataOrbitalEnergy[:,0]*1.0E-9, dataOrbitalEnergy[:,1]-dataOrbitalEnergy[0, 1], 'b')
    # plt.figure(2)
    # plt.plot(dataOrbitalAngMomMag[:,0]*1.0E-9, dataOrbitalAngMomMag[:,1]-dataOrbitalAngMomMag[0, 1], 'b')
    # plt.figure(3)
    # plt.plot(dataOrbitalAngMom_N[:,0]*1.0E-9, dataOrbitalAngMom_N[:,1]-dataOrbitalAngMom_N[0, 1], dataOrbitalAngMom_N[:,0]*1.0E-9, dataOrbitalAngMom_N[:,2]-dataOrbitalAngMom_N[0, 2], dataOrbitalAngMom_N[:,0]*1.0E-9, dataOrbitalAngMom_N[:,3]-dataOrbitalAngMom_N[0, 3])
    # plt.figure(4)
    # plt.plot(dataRotAngMomMag[:,0]*1.0E-9, dataRotAngMomMag[:,1]-dataRotAngMomMag[0, 1], 'b')
    # plt.figure(5)
    # plt.plot(dataRotAngMom_N[:,0]*1.0E-9, dataRotAngMom_N[:,1]-dataRotAngMom_N[0, 1], dataRotAngMom_N[:,0]*1.0E-9, dataRotAngMom_N[:,2]-dataRotAngMom_N[0, 2], dataRotAngMom_N[:,0]*1.0E-9, dataRotAngMom_N[:,3]-dataRotAngMom_N[0, 3])
    # # plt.figure(2)
    # # # plt.plot(dataRotEnergyRate[:,0]*1.0E-9, dataRotEnergyRate[:,1], 'b', dataRotPower[:,0]*1.0E-9, dataRotPower[:,1], 'r')
    # # # plt.figure(3)
    # # # plt.plot(dataRotEnergyRate[1:len(dataRotEnergyRate),0]*1.0E-9, dataRotEnergyRate[1:len(dataRotEnergyRate),1]-dataRotPower[1:len(dataRotEnergyRate),1], 'r')
    # plt.show()

    # Remove time zero from list
    dataPos = dataPos[1:len(dataPos), :]
    dataSigma = dataSigma[1:len(dataSigma), :]
    dataOrbitalEnergy = dataOrbitalEnergy[1:len(dataOrbitalEnergy), :]
    dataRotEnergy = dataRotEnergy[1:len(dataRotEnergy), :]
    dataOrbitalAngMom_N = dataOrbitalAngMom_N[1:len(dataOrbitalAngMom_N), :]
    dataRotAngMom_N = dataRotAngMom_N[1:len(dataRotAngMom_N), :]


    # Make all energy and momentum checks false
    checkOrbitalEnergy = False
    checkRotEnergy = False
    checkOrbitalAngMom = False
    checkRotAngMom = False

    if useTranslation == True:

        if useRotation == True and useFlag == 1:
            truePos = [
                 [-4.6321391621199939e+06, 7.0570169944370082e+06, 5.3580799406825136e+06]
                ,[-5.2173388015798805e+06, 6.5829101590330312e+06, 5.4370890838074163e+06]
                ,[-5.7726477425343171e+06, 6.0710508563772691e+06, 5.4849446330703171e+06]
                ,[-6.2949583322797501e+06, 5.5244455247360282e+06, 5.5014415700772973e+06]
                ,[-6.7813642252033837e+06, 4.9462859907889264e+06, 5.4865539514755039e+06]
            ]

        else:  # natural translation
            truePos = [
                [-4.63215860e+06, 7.05702991e+06, 5.35808355e+06]
                , [-5.21739172e+06, 6.58298551e+06, 5.43711328e+06]
                , [-5.77274669e+06, 6.07124005e+06, 5.48500543e+06]
                , [-6.29511743e+06, 5.52480250e+06, 5.50155642e+06]
                , [-6.78159911e+06, 4.94686541e+06, 5.48674159e+06]
            ]

    if useRotation == True:

        if useFlag:
            trueSigma = [
                 [1.3523162075523859e-01,  3.4115541930043958e-01, -6.6496388832945341e-01]
                ,[1.4524885918114658e-01, -6.0583197832064450e-01,  5.4508460523282254e-01]
                ,[2.9616897551309407e-01,  3.5236535047232992e-01, -2.2934687931489461e-01]
                ,[2.5751475949631308e-01, -5.9335610598839650e-01,  5.5450864007852685e-01]
                ,[4.9102597829083811e-01, -4.2158670685411598e-01,  3.6145950285894157e-01]
            ]

        else:  # natural dynamics without RW or thrusters
            trueSigma = [
                [-3.38921912e-02, -3.38798472e-01, 5.85609015e-01]
                , [2.61447681e-01, 6.34635269e-02, 4.70247303e-02]
                , [2.04899804e-01, 1.61548495e-01, -7.03973359e-01]
                , [2.92545849e-01, -2.01501819e-01, 3.73256986e-01]
                , [1.31464989e-01, 3.85929801e-02, -2.48566440e-01]
            ]

    # compare the module results to the truth values
    accuracy = 1e-8
    if useRotation == True:
        for i in range(0, len(trueSigma)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataSigma[i], trueSigma[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed attitude unit test at t="+str(
                    dataSigma[i, 0]*macros.NANO2SEC)+"sec\n")

    if useTranslation == True:
        for i in range(0, len(truePos)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataPos[i], truePos[i], 3, accuracy):
                testFailCount += 1
                testMessages.append(
                    "FAILED:  Dynamics Mode failed pos unit test at t="+str(dataPos[i, 0]*macros.NANO2SEC)+"sec\n")

    if checkOrbitalEnergy == True:
        accuracy = 1e-15
        for i in range(0, len(trueOrbitalEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataOrbitalEnergy[i], trueOrbitalEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed orbital energy unit test at t="+str(
                    dataOrbitalEnergy[i, 0]*macros.NANO2SEC)+"sec\n")

    if checkRotEnergy == True:
        accuracy = 1e-15
        for i in range(0, len(trueRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataRotEnergy[i], trueRotEnergy[i], 1, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed rotational energy unit test at t="+str(
                    dataRotEnergy[i, 0]*macros.NANO2SEC)+"sec\n")

    if checkOrbitalAngMom == True:
        accuracy = 1e-15
        for i in range(0, len(trueOrbitalAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataOrbitalAngMom_N[i], trueOrbitalAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed orbital angular momentum unit test at t="+str(
                    dataOrbitalAngMom_N[i, 0]*macros.NANO2SEC)+"sec\n")

    if checkRotAngMom == True:
        accuracy = 1e-15
        for i in range(0, len(trueRotAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataRotAngMom_N[i], trueRotAngMom_N[i], 3, accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed rotational angular momentum unit test at t="+str(
                    dataRotAngMom_N[i, 0]*macros.NANO2SEC)+"sec\n")

    # print out success message if no error were found
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
    test_unitSimDynEffectorTemplate(False   # show_plots
                           )

