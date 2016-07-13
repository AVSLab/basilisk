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
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import spice_interface
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import unitTestSupport                  # general support file with common unit test functions
import setupUtilitiesRW                 # RW simulation setup utilties
import setupUtilitiesThruster           # Thruster simulation setup utilties
import reactionwheel_dynamics
import thruster_dynamics
import macros
import hinged_rigid_bodies
import fuel_tank
import vehicleConfigData







# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useTranslation, useRotation, useRW, useJitter, useThruster, useHinged, useFuelSlosh", [
    (True, True, False, False, False, False, False),
    (False, True, False, False, False, False, False),
    (True, False, False, False, False, False, False),
    (False, True, True, False, False, False, False),
    (False, True, True, True, False, False, False),
    (True, True, True, False, False, False, False),
    (True, True, True, True, False, False, False),
    (True, True, False, False, True, False, False),
    (True, True, False, False, False, True, False),
    (True, True, False, False, False, False, True),
    (True, True, False, False, False, True, True)
])

# provide a unique test method name, starting with test_
def test_unitDynamicsModes(show_plots, useTranslation, useRotation, useRW, useJitter, useThruster, useHinged, useFuelSlosh):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitDynamicsModesTestFunction(
            show_plots, useTranslation, useRotation, useRW, useJitter, useThruster, useHinged, useFuelSlosh)
    assert testResults < 1, testMessage



def unitDynamicsModesTestFunction(show_plots, useTranslation, useRotation, useRW, useJitter, useThruster, useHinged, useFuelSlosh):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    rwCommandName = "reactionwheel_cmds"
    thrusterCommandName = "acs_thruster_cmds"

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    DynUnitTestProc = scSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(scSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))
    
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

    # Initialize Spacecraft Data
    VehDynObject.ModelTag = "VehicleDynamicsData"
    VehDynObject.PositionInit = six_dof_eom.DoubleVector([-4020338.690396649,	7490566.741852513,	5248299.211589362])
    VehDynObject.VelocityInit = six_dof_eom.DoubleVector([-5199.77710904224,	-3436.681645356935,	1041.576797498721])
    #Note that the above position/velocity get overwritten by the ICs from the target ephemeris
    VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.1, 0.2, -.3])
    VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.001, -0.01, 0.03])
    VehDynObject.baseMass = 750.0
    VehDynObject.baseInertiaInit = six_dof_eom.DoubleVector([900, 0.0, 0.0,
                                                         0.0, 800.0, 0.0,
                                                         0.0, 0.0, 600.0])
    VehDynObject.baseCoMInit = six_dof_eom.DoubleVector([0.0, 0.0, 1.0])
    
    VehDynObject.AddGravityBody(EarthGravBody)
    
    VehDynObject.useTranslation = useTranslation
    VehDynObject.useRotation = useRotation

    if useThruster:
        # add thruster devices
        # The clearThrusterSetup() is critical if the script is to run multiple times
        setupUtilitiesThruster.clearThrusterSetup()
        setupUtilitiesThruster.createThruster(
                'MOOG_Monarc_1',
                [1,0,0],                # location in S frame
                [0,1,0]                 # direction in S frame
                )

        # create thruster object container and tie to spacecraft object
        thrustersDynObject = thruster_dynamics.ThrusterDynamics()
        setupUtilitiesThruster.addThrustersToSpacecraft("Thrusters",
                                                       thrustersDynObject,
                                                       VehDynObject)

        # setup the thruster mass properties
        thrusterPropCM = [0.0, 0.0, 1.2]
        thrusterPropMass = 40.0
        thrusterPropRadius = 46.0 / 2.0 / 3.2808399 / 12.0
        sphereInerita = 2.0 / 5.0 * thrusterPropMass * thrusterPropRadius * thrusterPropRadius
        thrusterPropInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
        thrustersDynObject.objProps.Mass = thrusterPropMass
        SimulationBaseClass.SetCArray(thrusterPropCM, 'double', thrustersDynObject.objProps.CoM)
        SimulationBaseClass.SetCArray(thrusterPropInertia, 'double', thrustersDynObject.objProps.InertiaTensor)

        # set thruster commands
        ThrustMessage = thruster_dynamics.ThrustCmdStruct()
        ThrustMessage.OnTimeRequest = 10.0
        scSim.TotalSim.CreateNewMessage(unitProcessName, thrusterCommandName, 8, 2)
        scSim.TotalSim.WriteMessageData(thrusterCommandName, 8, 0, ThrustMessage)

    if useRW:
        # add RW devices
        # The clearRWSetup() is critical if the script is to run multiple times
        setupUtilitiesRW.clearRWSetup()
        setupUtilitiesRW.options.useRWJitter = useJitter
        setupUtilitiesRW.options.maxMomentum = 100
        setupUtilitiesRW.createRW(
                'Honeywell_HR16',
                [1,0,0],                # gsHat_S
                0.0                     # RPM
                )
        setupUtilitiesRW.createRW(
                'Honeywell_HR16',
                [0,1,0],                # gsHat_S
                0.0                     # RPM
                )
        setupUtilitiesRW.createRW(
                'Honeywell_HR16',
                [0,0,1],                # gsHat_S
                0.0,                    # RPM
                [0.5,0.5,0.5]           # r_S (optional)
                )

        # create RW object container and tie to spacecraft object
        rwDynObject = reactionwheel_dynamics.ReactionWheelDynamics()
        setupUtilitiesRW.addRWToSpacecraft("ReactionWheels", rwDynObject, VehDynObject)

        # set RW torque command
        scSim.TotalSim.CreateNewMessage(unitProcessName, rwCommandName, 8*vehicleConfigData.MAX_EFF_CNT, 2)
        cmdArray = sim_model.new_doubleArray(vehicleConfigData.MAX_EFF_CNT)
        sim_model.doubleArray_setitem(cmdArray, 0, 0.020) # RW-1 [Nm]
        sim_model.doubleArray_setitem(cmdArray, 1, 0.010) # RW-2 [Nm]
        sim_model.doubleArray_setitem(cmdArray, 2,-0.050) # RW-3 [Nm]
        scSim.TotalSim.WriteMessageData(rwCommandName, 8*vehicleConfigData.MAX_EFF_CNT, 1, cmdArray )

    if useHinged:
        # add SPs
        VehDynObject.PositionInit = six_dof_eom.DoubleVector([0.0,	0.0, 0.0])
        VehDynObject.VelocityInit = six_dof_eom.DoubleVector([0.0,	0.0, 0.0])
        VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.0, 0.0, 0.0])
        VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.1, -0.1, 0.1])
        panelSet1 = hinged_rigid_bodies.HingedRigidBodies()
        panel1 = hinged_rigid_bodies.HingedRigidBodyConfigData()
        panel2 = hinged_rigid_bodies.HingedRigidBodyConfigData()

        # Define Variable for panel 1
        panel1.mass = 100
        SimulationBaseClass.SetCArray([100.0, 0.0, 0.0, 0.0, 50, 0.0, 0.0, 0.0, 50.0], "double", panel1.IPntS_S)
        panel1.d = 1.5
        panel1.k = 100.0
        panel1.c = 0.0
        SimulationBaseClass.SetCArray([0.5, 0, 1], "double", panel1.r_HB_B)
        SimulationBaseClass.SetCArray([-1, 0, 0, 0, -1, 0, 0, 0, 1], "double", panel1.HB)
        panel1.theta = 5.0*numpy.pi/180
        panel1.thetaDot = 0.0

        # Define Variables for panel 2
        panel2.mass = 100.0
        SimulationBaseClass.SetCArray([100.0, 0.0, 0.0, 0.0, 50, 0.0, 0.0, 0.0, 50.0], "double", panel2.IPntS_S)
        panel2.d = 1.5
        panel2.k = 100
        panel2.c = 0.0
        SimulationBaseClass.SetCArray([-0.5, 0, 1], "double", panel2.r_HB_B)
        SimulationBaseClass.SetCArray([1, 0, 0, 0, 1, 0, 0, 0, 1], "double", panel2.HB)
        panel2.theta = 0.0
        panel2.thetaDot = 0.0

        panelSet1.addHingedRigidBody(panel1)
        panelSet1.addHingedRigidBody(panel2)
        VehDynObject.addHingedRigidBodySet(panelSet1)

        VehDynObject.useGravity = False

    if useFuelSlosh:
        # add Fuel Slosh particles
        VehDynObject.PositionInit = six_dof_eom.DoubleVector([0.0,	0.0, 0.0])
        VehDynObject.VelocityInit = six_dof_eom.DoubleVector([0.0,	0.0, 0.0])
        VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.0, 0.0, 0.0])
        VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.1, -0.1, 0.1])
        fuelTank1 = fuel_tank.FuelTank()
        sloshParticle1 = fuel_tank.FuelSloshParticleConfigData()
        sloshParticle2 = fuel_tank.FuelSloshParticleConfigData()
        sloshParticle3 = fuel_tank.FuelSloshParticleConfigData()

        # Define Variables for fuel tank 1
        fuelTank1.fuelTankData.massFT = 45
        SimulationBaseClass.SetCArray([0, 0, 0], "double", fuelTank1.fuelTankData.r_TB_B)

        # Define Variables for particle 1
        sloshParticle1.massFSP = 10
        sloshParticle1.k = 100.0
        sloshParticle1.c = 0.0
        SimulationBaseClass.SetCArray([0.1, 0, -0.1], "double", sloshParticle1.r_PT_B)
        SimulationBaseClass.SetCArray([1, 0, 0], "double", sloshParticle1.pHat_B)
        sloshParticle1.rho = 0.05
        sloshParticle1.rhoDot = 0.0

        # Define Variables for particle 2
        sloshParticle2.massFSP = 20
        sloshParticle2.k = 100.0
        sloshParticle2.c = 0.0
        SimulationBaseClass.SetCArray([0, 0, 0.1], "double", sloshParticle2.r_PT_B)
        SimulationBaseClass.SetCArray([0, 1, 0], "double", sloshParticle2.pHat_B)
        sloshParticle2.rho = -0.025
        sloshParticle2.rhoDot = 0.0

        # Define Variables for particle 2
        sloshParticle3.massFSP = 15
        sloshParticle3.k = 100.0
        sloshParticle3.c = 0.0
        SimulationBaseClass.SetCArray([-0.1, 0, 0.1], "double", sloshParticle3.r_PT_B)
        SimulationBaseClass.SetCArray([0, 0, 1], "double", sloshParticle3.pHat_B)
        sloshParticle3.rho = -0.015
        sloshParticle3.rhoDot = 0.0

        fuelTank1.addFuelSloshParticle(sloshParticle1)
        fuelTank1.addFuelSloshParticle(sloshParticle2)
        fuelTank1.addFuelSloshParticle(sloshParticle3)
        VehDynObject.addFuelTank(fuelTank1)

        VehDynObject.useGravity = False

    # add objects to the task process
    if useRW:
        scSim.AddModelToTask(unitTaskName, rwDynObject)
    if useThruster:
        scSim.AddModelToTask(unitTaskName, thrustersDynObject)
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
    scSim.ConfigureStopTime(macros.sec2nano(60*10.)) #Just a simple run to get initial conditions from ephem
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
    dataPos = dataPos[1:len(dataPos),:]
    dataSigma = dataSigma[1:len(dataSigma),:]
    dataOrbitalEnergy = dataOrbitalEnergy[1:len(dataOrbitalEnergy),:]
    dataRotEnergy = dataRotEnergy[1:len(dataRotEnergy),:]
    dataOrbitalAngMom_N = dataOrbitalAngMom_N[1:len(dataOrbitalAngMom_N),:]
    dataRotAngMom_N = dataRotAngMom_N[1:len(dataRotAngMom_N),:]

    # Make all energy and momentum checks false
    checkOrbitalEnergy = False
    checkRotEnergy = False
    checkOrbitalAngMom = False
    checkRotAngMom = False

    if useTranslation==True:
        if useRotation==True and useJitter==True and useRW==True:
            truePos = [
                        [-4.63215860e+06,   7.05702991e+06,   5.35808355e+06]
                        ,[-5.21739172e+06,   6.58298551e+06,   5.43711328e+06]
                        ,[-5.77274667e+06,   6.07124005e+06,   5.48500542e+06]
                        ,[-6.29511736e+06,   5.52480249e+06,   5.50155640e+06]
                        ,[-6.78159897e+06,   4.94686538e+06,   5.48674157e+06]
                        ]
        elif useThruster: # thrusters with no RW
            truePos = [
                        [-4.63215747e+06,   7.05703053e+06,   5.35808358e+06]
                        ,[-5.21738942e+06,   6.58298678e+06,   5.43711335e+06]
                        ,[-5.77274323e+06,   6.07124197e+06,   5.48500554e+06]
                        ,[-6.29511282e+06,   5.52480503e+06,   5.50155656e+06]
                        ,[-6.78159336e+06,   4.94686853e+06,   5.48674175e+06]
                        ]
            # Energy has been shown to follow prescribed rate of change from power calculations (thrusters were only on
            # for 10 seconds so energy flat lines after 10 seconds)
            trueRotEnergy = [
                        [6.9678869224507189e-01]
                        ,[6.9678869224507478e-01]
                        ,[6.9678869224507345e-01]
                        ,[6.9678869224507367e-01]
                        ,[6.9678869224507400e-01]
                        ]
            checkRotEnergy = True
        elif useRotation==True and useHinged==True and useFuelSlosh==False: # Hinged Dynamics
            truePos = [
                        [-2.53742996478815, -2.84830940967875, 0.216182452815001]
                        ,[-5.38845294582063, -5.29411572920721, 0.0214874807180885]
                        ,[-7.8808739903179,  -8.17415157897987, 0.15545245637636]
                        ,[-10.5787631792684, -10.7111009304237, 0.143079770048844]
                        ,[-13.3842058040891, -13.2968874812058, 0.155873769585104]
                        ]
            trueOrbitalEnergy = [
                        [4.7918109527400965e-01]
                        ,[4.7918109957510380e-01]
                        ,[4.7918109488776639e-01]
                        ,[4.7918109843619439e-01]
                        ,[4.7918110109649925e-01]
                        ]
            checkOrbitalEnergy = True
            trueOrbitalAngMom_N = [
                        [4.7917767026013856e+00, -4.7790087509782637e+00, 1.2836457266194756e-02]
                        ,[4.7917772716953486e+00, -4.7790093413973604e+00, 1.2836080726626120e-02]
                        ,[4.7917758489453632e+00, -4.7790079200361397e+00, 1.2836398575007735e-02]
                        ,[4.7917749015813627e+00, -4.7790070578211328e+00, 1.2832212136338228e-02]
                        ,[4.7917763797456487e+00, -4.7790085092306143e+00, 1.2833132287383364e-02]
                        ]
            checkOrbitalAngMom = True
        elif useRotation==True and useFuelSlosh==True and useHinged==False: #Fuel slosh
            truePos = [
                        [-2.69102169e-02, -3.34138085e-02, -7.63296148e-03]
                        ,[-5.16421395e-02, -6.69635270e-02, -1.41925287e-02]
                        ,[-7.99673260e-02, -1.00807085e-01, -1.90481350e-02]
                        ,[-1.06932569e-01, -1.35397480e-01, -2.56727424e-02]
                        ,[-1.36388979e-01, -1.70517452e-01, -3.27473799e-02]
                        ]
            trueOrbitalEnergy = [
                        [5.3938499170122701e-05]
                        ,[5.3938864846988306e-05]
                        ,[5.3938833040934856e-05]
                        ,[5.3938757903208845e-05]
                        ,[5.3938676148061827e-05]
                        ]
            checkOrbitalEnergy = True
            trueOrbitalAngMom_N = [
                        [6.8248289813816810e-04, -5.0795202220321528e-04, -1.1163017030526147e-04]
                        ,[6.8249052064090670e-04, -5.0795592593507063e-04, -1.1164305727854804e-04]
                        ,[6.8245621189251212e-04, -5.0792548802023577e-04, -1.1166400157220160e-04]
                        ,[6.8242403545833207e-04, -5.0789477448558379e-04, -1.1169292736488284e-04]
                        ,[6.8254983591487352e-04, -5.0799756117381471e-04, -1.1166820345125832e-04]
                        ]
            checkOrbitalAngMom = True
        elif useRotation==True and useFuelSlosh==True and useHinged==True: #Fuel slosh and Hinged Dynamics
            truePos = [
                        [-2.44400224e+00, -2.74717376e+00, 2.02431956e-01]
                        ,[-5.18632064e+00, -5.11176972e+00, 1.26571309e-02]
                        ,[-7.58841749e+00, -7.88184532e+00, 1.28402447e-01]
                        ,[-1.01751925e+01, -1.03386042e+01, 1.21233539e-01]
                        ,[-1.28900259e+01, -1.28174649e+01, 1.15619983e-01]
                        ]
            trueOrbitalEnergy = [
                        [4.6624143741499813e-01]
                        ,[4.6624142524053780e-01]
                        ,[4.6624145887683388e-01]
                        ,[4.6624145375104342e-01]
                        ,[4.6624144577492410e-01]
                        ]
            checkOrbitalEnergy=True
            trueOrbitalAngMom_N = [
                        [4.6732026867934335e+00, -4.6500351095875478e+00, 1.5892539006208128e-03]
                        ,[4.6732034819224486e+00, -4.6500356934250329e+00, 1.5932567291891131e-03]
                        ,[4.6732071906428594e+00, -4.6500391847572224e+00, 1.6003600063276835e-03]
                        ,[4.6732020100349247e+00, -4.6500341143814889e+00, 1.5953215407686905e-03]
                        ,[4.6731977139204757e+00, -4.6500297441736365e+00, 1.6020368640132787e-03]
                        ]
            checkOrbitalAngMom = True
        else: # natural translation
            truePos = [
                        [-4.63215860e+06,   7.05702991e+06,   5.35808355e+06]
                        ,[-5.21739172e+06,   6.58298551e+06,   5.43711328e+06]
                        ,[-5.77274669e+06,   6.07124005e+06,   5.48500543e+06]
                        ,[-6.29511743e+06,   5.52480250e+06,   5.50155642e+06]
                        ,[-6.78159911e+06,   4.94686541e+06,   5.48674159e+06]
                        ]
            trueOrbitalEnergy = [
                        [-1.4947516556250010e+10]
                        ,[-1.4947516556249987e+10]
                        ,[-1.4947516556249828e+10]
                        ,[-1.4947516556249819e+10]
                        ,[-1.4947516556249727e+10]
                        ]
            checkOrbitalEnergy=True
            trueOrbitalAngMom_N = [
                        [1.9379050566179688e+13, -1.7326870952867449e+13, 3.9574426247581703e+13]
                        ,[1.9379050566179707e+13, -1.7326870952867498e+13, 3.9574426247581719e+13]
                        ,[1.9379050566179738e+13, -1.7326870952867598e+13, 3.9574426247581961e+13]
                        ,[1.9379050566179766e+13, -1.7326870952867605e+13, 3.9574426247581961e+13]
                        ,[1.9379050566179824e+13, -1.7326870952867600e+13, 3.9574426247582102e+13]
                        ]
            checkOrbitalAngMom = True
    if useRotation==True:
        if useRW==True:
            if useJitter==True:
                trueSigma = [
                            [-1.76673450e-01,  -4.44695362e-01,   7.55887814e-01]
                            ,[1.14010584e-01,  -3.22414666e-01,   6.83156574e-01]
                            ,[-2.24562827e-01,  -3.80868645e-01,   8.25282928e-01]
                            ,[2.17619286e-01,   2.60981727e-01,  -2.17974719e-01]
                            ,[-1.67297652e-02,  -1.74656576e-01,   4.79193617e-01]
                            ]
            else: # RW without jitter
                trueSigma = [
                            [-1.76673530e-01,  -4.44695395e-01,   7.55888029e-01]
                            ,[ 1.12819073e-01,  -3.23001268e-01,   6.84420942e-01]
                            ,[-2.29698563e-01,  -3.85908142e-01,   8.29297402e-01]
                            ,[ 2.20123922e-01,   2.50820626e-01,  -2.08093724e-01]
                            ,[-2.37038506e-02,  -1.91520817e-01,   4.94757184e-01]
                            ]
                # Energy has been verified to follow the prescribed rate of change from power calculations
                trueRotEnergy = [
                            [1.3634215656299600e+02]
                            ,[5.4398693685249941e+02]
                            ,[1.2233068084927652e+03]
                            ,[2.1742504368170212e+03]
                            ,[3.3968284498306407e+03]
                            ]
                checkRotEnergy = True
                trueRotAngMom_N = [
                            [-9.5287780855852944e-01, -1.5092028316403020e+01, 1.2654355186213355e+01]
                            ,[-9.5287780857909521e-01, -1.5092028316409420e+01, 1.2654355186204228e+01]
                            ,[-9.5287780854726467e-01, -1.5092028316411398e+01, 1.2654355186204333e+01]
                            ,[-9.5287780882898065e-01, -1.5092028316372639e+01, 1.2654355186229234e+01]
                            ,[-9.5287780847799741e-01, -1.5092028316600940e+01, 1.2654355185983071e+01]
                            ]
                checkRotAngMom = True
        elif useThruster==True: # thrusters with no RW
            trueSigma = [
                        [ 1.19430832e-01,  4.59254882e-01, -3.89066833e-01]
                        ,[ 2.08138788e-01,  4.05119853e-01, -7.15382716e-01]
                        ,[-1.20724242e-01,  2.93740864e-01, -8.36793478e-01]
                        ,[ 3.07086772e-01, -5.36449617e-01,  6.59728850e-01]
                        ,[ 1.18593650e-01, -3.64833149e-01,  4.52223736e-01]
                        ]
        elif useHinged==True and useTranslation==True and useFuelSlosh==False: #Hinged Dynamics
            trueSigma = [
                        [-0.394048228873186, -0.165364626297029, 0.169133385881799]
                        ,[0.118117327421145,  -0.0701596164151959, 0.295067094904904]
                        ,[-0.128038074707568, -0.325975851032536, -0.00401165652329442]
                        ,[-0.163301363833482, -0.366780426316819, 0.384007061361585]
                        ,[0.228198675962671, -0.329880460528557,  0.266599868549938]
                        ]
            trueRotEnergy = [
                        [2.3445746399941818e+01]
                        ,[2.3445743875805185e+01]
                        ,[2.3445741342364720e+01]
                        ,[2.3445738790119655e+01]
                        ,[2.3445736257681787e+01]
                        ]
            checkRotEnergy=True
            trueRotAngMom_N = [
                        [1.2583981794635505e+02, -1.8777858377774368e+02, 1.4768114245247938e+02]
                        ,[1.2583981760136209e+02, -1.8777858441336801e+02, 1.4768114318042959e+02]
                        ,[1.2583981764025231e+02, -1.8777858545601674e+02, 1.4768114287796766e+02]
                        ,[1.2583981858326324e+02, -1.8777858573322075e+02, 1.4768114265469842e+02]
                        ,[1.2583981919986654e+02, -1.8777858531140168e+02, 1.4768114358533944e+02]
                        ]
            checkRotAngMom = True
        elif useFuelSlosh==True and useTranslation==True and useHinged==False: #Fuel Slosh
            trueSigma = [
                        [-8.19602045e-02, -2.38564447e-01, 8.88132824e-01]
                        ,[2.86731068e-01, -4.36081263e-02, -7.20708425e-02]
                        ,[8.62854092e-03, -6.54746392e-01, 5.93541182e-01]
                        ,[5.69042347e-01, 9.22140866e-02, -3.06825156e-02]
                        ,[-2.46306074e-01, 8.25425414e-01, -3.37112618e-01]
                        ]
            trueRotEnergy = [
                        [1.1675465579266232e+01]
                        ,[1.1673286902296834e+01]
                        ,[1.1671143043899853e+01]
                        ,[1.1669033343674718e+01]
                        ,[1.1666957075118878e+01]
                        ]
            checkRotEnergy=True
            trueRotAngMom_N = [
                        [9.0069135339301980e+01, -8.0072812166013662e+01, 6.0061598702601444e+01]
                        ,[9.0069115383862737e+01, -8.0072794579006512e+01, 6.0061585388893562e+01]
                        ,[9.0069096528631817e+01, -8.0072777390512712e+01, 6.0061572484175265e+01]
                        ,[9.0069078579018239e+01, -8.0072761316733022e+01, 6.0061560487101630e+01]
                        ,[9.0069060775156075e+01, -8.0072744724956891e+01, 6.0061548781524650e+01]
                        ]
            checkRotAngMom = True
        elif useFuelSlosh==True and useTranslation==True and useHinged==True: #Fuel Slosh and Hinged Dynamics
            trueSigma = [
                        [-3.93694358e-01, -1.66142060e-01, 1.66953504e-01]
                        ,[1.15251028e-01, -7.13753632e-02, 2.98706088e-01]
                        ,[-1.18726297e-01, -3.23347370e-01, -2.75519994e-03]
                        ,[-1.74899479e-01, -3.73016665e-01, 3.73859155e-01]
                        ,[2.32402342e-01, -3.25845947e-01, 2.81360092e-01]
                        ]
            trueRotEnergy = [
                        [2.3634205370447170e+01]
                        ,[2.3632024766628916e+01]
                        ,[2.3629878703742676e+01]
                        ,[2.3627765492856582e+01]
                        ,[2.3625686935050652e+01]
                        ]
            checkRotEnergy=True
            trueRotAngMom_N = [
                        [1.2602821565134619e+02, -1.8798087281620440e+02, 1.4775387338882575e+02]
                        ,[1.2602820080709847e+02, -1.8798085268074720e+02, 1.4775385752169046e+02]
                        ,[1.2602818730677048e+02, -1.8798083236585873e+02, 1.4775384156156829e+02]
                        ,[1.2602817358923114e+02, -1.8798081487929164e+02, 1.4775382678456063e+02]
                        ,[1.2602816304352135e+02, -1.8798079656437807e+02, 1.4775381108506704e+02]
                        ]
            checkRotAngMom = True
        else: # natural dynamics without RW or thrusters
            trueSigma = [
                        [-3.38921912e-02,  -3.38798472e-01,   5.85609015e-01]
                        ,[ 2.61447681e-01,   6.34635269e-02,   4.70247303e-02]
                        ,[ 2.04899804e-01,   1.61548495e-01,  -7.03973359e-01]
                        ,[ 2.92545849e-01,  -2.01501819e-01,   3.73256986e-01]
                        ,[ 1.31464989e-01,   3.85929801e-02,  -2.48566440e-01]
                        ]
            trueRotEnergy = [
                        [3.1045000000000106e-01]
                        ,[3.1045000000000150e-01]
                        ,[3.1045000000000000e-01]
                        ,[3.1045000000000039e-01]
                        ,[3.1045000000000045e-01]
                        ]
            checkRotEnergy = True
            trueRotAngMom_N =[
                        [-9.5287780855568105e-01, -1.5092028316404720e+01, 1.2654355186211642e+01]
                        ,[-9.5287780855882609e-01, -1.5092028316405013e+01, 1.2654355186211070e+01]
                        ,[-9.5287780855992477e-01, -1.5092028316404509e+01, 1.2654355186211525e+01]
                        ,[-9.5287780855988968e-01, -1.5092028316404441e+01, 1.2654355186211628e+01]
                        ,[-9.5287780856247450e-01, -1.5092028316405106e+01, 1.2654355186210628e+01]
                        ]
            checkRotAngMom = True
    # compare the module results to the truth values
    accuracy = 1e-8
    if useRotation==True:
        for i in range(0,len(trueSigma)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed attitude unit test at t=" + str(dataSigma[i,0]*macros.NANO2SEC) + "sec\n")

    if useTranslation==True:
        for i in range(0,len(truePos)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed pos unit test at t=" + str(dataPos[i,0]*macros.NANO2SEC) + "sec\n")

    if checkOrbitalEnergy==True:
        accuracy = 1e-15
        for i in range(0,len(trueOrbitalEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataOrbitalEnergy[i],trueOrbitalEnergy[i],1,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed orbital energy unit test at t=" + str(dataOrbitalEnergy[i,0]*macros.NANO2SEC) + "sec\n")

    if checkRotEnergy==True:
        accuracy = 1e-15
        for i in range(0,len(trueRotEnergy)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataRotEnergy[i],trueRotEnergy[i],1,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed rotational energy unit test at t=" + str(dataRotEnergy[i,0]*macros.NANO2SEC) + "sec\n")

    if checkOrbitalAngMom==True:
        accuracy = 1e-15
        for i in range(0,len(trueOrbitalAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataOrbitalAngMom_N[i],trueOrbitalAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed orbital angular momentum unit test at t=" + str(dataOrbitalAngMom_N[i,0]*macros.NANO2SEC) + "sec\n")

    if checkRotAngMom==True:
        accuracy = 1e-15
        for i in range(0,len(trueRotAngMom_N)):
            # check a vector values
            if not unitTestSupport.isArrayEqualRelative(dataRotAngMom_N[i],trueRotAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED:  Dynamics Mode failed rotational angular momentum unit test at t=" + str(dataRotAngMom_N[i,0]*macros.NANO2SEC) + "sec\n")

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
    test_unitDynamicsModes(False,       # show_plots
                           True,       # useTranslation
                           True,        # useRotation
                           False,        # useRW
                           False,        # useJitter
                           False,       # useThruster
                           True,       # useHinged
                           True       # useFuelSlosh
                           )

