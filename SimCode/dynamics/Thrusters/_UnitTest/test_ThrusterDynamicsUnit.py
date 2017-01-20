'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
# Thurster Unit Test
#
# Purpose:  Test the proper function of the Thruster Dynamics module.
#           This is done by comparing expected torques and forces to
#           what is sumulated
# Author:   Thibaud Teil
# Creation Date:  Dec. 20, 2016
#

# @cond DOXYGEN_IGNORE
import sys, os
import numpy as np
import ctypes
import math
import pytest
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')
# @endcond

#Import all of the modules that we are going to call in this simulation
import unitTestSupport
import matplotlib.pyplot as plt
import MessagingAccess
import SimulationBaseClass
import sim_model
import thrusterDynamicEffector
import stateArchitecture
import spacecraftPlus
import macros


def thrusterEffectorAllTests(show_plots):
   [testResults, testMessage] = test_unitThrusters(show_plots)

# Create function to run the simulation who's results will be compared to expected values
def executeSimRun(simContainer, thrusterSet, simRate, totalTime):
    newStopTime = simContainer.TotalSim.CurrentNanos + totalTime
    while(simContainer.TotalSim.CurrentNanos < newStopTime):
        simContainer.ConfigureStopTime(simContainer.TotalSim.CurrentNanos + simRate)
        simContainer.ExecuteSimulation()
        thrusterSet.computeBodyForceTorque(simContainer.TotalSim.CurrentNanos*1.0E-9)
        thrusterSet.computeBodyForceTorque(simContainer.TotalSim.CurrentNanos*1.0E-9 + simRate*1.0E-9/2.0)
        thrusterSet.computeBodyForceTorque(simContainer.TotalSim.CurrentNanos*1.0E-9 + simRate*1.0E-9/2.0)
        thrusterSet.computeBodyForceTorque(simContainer.TotalSim.CurrentNanos*1.0E-9 + simRate*1.0E-9)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("RAMP, ThrustNumber , Duration , Angle, Location, Rate", [
    ("OFF", 1 , 10.0 , 30. , [[1.125], [0.0], [2.0]], 1E8),
    ("OFF", 1 , 0.1, 30., [[1.125], [0.0], [2.0]], 1E8),
    ("OFF", 1, 0.1, 30., [[1.125], [0.0], [2.0]], 1E6),
    ("OFF", 1, 10.0, 30., [[1.125], [0.0], [2.0]], 1E7),
    ("OFF", 1 , 10.0 , 10. , [[1.125], [0.0], [2.0]], 1E8),
    ("OFF", 1 , 10.0 , 30. , [[1.], [0.0], [0.0]], 1E8),
    ("OFF", 2 , 10.0 , 30. , [[1.125], [0.0], [2.0]], 1E8)
    ])


# provide a unique test method name, starting with test_
def test_unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle , Location, Rate):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle , Location, Rate)
    assert testResults < 1, testMessage

# Run the test
def unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle, Location, Rate):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    # terminateSimulation() is needed if multiple unit test scripts are run
    # that run a simulation for the test. This creates a fresh and
    # consistent simulation environment for each test run.
    unitTestSim.TotalSim.terminateSimulation()

    #  Create thrusters
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thrusterSet.ModelTag = "ACSThrusterDynamics"

    #  Create thruster characteristic parameters (position, angle thrust, ISP, time of thrust)
    angledeg = Angle # Parametrized angle of thrust
    anglerad = angledeg*math.pi/180.0
    thruster1 = thrusterDynamicEffector.ThrusterConfigData()
    thruster1.inputThrLoc_S =Location # Parametrized location for thruster
    thruster1.inputThrDir_S = [[math.cos(anglerad)], [math.sin(anglerad)], [0.0]]
    thruster1.MaxThrust = 1.0
    thruster1.steadyIsp = 226.7
    thruster1.MinOnTime = 0.006
    thrusterSet.AddThruster(thruster1)

    if ThrustNumber==2:
        thruster2 = thrusterDynamicEffector.ThrusterConfigData()
        thruster2.inputThrLoc_S =[[1.], [0.0], [0.0]]
        thruster2.inputThrDir_S = [[math.cos(anglerad+math.pi/4)], [math.sin(anglerad+math.pi/4)], [0.0]]
        thruster2.MaxThrust = 1.0
        thruster2.steadyIsp = 226.7
        thruster2.MinOnTime = 0.006
        thrusterSet.AddThruster(thruster2)

    #  Create a Simulation
    testRate = int(Rate) # Parametrized rate of test
    TotalSim = SimulationBaseClass.SimBaseClass()
    #Create a new process for the unit test task and add the module to tasking
    testProc = TotalSim.CreateNewProcess(unitProcessName)
    testProc.addTask(TotalSim.CreateNewTask(unitTaskName, testRate))
    TotalSim.AddModelToTask(unitTaskName, thrusterSet)
    TotalSim.scObject = spacecraftPlus.SpacecraftPlus()
    TotalSim.scObject.ModelTag = "spacecraftBody"

    #  Create a task manager
    TotalSim.newManager = stateArchitecture.DynParamManager()
    #TotalSim.AddModelToTask("thrusterbasic", scObject)

    #  Define the start of the thrust and it's duration
    sparetime = 1.
    thrStartTime=sparetime*1./macros.NANO2SEC
    thrDurationTime=Duration*1./macros.NANO2SEC # Parametrized thrust duration

    #Configure a single thruster firing, create a message for it
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.forceExternal_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.torqueExternalPntB_B', testRate, 0, 2)
    if ThrustNumber==1:
        ThrustMessage = thrusterDynamicEffector.ThrustCmdStruct()
        ThrustMessage.OnTimeRequest = 0.
        thrMessageSize = ThrustMessage.getStructSize()
    if ThrustNumber==2:
        ThrustMessage = sim_model.new_doubleArray(2) #Create a C double array
        sim_model.doubleArray_setitem(ThrustMessage,1,0.)
        sim_model.doubleArray_setitem(ThrustMessage,0,0.)
        thrMessageSize = 8*ThrustNumber
    TotalSim.TotalSim.CreateNewMessage(unitProcessName,"acs_thruster_cmds", thrMessageSize, 2)
    TotalSim.InitializeSimulation()

    #Configure the hub and link states
    dcmBS = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    dcmName = "dcm_BS"
    TotalSim.newManager.createProperty(dcmName, dcmBS)
    TotalSim.scObject.hub.registerStates(TotalSim.newManager)
    thrusterSet.linkInStates(TotalSim.newManager)

    # Run the simulation
    executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
    if ThrustNumber==1:
        ThrustMessage.OnTimeRequest = thrDurationTime*macros.NANO2SEC
    if ThrustNumber==2:
        sim_model.doubleArray_setitem(ThrustMessage, 0, thrDurationTime * macros.NANO2SEC)
        sim_model.doubleArray_setitem(ThrustMessage, 1, thrDurationTime * macros.NANO2SEC)
    TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 0, ThrustMessage)
    executeSimRun(TotalSim, thrusterSet, testRate, int(thrDurationTime+sparetime*1./macros.NANO2SEC))

    # Gather the Force and Torque results
    thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
    thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')

    # Auto Generate LaTex Figures
    format = "width=0.5\\textwidth"

    PlotName = "Force_" +  str(ThrustNumber) + "Thrusters_" +  str(Duration)+ "s_" + str(Angle)+"deg_"+ "Loc"+str(Location[0])
    PlotTitle = "Force on Y with " + str(ThrustNumber) + " thrusters, for "  +  str(Duration)+ " sec at " + str(Angle)+" deg "

    plt.figure(1)
    plt.plot(thrForce[:,0]*macros.NANO2SEC, thrForce[:,2])
    plt.ylim(0,1)
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust Factor (-)')
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots==True:
        plt.show()

    PlotName = "Torque_" +  str(ThrustNumber) + "Thrusters_" +  str(Duration)+ "s_" + str(Angle)+"deg_"+ "Loc"+str(Location[0])
    PlotTitle = "Torque on X with " + str(ThrustNumber) + " thrusters, for "  +  str(Duration)+ " sec at " + str(Angle)+" deg "

    plt.figure(2)
    plt.plot(thrForce[:,0]*macros.NANO2SEC, thrTorque[:,1])
    plt.ylim(-2.,2.)
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust Torque (-)')
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots==True:
        plt.show()

    # Create expected Force to test against thrForce
    expectedpoints=np.zeros([3,np.shape(thrForce)[0]])
    for i in range(np.shape(thrForce)[0]):
        if (i<int(round(thrStartTime/testRate)) + 2): # Thrust fires 2 times steps after the pause of sim and restart
            expectedpoints[0,i] = 0.0
            expectedpoints[1,i] = 0.0
        if (i>int(round(thrStartTime/ testRate)) + 1 and i<int(round((thrStartTime+thrDurationTime)/ testRate)) + 2):
            if ThrustNumber == 1:
                expectedpoints[0,i] = math.cos(anglerad)
                expectedpoints[1,i] = math.sin(anglerad)
            else:
                expectedpoints[0, i] = math.cos(anglerad)+math.cos(anglerad+math.pi / 4)
                expectedpoints[1, i] = math.sin(anglerad)+math.sin(anglerad+math.pi / 4)
        else:
            expectedpoints[0, i] = 0.0
            expectedpoints[1, i] = 0.0

    # Modify expected values for comparison and define errorTolerance
    TruthForce = np.transpose(expectedpoints)
    ErrTolerance = 10E-9

    # Compare Force values
    testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force", testFailCount, testMessages)

    # Create expected Torque to test against thrTorque
    expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
    for i in range(np.shape(thrForce)[0]):
        if (i < int(round(thrStartTime/ testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
            expectedpointstor[0, i] = 0.0
            expectedpointstor[1, i] = 0.0
            expectedpointstor[2, i] = 0.0
        if (i>int(round(thrStartTime/ testRate)) + 1 and i<int(round((thrStartTime+thrDurationTime)/ testRate)) + 2):
            if ThrustNumber == 1:
                expectedpointstor[0, i] = -math.sin(anglerad)*thruster1.inputThrLoc_S[2][0] #Torque about x is arm along z by the force projected upon y
                expectedpointstor[1, i] = math.cos(anglerad)*math.sqrt(thruster1.inputThrLoc_S[2][0]**2+thruster1.inputThrLoc_S[1][0]**2 +thruster1.inputThrLoc_S[0][0]**2)*math.sin(math.atan(thruster1.inputThrLoc_S[2][0]/thruster1.inputThrLoc_S[0][0])) #Torque about x is arm along z by the force projected upon x
                expectedpointstor[2, i] = math.sin(anglerad)*thruster1.inputThrLoc_S[0][0] #Torque about z is arm along x by the force projected upon y
            else:
                expectedpointstor[0, i] = -math.sin(anglerad)*thruster1.inputThrLoc_S[2][0]\
                                          - math.sin(anglerad+math.pi/4)*thruster2.inputThrLoc_S[2][0]
                expectedpointstor[1, i] = math.cos(anglerad)*math.sqrt(thruster1.inputThrLoc_S[2][0]**2+thruster1.inputThrLoc_S[1][0]**2 +thruster1.inputThrLoc_S[0][0]**2)*math.sin(math.atan(thruster1.inputThrLoc_S[2][0]/thruster1.inputThrLoc_S[0][0])) \
                                          + math.cos(anglerad+math.pi / 4)*math.sqrt(thruster2.inputThrLoc_S[2][0]**2+thruster2.inputThrLoc_S[1][0]**2 +thruster2.inputThrLoc_S[0][0]**2)*math.sin(math.atan(thruster2.inputThrLoc_S[2][0]/thruster2.inputThrLoc_S[0][0]))
                expectedpointstor[2, i] = math.sin(anglerad)*thruster1.inputThrLoc_S[0][0] \
                                          + math.sin(anglerad+math.pi/4) * thruster2.inputThrLoc_S[0][0]
        else:
            expectedpointstor[0, i] = 0.0
            expectedpointstor[1, i] = 0.0
            expectedpointstor[2, i] = 0.0

    # Modify expected values for comparison and define errorTolerance
    TruthTorque = np.transpose(expectedpointstor)
    ErrTolerance = 10E-9

    # Compare Torque values
    # Compare Force values
    testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance, "Torque", testFailCount, testMessages)

    if testFailCount == 0:
        print "PASSED: " + " No ramp force and torque"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitThrusters(False)

#def executeSimulationRun(stopTime, stepTime, TotalSim, ACSThrusterDynObject, 
#    massPropsData, outputState):
#    newStop = TotalSim.TotalSim.CurrentNanos
#    while(TotalSim.TotalSim.CurrentNanos < stopTime):
#        ACSThrusterDynObject.ComputeDynamics(massPropsData, outputState,
#            newStop*1.0E-9 - stepTime*1.0E-9)
#        ACSThrusterDynObject.ComputeDynamics(massPropsData, outputState,
#            newStop*1.0E-9 - stepTime/2*1.0E-9)
#        ACSThrusterDynObject.ComputeDynamics(massPropsData, outputState,
#            newStop*1.0E-9 - stepTime/2*1.0E-9)
#        ACSThrusterDynObject.ComputeDynamics(massPropsData, outputState,
#            newStop*1.0E-9)
#        newStop += var1
#        TotalSim.TotalSim.SingleStepProcesses()
#        TotalSim.RecordLogVars()
#
#
##Instantiate a new set of thrusters using the thruster_dynamics module
#ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()
#
##Configure a single thruster inside this thruster module
#ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
#Thruster1 = thruster_dynamics.ThrusterConfigData()
#Thruster1.ThrusterLocation = thruster_dynamics.DoubleVector([1.125, 0.0, 2.0])
#Thruster1.ThrusterDirection = thruster_dynamics.DoubleVector([
#    math.cos(30.0*math.pi/180.0), math.sin(30.0*math.pi/180.0), 0.0])
#Thruster1.MaxThrust = 1.0
#Thruster1.steadyIsp = 226.7
#Thruster1.MinOnTime = 0.006
#ACSThrusterDynObject.AddThruster(Thruster1)
#
#ACSpropCM = [0.0, 0.0, 1.0]
#ACSpropMass = 1.0 #Made up!!!!
#ACSpropRadius = 46.0/2.0/3.2808399/12.0
#sphereInerita = 2.0/5.0*ACSpropMass*ACSpropRadius*ACSpropRadius
#ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
#ACSThrusterDynObject.objProps.Mass = ACSpropMass
#SimulationBaseClass.SetCArray(ACSpropCM, 'double', 
#    ACSThrusterDynObject.objProps.CoM)
#SimulationBaseClass.SetCArray(ACSInertia, 'double', 
#    ACSThrusterDynObject.objProps.InertiaTensor)
#
#earthGrav = 9.80665
#mDotExpect = Thruster1.MaxThrust/(Thruster1.steadyIsp*earthGrav)
#print mDotExpect
#
##Set the test step (var1) and the various stop times (var2-var4)
#var1 = int(1E7) # call the thread at 1000 Hz, let the thread do stuff at 10 Hz
#var2 = int(60*1E9) # stop after 60 seconds
#var3  = int(80*1E9) # next stop
#var4  = int(100*1E9) # next stop
#var5 = int(((110)*1E9)) #next stop
#var6 = int((110 + 0.02)*1E9) #next stop
#var7 = int(((120)*1E9)) #next stop
#var8 = int(((120+0.04)*1E9)) #next stop
#var9 = int(((130)*1E9)) #next stop
#var10 = int(((140)*1E9)) #next stop
#
##Create a sim module as an empty container
#TotalSim = SimulationBaseClass.SimBaseClass()
##Create a new process for the unit test task and add the module to tasking
#testProc = TotalSim.CreateNewProcess("TestProcess")
#testProc.addTask(TotalSim.CreateNewTask("thrusterbasic", var1))
#TotalSim.AddModelToTask("thrusterbasic", ACSThrusterDynObject)
#
##Configure the vehicle mass properties to use in the simulation
#massPropsData = six_dof_eom.MassPropsData()
#CoMLoc = [1.0, 0.0, 0.0]
#SimulationBaseClass.SetCArray(CoMLoc, 'double', massPropsData.CoM)
#T_str2Bdy = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
#SimulationBaseClass.SetCArray(T_str2Bdy, 'double', massPropsData.T_str2Bdy)
#outputState = six_dof_eom.OutputStateData()
#
##Setup variables for logging
#TotalSim.AddVectorForLogging('ACSThrusterDynamics.StrForce', 'double', 0, 2, int(1E2))
#TotalSim.AddVectorForLogging('ACSThrusterDynamics.BodyForce', 'double', 0, 2, int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor', int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.objProps.Mass', int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.mDotTotal', int(1E2))
#
##Configure a single thruster firing, create a message for it
#ThrustMessage = thruster_dynamics.ThrustCmdStruct()
#ThrustMessage.OnTimeRequest = 10.0;
#TotalSim.TotalSim.CreateNewMessage("TestProcess", "acs_thruster_cmds", 8, 2)
#
##Step the simulation for a bit to get clear of any init funnies
#TotalSim.InitializeSimulation()
#TotalSim.ConfigureStopTime(int(10*1E9-var1))
#TotalSim.ExecuteSimulation()
#
##Write firing command message and step the simulation
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 0, ThrustMessage);
#executeSimulationRun(var2, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#   
##Configure a shorter firing command, execute it, and step simulation 
#ThrustMessage.OnTimeRequest = 0.1;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, int(30E9), ThrustMessage);
#executeSimulationRun(var3, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#  
##Setup thruster ramp on and ramp off configuration
#rampOnList = []
#rampOffList = []
##Note that this ramp is totally linear and ramps up 30 ms using 30 steps
#for i in range(30):
#    newElement = thruster_dynamics.ThrusterTimePair()
#    newElement.TimeDelta = (i+1)*0.001
#    newElement.ThrustFactor = (i+1.0)/30.0
#    newElement.IspFactor = (i+1.0)/30.0
#    rampOnList.append(newElement)
#    newElement = thruster_dynamics.ThrusterTimePair()
#    newElement.TimeDelta = (i+1)*0.001
#    newElement.ThrustFactor = 1.0-(i+1.0)/30.0
#    newElement.IspFactor = newElement.ThrustFactor
#    rampOffList.append(newElement)
#
##Set up the ramps 
#ACSThrusterDynObject.ThrusterData[0].ThrusterOnRamp = \
#    thruster_dynamics.ThrusterTimeVector(rampOnList)
#ACSThrusterDynObject.ThrusterData[0].ThrusterOffRamp = \
#    thruster_dynamics.ThrusterTimeVector(rampOffList)
#
##Execute a new firing that will use the thruster ramps
#ThrustMessage.OnTimeRequest = 0.5;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 1, ThrustMessage);
#executeSimulationRun(var4, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.015;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 2, ThrustMessage);
#executeSimulationRun(var5, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.5;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 3, ThrustMessage);
#executeSimulationRun(var6, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.005;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 4, ThrustMessage);
#executeSimulationRun(var7, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.025;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 5, ThrustMessage);
#executeSimulationRun(var8, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.2;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 6, ThrustMessage);
#executeSimulationRun(var9, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.001;
#TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 7, ThrustMessage);
#executeSimulationRun(var10, var1, TotalSim, ACSThrusterDynObject,
#    massPropsData, outputState)
#
##Extract log variables and plot the results
#thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.StrForce')
#thrBodForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.BodyForce')
#thrFactor = TotalSim.GetLogVariableData('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor')
#thrMass = TotalSim.GetLogVariableData('ACSThrusterDynamics.objProps.Mass')
#thrMDot = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')
#plt.figure(1)
#plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,1], 'b', label='x Str')
#plt.plot(thrBodForce[:,0]*1.0E-9, thrBodForce[:,1], 'b--', label='x Body')
#plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,2], 'g', label='y Str')  
#plt.plot(thrBodForce[:,0]*1.0E-9, thrBodForce[:,2], 'g--', label='y Body')
#plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,3], 'r', label = 'z Str')
#plt.plot(thrBodForce[:,0]*1.0E-9, thrBodForce[:,3], 'r--', label='z Body')
#plt.legend()
#plt.xlabel('Time (s)')
#plt.ylabel('Thruster Force (N)')
#
#plt.figure(2)
#plt.plot(thrFactor[:,0]*1.0E-9, thrFactor[:,1])
#plt.xlabel('Time(s)')
#plt.ylabel('Thrust Factor (-)')
#
#plt.figure(3)
#plt.plot(thrMass[:,0]*1.0E-9, thrMass[:,1])
#plt.xlabel('Time (s)')
#plt.ylabel('Propellant Mass (kg)')
#
#plt.figure(4)
#plt.plot(thrMDot[:,0]*1.0E-9, thrMDot[:,1])
#plt.xlabel('Time (s)')
#plt.ylabel('Propellant Mass-Rate (kg/s)')
#
#ACSThrusterDynObject.InputCmds = "dummy_fire_not_work"
#TotalSim.InitializeSimulation()
#TotalSim.ConfigureStopTime(int(2E6))
#TotalSim.ExecuteSimulation()
#
#if(len(sys.argv) > 1):
#   if(sys.argv[1] == 'True'):
#      plt.show()
