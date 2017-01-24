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

# Create function to create truth points for 1 thruster
def expected1(Shape, thrDurationTime, thrStartTime, testRate, anglerad, thruster1):
    expectedpoints = np.zeros([3, Shape])
    for i in range(Shape):
        if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
            expectedpoints[0, i] = 0.0
            expectedpoints[1, i] = 0.0
        if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime) / testRate)) + 2):
            expectedpoints[0, i] = math.cos(anglerad)
            expectedpoints[1, i] = math.sin(anglerad)
        else:
            expectedpoints[0, i] = 0.0
            expectedpoints[1, i] = 0.0
    expectedpointstor = np.zeros([3, Shape])
    for i in range(Shape):
        if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
            expectedpointstor[0, i] = 0.0
            expectedpointstor[1, i] = 0.0
            expectedpointstor[2, i] = 0.0
        if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime) / testRate)) + 2):
            expectedpointstor[0, i] = -math.sin(anglerad) * thruster1.inputThrLoc_S[2][0]  # Torque about x is arm along z by the force projected upon y
            expectedpointstor[1, i] = math.cos(anglerad) * math.sqrt(thruster1.inputThrLoc_S[2][0] ** 2 + thruster1.inputThrLoc_S[1][0] ** 2 + thruster1.inputThrLoc_S[0][0] ** 2) * math.sin(math.atan(thruster1.inputThrLoc_S[2][0] / thruster1.inputThrLoc_S[0][0]))  # Torque about x is arm along z by the force projected upon x
            expectedpointstor[2, i] = math.sin(anglerad) * thruster1.inputThrLoc_S[0][0]  # Torque about z is arm along x by the force projected upon y
        else:
            expectedpointstor[0, i] = 0.0
            expectedpointstor[1, i] = 0.0
            expectedpointstor[2, i] = 0.0
        return np.transpose(expectedpoints), np.transpose(expectedpointstor)

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


@pytest.mark.parametrize("RAMP, ThrustNumber , Duration , Angle, Location, Rate, Cutoff, RampDown", [
    ("OFF", 1 , 5.0 , 30. , [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF"), #Test random thrust config
    ("OFF", 1 , 0.1, 30., [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF"), # Short fire test
    ("OFF", 1, 0.1, 30., [[1.125], [0.0], [2.0]], 1E6, "OFF", "OFF"), # Short fire test with higher test rate
    ("OFF", 1, 5.0, 30., [[1.125], [0.0], [2.0]], 1E7, "OFF", "OFF"),# Rate test
    ("OFF", 1 , 5.0 , 10. , [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF"), # Angle test
    ("OFF", 1 , 5.0 , 30. , [[1.], [0.0], [0.0]], 1E8, "OFF", "OFF"),# Position test
    ("OFF", 2 , 5.0 , 30. , [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF"), # Number of thrusters test
    ("ON", 1 , 5.0 , 30. , [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF") , # Basic ramp test
    ("ON", 1 , 0.5 , 30. , [[1.125], [0.0], [2.0]], 1E8, "OFF", "OFF") , # Short ramp test
    ("ON", 1 , 5.0 , 30. , [[1.125], [0.0], [2.0]], 1E7, "OFF", "OFF"), # Rate ramp test
    ("ON", 1 , 5.0 , 30. , [[1.125], [0.0], [2.0]], 1E8, "ON", "OFF"), # Cuttoff test
    ("ON", 1, 5.0, 30., [[1.125], [0.0], [2.0]], 1E8, "ON", "ON")  # Rampdown test
    ])


# provide a unique test method name, starting with test_
def test_unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle , Location, Rate, Cutoff, RampDown):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle , Location, Rate, Cutoff, RampDown)
    assert testResults < 1, testMessage

# Run the test
def unitThrusters(show_plots, RAMP, ThrustNumber , Duration , Angle, Location, Rate, Cutoff, RampDown):
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
    sparetime = 2.5*1./macros.NANO2SEC
    thrStartTime=sparetime
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

    if RAMP == "OFF":
        # Run the simulation
        executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
        if ThrustNumber==1:
            ThrustMessage.OnTimeRequest = thrDurationTime*macros.NANO2SEC
        if ThrustNumber==2:
            sim_model.doubleArray_setitem(ThrustMessage, 0, thrDurationTime * macros.NANO2SEC)
            sim_model.doubleArray_setitem(ThrustMessage, 1, thrDurationTime * macros.NANO2SEC)
        TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 0, ThrustMessage)
        executeSimRun(TotalSim, thrusterSet, testRate, int(thrDurationTime+sparetime))

        # Gather the Force and Torque results
        thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
        thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')

        # Auto Generate LaTex Figures
        format = "width=0.5\\textwidth"

        PlotName = "Force_" +  str(ThrustNumber) + "Thrusters_" +  str(Duration)+ "s_" + str(Angle)+"deg_"+ "Loc"+str(Location[0])
        PlotTitle = "Force on Y with " + str(ThrustNumber) + " thrusters, for "  +  str(Duration)+ " sec at " + str(Angle)+" deg "

        plt.figure(1)
        plt.plot(thrForce[:,0]*macros.NANO2SEC, thrForce[:,2])
        plt.xlabel('Time(s)')
        plt.ylabel('Thrust Factor (-)')
        plt.ylim(-0.2,1)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
        plt.close()

        PlotName = "Torque_" +  str(ThrustNumber) + "Thrusters_" +  str(Duration)+ "s_" + str(Angle)+"deg_"+ "Loc"+str(Location[0])
        PlotTitle = "Torque on X with " + str(ThrustNumber) + " thrusters, for "  +  str(Duration)+ " sec at " + str(Angle)+" deg "

        plt.figure(11)
        plt.plot(thrForce[:,0]*macros.NANO2SEC, thrTorque[:,1])
        plt.xlabel('Time(s)')
        plt.ylabel('Thrust Torque (-)')
        plt.ylim(-1.5, 2)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
        plt.close()

        PlotName =  str(ThrustNumber) + "Thrusters_" +  str(Duration)+ "s_" + str(Angle)+"deg_"+ "Loc"+str(Location[0])
        PlotTitle = "All Forces and Torques " + str(ThrustNumber) + " thrusters, for "  +  str(Duration)+ " sec at " + str(Angle)+" deg "

        plt.figure(22)
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,1], 'b', label='x Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,1], 'b--', label='x Torque')
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,2], 'g', label='y Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,2], 'g--', label='y Torque')
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,3], 'r', label = 'z Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,3], 'r--', label='z Torque')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylim(-1.5, 2)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
        plt.close()

        #Create expected Force to test against thrForce
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

        #Create expected Torque to test against thrTorque
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


        # Define errorTolerance
        TruthTorque = np.transpose(expectedpointstor)
        ErrTolerance = 10E-9

        # Compare Torque values
        # Compare Force values
        testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance, "Torque", testFailCount, testMessages)

    if RAMP == "ON":
        format = "width=0.5\\textwidth"
        rampsteps = 10
        sparetime = 2.*1/macros.NANO2SEC
        thrStartTime = sparetime - 1.*1/macros.NANO2SEC

        # Setup thruster ramp on and ramp off configuration
        rampOnList = []
        rampOffList = []
        # Note that this ramp is totally linear and ramps up 30 ms using 30 steps
        for i in range(rampsteps):
            newElement = thrusterDynamicEffector.ThrusterTimePair()
            newElement.TimeDelta = (i + 1.) * 0.1
            newElement.ThrustFactor = (i + 1.0) / 10.0
            newElement.IspFactor = (i + 1.0) / 10.0
            rampOnList.append(newElement)
            newElement = thrusterDynamicEffector.ThrusterTimePair()
            newElement.TimeDelta = (i + 1) * 0.1
            newElement.ThrustFactor = 1.0 - (i + 1.0) / 10.0
            newElement.IspFactor = newElement.ThrustFactor
            rampOffList.append(newElement)

        # Set up the ramps
        thrusterSet.ThrusterData[0].ThrusterOnRamp = \
            thrusterDynamicEffector.ThrusterTimeVector(rampOnList)
        thrusterSet.ThrusterData[0].ThrusterOffRamp = \
            thrusterDynamicEffector.ThrusterTimeVector(rampOffList)

        if RampDown == "OFF":
            if Cutoff == "OFF":

                ##Execute a new firing that will use the thruster ramps
                executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
                ThrustMessage.OnTimeRequest =  thrDurationTime*macros.NANO2SEC;
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 0, ThrustMessage)
                executeSimRun(TotalSim, thrusterSet, testRate, int(thrDurationTime+sparetime))

                #Extract log variables and plot the results
                thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
                thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')


                PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + Cutoff
                PlotTitle = "All Forces and Torques, Ramp" + "steps_Cutoff" + Cutoff

                plt.figure(22)
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
                plt.legend()
                plt.xlabel('Time(s)')
                plt.ylim(-1.5, 2)
                unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
                if show_plots == True:
                    plt.show()
                plt.close()

                # Create expected Force to test against thrForce
                expectedpoints = np.zeros([3, np.shape(thrForce)[0]])
                RampFunction= np.zeros([np.shape(thrForce)[0]])
                ramplength = 1.
                if ramplength < thrDurationTime*macros.NANO2SEC:
                    for i in range(np.shape(thrForce)[0]):
                        if i<int(round(thrStartTime / testRate)) + 2:
                            RampFunction[i] = 0.0
                        if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC)/ testRate)) + 2) : #ramp up
                            RampFunction[i] = (i-int(round(thrStartTime / testRate)) - 2 + 1.0) * (macros.NANO2SEC*testRate)
                        if (i > int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC)/ testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime) / testRate)) + 2):
                            RampFunction[i]=1.0
                        if (i > int(round((thrStartTime + thrDurationTime) / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                            RampFunction[i] = 1.0 - (i - int(round((thrStartTime + thrDurationTime) / testRate))-2 + 1.0) *(macros.NANO2SEC*testRate)
                        if (i > int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 1):
                            RampFunction[i] = 0.
                else:
                    for i in range(np.shape(thrForce)[0]):
                        if i<int(round(thrStartTime / testRate)) + 2:
                            RampFunction[i] = 0.0
                        if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime)/ testRate)) + 2) : #ramp up
                            RampFunction[i] = (i-int(round(thrStartTime / testRate)) - 2 + 1.0) * (macros.NANO2SEC*testRate)
                        if (i > int(round((thrStartTime + thrDurationTime) / testRate)) + 1 and i < int(round((thrStartTime + 2*thrDurationTime) / testRate)) + 2):
                            RampFunction[i] = RampFunction[int(round((thrStartTime + thrDurationTime) / testRate)) + 1] - (i - int(round((thrStartTime + thrDurationTime) / testRate))-2 + 1.0) *(macros.NANO2SEC*testRate)
                        if (i > int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 1):
                            RampFunction[i] = 0.


                for i in range(np.shape(thrForce)[0]):
                    if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                        expectedpoints[0, i] = 0.0
                        expectedpoints[1, i] = 0.0
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                        expectedpoints[0, i] = math.cos(anglerad)*RampFunction[i]
                        expectedpoints[1, i] = math.sin(anglerad)*RampFunction[i]
                    else:
                        expectedpoints[0, i] = 0.0
                        expectedpoints[1, i] = 0.0

                # Modify expected values for comparison and define errorTolerance
                TruthForce = np.transpose(expectedpoints)
                ErrTolerance = 10E-9

                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                           testFailCount, testMessages)

                # Create expected Torque to test against thrTorque
                expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
                for i in range(np.shape(thrForce)[0]):
                    if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                        expectedpointstor[0, i] = 0.0
                        expectedpointstor[1, i] = 0.0
                        expectedpointstor[2, i] = 0.0
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                            expectedpointstor[0, i] = -math.sin(anglerad) * thruster1.inputThrLoc_S[2][0]*RampFunction[i]  # Torque about x is arm along z by the force projected upon y
                            expectedpointstor[1, i] = math.cos(anglerad) * math.sqrt(
                                thruster1.inputThrLoc_S[2][0] ** 2 + thruster1.inputThrLoc_S[1][0] ** 2 +
                                thruster1.inputThrLoc_S[0][0] ** 2) * math.sin(math.atan(
                                thruster1.inputThrLoc_S[2][0] / thruster1.inputThrLoc_S[0][0]))*RampFunction[i]  # Torque about x is arm along z by the force projected upon x
                            expectedpointstor[2, i] = math.sin(anglerad) * thruster1.inputThrLoc_S[0][0]*RampFunction[i]  # Torque about z is arm along x by the force projected upon y
                    else:
                        expectedpointstor[0, i] = 0.0
                        expectedpointstor[1, i] = 0.0
                        expectedpointstor[2, i] = 0.0

                # Define errorTolerance
                TruthTorque = np.transpose(expectedpointstor)
                ErrTolerance = 10E-9

                # Compare Torque values
                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                           "Torque", testFailCount, testMessages)
            if Cutoff == "ON":
                COtime = 0.2
                COrestart = 0.3

                executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
                ThrustMessage.OnTimeRequest = COtime * 10.
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 0, ThrustMessage)
                executeSimRun(TotalSim, thrusterSet, testRate, int(COtime * 1.0 / macros.NANO2SEC))
                ThrustMessage.OnTimeRequest = COrestart
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 1,
                                                   ThrustMessage)  # Need to change 0 to 1 to chain the message
                executeSimRun(TotalSim, thrusterSet, testRate, int(COrestart * 1.0 / macros.NANO2SEC + sparetime))

                # Extract log variables and plot the results
                thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
                thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')

                PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + Cutoff
                PlotTitle = "All Forces and Torques, Ramp" + "steps_Cutoff" + Cutoff

                plt.figure(55)
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
                plt.legend()
                plt.xlabel('Time(s)')
                plt.ylim(-1.5, 2)
                unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
                if show_plots == True:
                    plt.show()
                plt.close()

                # Create expected Force to test against thrForce
                expectedpoints = np.zeros([3, np.shape(thrForce)[0]])
                RampFunction= np.zeros([np.shape(thrForce)[0]])
                ramplength = 0.5

                for i in range(np.shape(thrForce)[0]):
                    if i<int(round(thrStartTime / testRate)) + 2:
                        RampFunction[i] = 0.0
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC)/ testRate)) + 2) : #ramp up
                        RampFunction[i] = (i-int(round(thrStartTime / testRate)) - 2 + 1.0) * (macros.NANO2SEC*testRate)
                    if (i > int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC) / testRate)) + 1 and i < int(round((thrStartTime + 2*ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                        RampFunction[i] = RampFunction[int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC) / testRate)) + 1] - (i - int(round((thrStartTime + ramplength*1.0/macros.NANO2SEC) / testRate))-2 + 1.0) *(macros.NANO2SEC*testRate)
                    if (i > int(round((thrStartTime + 2*ramplength*1.0/macros.NANO2SEC) / testRate)) + 1):
                        RampFunction[i] = 0.

                for i in range(np.shape(thrForce)[0]):
                    if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                        expectedpoints[0, i] = 0.0
                        expectedpoints[1, i] = 0.0
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                        expectedpoints[0, i] = math.cos(anglerad)*RampFunction[i]
                        expectedpoints[1, i] = math.sin(anglerad)*RampFunction[i]
                    else:
                        expectedpoints[0, i] = 0.0
                        expectedpoints[1, i] = 0.0

                # Modify expected values for comparison and define errorTolerance
                TruthForce = np.transpose(expectedpoints)
                ErrTolerance = 10E-9

                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                           testFailCount, testMessages)

                # Create expected Torque to test against thrTorque
                expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
                for i in range(np.shape(thrForce)[0]):
                    if (i < int(round(thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                        expectedpointstor[0, i] = 0.0
                        expectedpointstor[1, i] = 0.0
                        expectedpointstor[2, i] = 0.0
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                            expectedpointstor[0, i] = -math.sin(anglerad) * thruster1.inputThrLoc_S[2][0]*RampFunction[i]  # Torque about x is arm along z by the force projected upon y
                            expectedpointstor[1, i] = math.cos(anglerad) * math.sqrt(
                                thruster1.inputThrLoc_S[2][0] ** 2 + thruster1.inputThrLoc_S[1][0] ** 2 +
                                thruster1.inputThrLoc_S[0][0] ** 2) * math.sin(math.atan(
                                thruster1.inputThrLoc_S[2][0] / thruster1.inputThrLoc_S[0][0]))*RampFunction[i]  # Torque about x is arm along z by the force projected upon x
                            expectedpointstor[2, i] = math.sin(anglerad) * thruster1.inputThrLoc_S[0][0]*RampFunction[i]  # Torque about z is arm along x by the force projected upon y
                    else:
                        expectedpointstor[0, i] = 0.0
                        expectedpointstor[1, i] = 0.0
                        expectedpointstor[2, i] = 0.0

                # Define errorTolerance
                TruthTorque = np.transpose(expectedpointstor)
                ErrTolerance = 10E-9

                # Compare Torque values
                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                           "Torque", testFailCount, testMessages)

        if RampDown == "ON":
            RDrestart = 0.2
            RDstart = 0.6
            RDlength = 1.5

            executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
            ThrustMessage.OnTimeRequest = RDstart
            TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 0, ThrustMessage)
            executeSimRun(TotalSim, thrusterSet, testRate, int((RDstart+ RDrestart) * 1.0 / macros.NANO2SEC))
            ThrustMessage.OnTimeRequest = RDlength
            TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, 1,
                                               ThrustMessage)  # Need to change 0 to 1 to chain the message
            executeSimRun(TotalSim, thrusterSet, testRate, int(RDlength * 1.0 / macros.NANO2SEC + sparetime))

            # Extract log variables and plot the results
            thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
            thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')

            PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + Cutoff + "RampDown" + RampDown
            PlotTitle = "All Forces and Torques, Ramp" + "steps_Cutoff" + Cutoff + "RampDown" + RampDown

            plt.figure(55)
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
            plt.legend()
            plt.xlabel('Time(s)')
            plt.ylim(-1.5, 2)
            unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
            if show_plots == True:
                plt.show()
            plt.close()

            # Create expected Force to test against thrForce
            expectedpoints = np.zeros([3, np.shape(thrForce)[0]])
            RampFunction = np.zeros([np.shape(thrForce)[0]])
            ramplength = 1.
            for i in range(np.shape(thrForce)[0]):
                if i < int(round(thrStartTime / testRate)) + 2:
                    RampFunction[i] = 0.0
                if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + RDstart * 1.0 / macros.NANO2SEC) / testRate)) + 2):  # ramp up
                    RampFunction[i] = (i - int(round(thrStartTime / testRate)) - 2 + 1.0) * (macros.NANO2SEC * testRate)
                if (i > int(round((thrStartTime + RDstart * 1.0 / macros.NANO2SEC) / testRate)) + 1 and i < int(round((thrStartTime + (RDstart+RDrestart) * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    RampFunction[i] = RampFunction[int(round((thrStartTime + RDstart * 1.0 / macros.NANO2SEC) / testRate)) + 1] - (i - int(round((thrStartTime + (RDstart) * 1.0 / macros.NANO2SEC) / testRate)) - 2 + 1.0) * (macros.NANO2SEC * testRate)
                if (i > int(round((thrStartTime + (RDstart+RDrestart) * 1.0 / macros.NANO2SEC) / testRate)) + 1 and i < int(round((thrStartTime + (RDstart+RDrestart+(1. -RDstart+RDrestart)) * 1.0 / macros.NANO2SEC) / testRate)) + 2):  # ramp up
                    RampFunction[i] = RampFunction[int(round((thrStartTime + (RDstart+RDrestart) * 1.0 / macros.NANO2SEC) / testRate)) + 1]+ (i - int(round((thrStartTime+ (RDstart+RDrestart)* 1.0 / macros.NANO2SEC) / testRate)) - 2 + 1.0) * (macros.NANO2SEC * testRate)
                if (i > int(round((thrStartTime +  (RDstart+RDrestart+(1. -RDstart+RDrestart)) * 1.0 / macros.NANO2SEC) / testRate)) + 1 and i < int(round((thrStartTime + (RDstart+RDrestart + RDlength) * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    RampFunction[i] = 1.0
                if (i > int(round((thrStartTime + (RDstart+RDrestart + RDlength) * 1.0 / macros.NANO2SEC) / testRate)) + 1 and i < int(round((thrStartTime + (RDstart+RDrestart + RDlength+1) * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    RampFunction[i] = 1.0 - (i - int(round((thrStartTime + (RDstart+RDrestart + RDlength) * 1.0 / macros.NANO2SEC) / testRate)) - 2 + 1.0) * (macros.NANO2SEC * testRate)
                if (i > int(round((thrStartTime + (RDstart+RDrestart + RDlength+ ramplength) * 1.0 / macros.NANO2SEC) / testRate)) + 1):
                    RampFunction[i] = 0.

            for i in range(np.shape(thrForce)[0]):
                if (i < int(round(
                            thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                    expectedpoints[0, i] = 0.0
                    expectedpoints[1, i] = 0.0
                if (i > int(round(thrStartTime / testRate)) + 1 and i < int(
                        round((thrStartTime + thrDurationTime + ramplength * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    expectedpoints[0, i] = math.cos(anglerad) * RampFunction[i]
                    expectedpoints[1, i] = math.sin(anglerad) * RampFunction[i]
                else:
                    expectedpoints[0, i] = 0.0
                    expectedpoints[1, i] = 0.0

            # Modify expected values for comparison and define errorTolerance
            TruthForce = np.transpose(expectedpoints)
            ErrTolerance = 10E-9

            # Compare Force values
            testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                       testFailCount, testMessages)

            # Create expected Torque to test against thrTorque
            expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
            for i in range(np.shape(thrForce)[0]):
                if (i < int(round(
                            thrStartTime / testRate)) + 2):  # Thrust fires 2 times steps after the pause of sim and restart
                    expectedpointstor[0, i] = 0.0
                    expectedpointstor[1, i] = 0.0
                    expectedpointstor[2, i] = 0.0
                if (i > int(round(thrStartTime / testRate)) + 1 and i < int(
                        round((thrStartTime + thrDurationTime + ramplength * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    expectedpointstor[0, i] = -math.sin(anglerad) * thruster1.inputThrLoc_S[2][0] * RampFunction[
                        i]  # Torque about x is arm along z by the force projected upon y
                    expectedpointstor[1, i] = math.cos(anglerad) * math.sqrt(
                        thruster1.inputThrLoc_S[2][0] ** 2 + thruster1.inputThrLoc_S[1][0] ** 2 +
                        thruster1.inputThrLoc_S[0][0] ** 2) * math.sin(math.atan(
                        thruster1.inputThrLoc_S[2][0] / thruster1.inputThrLoc_S[0][0])) * RampFunction[
                                                  i]  # Torque about x is arm along z by the force projected upon x
                    expectedpointstor[2, i] = math.sin(anglerad) * thruster1.inputThrLoc_S[0][0] * RampFunction[
                        i]  # Torque about z is arm along x by the force projected upon y
                else:
                    expectedpointstor[0, i] = 0.0
                    expectedpointstor[1, i] = 0.0
                    expectedpointstor[2, i] = 0.0

            # Define errorTolerance
            TruthTorque = np.transpose(expectedpointstor)
            ErrTolerance = 10E-9

            # Compare Torque values
            # Compare Force values
            testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                       "Torque", testFailCount, testMessages)

    if testFailCount == 0:
        print "PASSED: " + " No ramp force and torque"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitThrusters(False, "ON", 1 , 5. , 30., [[1.125], [0.0], [2.0]], 10E6, "ON", "ON")

