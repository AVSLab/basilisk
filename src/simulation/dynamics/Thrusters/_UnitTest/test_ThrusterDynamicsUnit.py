''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import os
import numpy as np
import math
import pytest
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

# @endcond

#Import all of the modules that we are going to call in this simulation
from Basilisk.utilities import unitTestSupport
import matplotlib.pyplot as plt
from Basilisk.utilities import MessagingAccess
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.simulation import stateArchitecture
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import macros


class ResultsStore:
    def __init__(self):
        self.PassFail = []
    def texSnippet(self):
        for i in range(len(self.PassFail)):
            snippetName = 'Result' + str(i)
            if self.PassFail[i] == 'PASSED':
                textColor = 'ForestGreen'
            elif self.PassFail[i] == 'FAILED':
                textColor = 'Red'
            texSnippet =  '\\textcolor{' + textColor + '}{'+ self.PassFail[i] + '}'
            unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

@pytest.fixture(scope="module")
def testFixture():
    listRes = ResultsStore()
    yield listRes
    listRes.texSnippet()

def thrusterEffectorAllTests(show_plots):
   [testResults, testMessage] = test_unitThrusters(show_plots)


# Create function to run the simulation who's results will be compared to expected values
def executeSimRun(simContainer, thrusterSet, simRate, totalTime):
    newStopTime = simContainer.TotalSim.CurrentNanos + totalTime
    while(simContainer.TotalSim.CurrentNanos < newStopTime):
        simContainer.ConfigureStopTime(simContainer.TotalSim.CurrentNanos + simRate)
        simContainer.ExecuteSimulation()

        thrusterSet.computeForceTorque(simContainer.TotalSim.CurrentNanos*macros.NANO2SEC)
        thrusterSet.computeForceTorque(simContainer.TotalSim.CurrentNanos*macros.NANO2SEC + simRate*macros.NANO2SEC/2.0)
        thrusterSet.computeForceTorque(simContainer.TotalSim.CurrentNanos * macros.NANO2SEC + simRate * macros.NANO2SEC / 2.0)
        thrusterSet.computeForceTorque(simContainer.TotalSim.CurrentNanos*macros.NANO2SEC + simRate*macros.NANO2SEC)

        thrusterSet.computeStateContribution(simContainer.TotalSim.CurrentNanos * macros.NANO2SEC)
        thrusterSet.computeStateContribution(
            simContainer.TotalSim.CurrentNanos * macros.NANO2SEC + simRate * macros.NANO2SEC / 2.0)
        thrusterSet.computeStateContribution(
            simContainer.TotalSim.CurrentNanos * macros.NANO2SEC + simRate * macros.NANO2SEC / 2.0)
        thrusterSet.computeStateContribution(
            simContainer.TotalSim.CurrentNanos * macros.NANO2SEC + simRate * macros.NANO2SEC)


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("ramp, thrustNumber , duration , long_angle, lat_angle, location, rate, cutoff, rampDown", [
    ("OFF", 1 , 5.0 , 30. , 15., [[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF"), #Test random thrust config
    ("OFF", 1 , 0.1, 30.,  15.,[[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF"), # Short fire test
    ("OFF", 1, 0.1, 30.,  15.,[[1.125], [0.5], [2.0]], 1E6, "OFF", "OFF"), # Short fire test with higher test rate
    ("OFF", 1, 5.0, 30.,  15.,[[1.125], [0.5], [2.0]], 1E7, "OFF", "OFF"),# rate test
    ("OFF", 1 , 5.0 , 10. ,  35.,[[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF"), # angle test
    ("OFF", 1 , 5.0 , 30. ,  15.,[[1.], [1.5], [0.0]], 1E8, "OFF", "OFF"),# Position test
    ("OFF", 2 , 5.0 , 30. ,  15.,[[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF"), # Number of thrusters test
    ("ON", 1 , 5.0 , 30. ,  15.,[[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF") , # Basic ramp test
    ("ON", 1 , 0.5 , 30. ,  15.,[[1.125], [0.5], [2.0]], 1E8, "OFF", "OFF") , # Short ramp test
    ("ON", 1 , 5.0 , 30. ,  15.,[[1.125], [0.5], [2.0]], 1E7, "OFF", "OFF"), # rate ramp test
    ("ON", 1 , 5.0 , 30. ,  15.,[[1.125], [0.5], [2.0]], 1E8, "ON", "OFF"), # Cuttoff test
    ("ON", 1, 5.0, 30.,  15.,[[1.125], [0.5], [2.0]], 1E8, "ON", "ON")  # Rampdown test
    ])


# provide a unique test method name, starting with test_
def test_unitThrusters(testFixture, show_plots, ramp, thrustNumber , duration ,  long_angle, lat_angle,  location, rate, cutoff, rampDown):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(testFixture, show_plots, ramp, thrustNumber , duration  ,  long_angle, lat_angle , location, rate, cutoff, rampDown)
    assert testResults < 1, testMessage


# Run the test
def unitThrusters(testFixture, show_plots, ramp, thrustNumber , duration  ,  long_angle, lat_angle, location, rate, cutoff, rampDown):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Constants
    g = 9.80665
    Isp = 226.7

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
    angledeg_long = long_angle # Parametrized angle of thrust
    angledeg_lat = lat_angle
    anglerad_long = angledeg_long * math.pi/180.0
    anglerad_lat = angledeg_lat * math.pi / 180.0
    thruster1 = thrusterDynamicEffector.THRConfigSimMsg()
    thruster1.thrLoc_B =location # Parametrized location for thruster
    thruster1.thrDir_B = [[math.cos(anglerad_long)*math.cos(anglerad_lat)], [math.sin(anglerad_long)*math.cos(anglerad_lat)], [math.sin(anglerad_lat)]]
    thruster1.MaxThrust = 1.0
    thruster1.steadyIsp = 226.7
    thruster1.MinOnTime = 0.006
    thrusterSet.addThruster(thruster1)

    loc1 = np.array([thruster1.thrLoc_B[0][0],thruster1.thrLoc_B[1][0],thruster1.thrLoc_B[2][0]])
    dir1 = np.array([thruster1.thrDir_B[0][0], thruster1.thrDir_B[1][0], thruster1.thrDir_B[2][0]])

    if thrustNumber==2:
        thruster2 = thrusterDynamicEffector.THRConfigSimMsg()
        thruster2.thrLoc_B =np.array([[1.], [0.0], [0.0]]).reshape([3,1])
        thruster2.thrDir_B = np.array([[math.cos(anglerad_long+math.pi/4.)*math.cos(anglerad_lat-math.pi/4.)], [math.sin(anglerad_long+math.pi/4.)*math.cos(anglerad_lat-math.pi/4.)], [math.sin(anglerad_lat-math.pi/4.)]]).reshape([3,1])
        thruster2.MaxThrust = 1.0
        thruster2.steadyIsp = 226.7
        thruster2.MinOnTime = 0.006
        thrusterSet.addThruster(thruster2)

        loc2 = np.array([thruster2.thrLoc_B[0][0],thruster2.thrLoc_B[1][0],thruster2.thrLoc_B[2][0]])
        dir2 = np.array([thruster2.thrDir_B[0][0], thruster2.thrDir_B[1][0], thruster2.thrDir_B[2][0]])

    #  Create a Simulation
    testRate = int(rate) # Parametrized rate of test
    TotalSim = SimulationBaseClass.SimBaseClass()
    #Create a new process for the unit test task and add the module to tasking
    testProc = TotalSim.CreateNewProcess(unitProcessName)
    testProc.addTask(TotalSim.CreateNewTask(unitTaskName, testRate))
    TotalSim.AddModelToTask(unitTaskName, thrusterSet)
    TotalSim.scObject = spacecraftPlus.SpacecraftPlus()
    TotalSim.scObject.ModelTag = "spacecraftBody"

    #  Create a task manager
    TotalSim.newManager = stateArchitecture.DynParamManager()
    # TotalSim.AddModelToTask(unitTaskName, TotalSim.scObject)

    #  Define the start of the thrust and it's duration
    sparetime = 3.*1./macros.NANO2SEC
    thrStartTime=sparetime
    thrDurationTime=duration*1./macros.NANO2SEC # Parametrized thrust duration

    #Configure a single thruster firing, create a message for it
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.forceExternal_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.torqueExternalPntB_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.mDotTotal', testRate, 0, 0)

    ThrustMessage = thrusterDynamicEffector.THRArrayOnTimeCmdIntMsg()
    thrMessageSize = ThrustMessage.getStructSize()
    if thrustNumber==1:
        ThrustMessage.OnTimeRequest = [0.]
    if thrustNumber==2:
        ThrustMessage.OnTimeRequest = [0., 0.]
    TotalSim.TotalSim.CreateNewMessage(unitProcessName,"acs_thruster_cmds", thrMessageSize, 2)
    TotalSim.InitializeSimulation()

    #Configure the hub and link states
    TotalSim.scObject.hub.registerStates(TotalSim.newManager)
    thrusterSet.linkInStates(TotalSim.newManager)

    if ramp == "OFF":
        # Run the simulation
        executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
        if thrustNumber==1:
            ThrustMessage.OnTimeRequest = [thrDurationTime*macros.NANO2SEC]
        if thrustNumber==2:
            ThrustMessage.OnTimeRequest = [thrDurationTime * macros.NANO2SEC, thrDurationTime * macros.NANO2SEC]
        TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate, ThrustMessage)
        executeSimRun(TotalSim, thrusterSet, testRate, int(thrDurationTime+sparetime))

        # Gather the Force and Torque results
        thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
        thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')
        mDotData = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')

        # Auto Generate LaTex Figures
        format = "width=0.8\\textwidth"

        snippetName = "Snippet" + str(thrustNumber) + "Thrusters_" +  str(int(duration))+ "s_" +\
                      str(int(long_angle))+"deg_"+ "Loc"+ str(int(loc1[2])) + "_Rate"+str(int(1./(testRate*macros.NANO2SEC)))
        if thrustNumber==1:
            texSnippet = "The thruster is set at " +str(int(long_angle))+"$^\circ$ off the x-axis " +str(int(lat_angle))+"$^\circ$ off the z-axis, in the position $\\bm r = \left("+\
                         str(loc1[0])+","+str(loc1[1])+"," +str(loc1[2])+ \
                         "\\right)$. The test is launched using " + str(thrustNumber) + " thruster, for " + \
                         str(duration)+ " seconds. The test rate is " + str(int(1./(testRate*macros.NANO2SEC))) + " steps per second"
        if thrustNumber==2:
            texSnippet = "The first thruster is set at " + str(int(long_angle)) + "$^\circ$ off the x-axis " + str(
                int(lat_angle)) + "$^\circ$ off the z-axis, in the position $\\bm r = \left(" + \
                         str(loc1[0]) + "," + str(loc1[1]) + "," + str(loc1[2]) + \
                         "\\right)$. The second thruster is set at " + str(int(long_angle+45)) + "$^\circ$ off the x-axis " + str(
                int(lat_angle+45)) + "$^\circ$ off the z-axis, in the position $\\bm r = \left(" + \
                         str(loc2[0]) + "," + str(loc2[1]) + "," + str(loc2[2]) + \
                         "\\right)$. The test uses these " + str(thrustNumber) + " thrusters for " + \
                         str(duration) + " seconds. The test rate is " + str(
                int(1. / (testRate * macros.NANO2SEC))) + " steps per second"
        unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

        PlotName = "Force_" +  str(thrustNumber) + "Thrusters_" +  str(int(duration))+ "s_" +str(int(long_angle))+"deg_"+ "Loc"+str(int(location[2][0]))+ "_Rate"+str(int(1./(testRate*macros.NANO2SEC)))
        PlotTitle = "Force on Y with " + str(thrustNumber) + " thrusters, for "  +  str(int(duration))+ " sec at " +str(int(long_angle))+" deg "+ "Rate"+str(int(1./(testRate*macros.NANO2SEC)))

        plt.figure(1)
        plt.clf()
        plt.plot(thrForce[:,0]*macros.NANO2SEC, thrForce[:,2])
        plt.xlabel('Time(s)')
        plt.ylabel('Thrust Factor (N)')
        plt.ylim(-0.2,1)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
            plt.close('all')

        PlotName = "Torque_" +  str(thrustNumber) + "Thrusters_" +  str(int(duration))+ "s_" + str(int(long_angle))+"deg_"+ "Loc"+str(int(location[2][0]))+ "_Rate"+str(int(1./(testRate*macros.NANO2SEC)))
        PlotTitle = "Torque on X with " + str(thrustNumber) + " thrusters, for "  +  str(int(duration))+ " sec at " + str(int(long_angle))+" deg " + "Rate"+str(int(1./(testRate*macros.NANO2SEC)))

        plt.figure(11)
        plt.clf()
        plt.plot(thrForce[:,0]*macros.NANO2SEC, thrTorque[:,1])
        plt.xlabel('Time(s)')
        plt.ylabel('Thrust Torque (Nm)')
        plt.ylim(-1.5, 2)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
            plt.close('all')

        PlotName =  str(thrustNumber) + "Thrusters_" +  str(int(duration))+ "s_" + str(int(long_angle))+"deg_"+ "Loc"+str(int(location[2][0]))+ "_Rate"+str(int(1./(testRate*macros.NANO2SEC)))
        PlotTitle = "All Forces and Torques " + str(thrustNumber) + " thrusters, for "  +  str(int(duration))+ " sec at " + str(int(long_angle))+" deg "+ "Rate"+str(int(1./(testRate*macros.NANO2SEC)))

        plt.figure(22)
        plt.clf()
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,1], 'b', label='x Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,1], 'b--', label='x Torque')
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,2], 'g', label='y Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,2], 'g--', label='y Torque')
        plt.plot(thrForce[:,0]*1.0E-9, thrForce[:,3], 'r', label = 'z Force')
        plt.plot(thrTorque[:,0]*1.0E-9, thrTorque[:,3], 'r--', label='z Torque')
        plt.legend()
        plt.xlabel('Time(s)')
        plt.ylim(-1.5, 2)
        plt.legend(loc=1)
        unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
        if show_plots==True:
            plt.show()
            plt.close('all')

        #Create expected Force to test against thrForce
        expMDot = np.zeros([np.shape(np.array(mDotData))[0],1])
        for i in range(np.shape(np.array(mDotData))[0]):
            if (i > 0 and i < int(round((thrDurationTime) / testRate)) + 1):
                expMDot[i, 0] = thrustNumber / (g * Isp)


        expectedpoints=np.zeros([3,np.shape(thrForce)[0]])
        for i in range(np.shape(thrForce)[0]):# Thrust fires 2 times steps after the pause of sim and restart
            if (i>int(round(thrStartTime/ testRate)) + 1 and i<int(round((thrStartTime+thrDurationTime)/ testRate)) + 2):
                if thrustNumber == 1:
                    expectedpoints[0:3,i] = dir1
                else:
                    expectedpoints[0:3, i] = dir1 + dir2

        # Modify expected values for comparison and define errorTolerance
        TruthForce = np.transpose(expectedpoints)
        ErrTolerance = 10E-9

        # Compare Force values
        testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force", testFailCount, testMessages)
        for i in range(0, len(np.array(mDotData))):
            if not unitTestSupport.isArrayEqual(np.array(mDotData)[i,:], expMDot[i,:], 1, ErrTolerance):
                testFailCount+=1
                testMessages.append('M dot failure')

        #Create expected Torque to test against thrTorque
        expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
        for i in range(np.shape(thrForce)[0]): # Thrust fires 2 times steps after the pause of sim and restart
            if (i>int(round(thrStartTime/ testRate)) + 1 and i<int(round((thrStartTime+thrDurationTime)/ testRate)) + 2):
                if thrustNumber == 1:
                    expectedpointstor[0:3, i] = np.cross(loc1, dir1)
                else:
                    expectedpointstor[0:3, i] = np.cross(loc1, dir1) + np.cross(loc2, dir2)


        # Define errorTolerance
        TruthTorque = np.transpose(expectedpointstor)
        ErrTolerance = 10E-9

        # Compare Torque values
        # Compare Force values
        testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance, "Torque", testFailCount, testMessages)


    if ramp == "ON":
        format = "width=0.8\\textwidth"
        rampsteps = 10
        sparetime = 3.*1/macros.NANO2SEC
        thrStartTime = sparetime - 1.*1/macros.NANO2SEC

        # Setup thruster ramp on and ramp off configuration
        rampOnList = []
        rampOffList = []
        # Note that this ramp is totally linear and ramps up 30 ms using 30 steps
        for i in range(rampsteps):
            newElement = thrusterDynamicEffector.THRTimePairSimMsg()
            newElement.TimeDelta = (i + 1.) * 0.1
            newElement.ThrustFactor = (i + 1.0) / 10.0
            newElement.IspFactor = (i + 1.0) / 10.0
            rampOnList.append(newElement)
            newElement = thrusterDynamicEffector.THRTimePairSimMsg()
            newElement.TimeDelta = (i + 1) * 0.1
            newElement.ThrustFactor = 1.0 - (i + 1.0) / 10.0
            newElement.IspFactor = newElement.ThrustFactor
            rampOffList.append(newElement)

        # Set up the ramps
        thrusterSet.thrusterData[0].ThrusterOnRamp = \
            thrusterDynamicEffector.ThrusterTimeVector(rampOnList)
        thrusterSet.thrusterData[0].ThrusterOffRamp = \
            thrusterDynamicEffector.ThrusterTimeVector(rampOffList)

        if rampDown == "OFF":
            if cutoff == "OFF":

                ##Execute a new firing that will use the thruster ramps
                executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
                ThrustMessage.OnTimeRequest =  [thrDurationTime*macros.NANO2SEC]
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate, ThrustMessage)
                executeSimRun(TotalSim, thrusterSet, testRate, int(thrDurationTime+sparetime))

                #Extract log variables and plot the results
                thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
                thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')
                mDotData = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')

                snippetName = "Snippet" + "Ramp_" + str(rampsteps) +"steps_" + str(int(duration)) + "s"+  "_Cutoff" + cutoff + "_Rate" + str(
                    int(1. / (testRate * macros.NANO2SEC))) + "_Cutoff" + cutoff
                texSnippet = "We test the ramped thrust with " + str(rampsteps) + " incremental steps. The single thruster is set at the default " +str(int(long_angle))+"$^\circ$ off the x-axis " +str(int(lat_angle))+"$^\circ$ off the z-axis, at $\\bm r = \left(" + \
                             str(loc1[0]) + "," + str(loc1[1]) + "," + str(loc1[2]) + \
                             "\\right)$. The thrust is set for " + \
                             str(duration) + " seconds with a test rate of " + str(
                    int(1. / (testRate * macros.NANO2SEC))) + " steps per second. The Cutoff test is " + cutoff
                unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

                PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + cutoff +"_" + str(int(duration)) + "s"+"_testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))
                PlotTitle = "All Forces and Torques with " + str(rampsteps) + " step Ramp, thrust for " + str(int(duration)) + "s. Cutoff " + cutoff+", testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))

                plt.figure(22)
                plt.clf()
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
                plt.legend()
                plt.xlabel('Time(s)')
                plt.ylim(-1.5, 2)
                plt.legend(loc=2)
                unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
                if show_plots == True:
                    plt.show()
                    plt.close('all')

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


                # Create expected Force to test against thrForce
                expMDot = np.zeros([np.shape(np.array(mDotData))[0], 1])
                for i in range(np.shape(RampFunction)[0]- (int(round(thrStartTime/testRate))+1)):
                    if (i>0 and RampFunction[i + int(round(thrStartTime/testRate))+1]!=0.):
                        expMDot[i, 0] = thrustNumber / (g * Isp)

                for i in range(np.shape(thrForce)[0]): # Thrust fires 2 times steps after the pause of sim and restart
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                        expectedpoints[0:3, i] = dir1*RampFunction[i]

                # Modify expected values for comparison and define errorTolerance
                TruthForce = np.transpose(expectedpoints)
                ErrTolerance = 10E-9

                # Compare Force values and M-dot
                testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                           testFailCount, testMessages)
                for i in range(0, len(np.array(mDotData))):
                    if not unitTestSupport.isArrayEqual(np.array(mDotData)[i, :], expMDot[i, :], 1, ErrTolerance):
                        testFailCount += 1
                        testMessages.append('M dot failure')

                # Create expected Torque to test against thrTorque
                expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
                for i in range(np.shape(thrForce)[0]):  # Thrust fires 2 times steps after the pause of sim and restart
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                            expectedpointstor[0:3, i] = np.cross(loc1,dir1)*RampFunction[i]

                # Define errorTolerance
                TruthTorque = np.transpose(expectedpointstor)
                ErrTolerance = 10E-9

                # Compare Torque values
                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                           "Torque", testFailCount, testMessages)
            if cutoff == "ON":
                COtime = 0.2
                COrestart = 0.3

                executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
                ThrustMessage.OnTimeRequest = [COtime * 10.]
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate, ThrustMessage)
                executeSimRun(TotalSim, thrusterSet, testRate, int(COtime * 1.0 / macros.NANO2SEC))
                ThrustMessage.OnTimeRequest = [COrestart]
                TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate,
                                                   ThrustMessage)  # Need to change 0 to 1 to chain the message
                executeSimRun(TotalSim, thrusterSet, testRate, int(COrestart * 1.0 / macros.NANO2SEC + sparetime))

                # Extract log variables and plot the results
                thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
                thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')
                mDotData = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')

                PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + cutoff +"_" + str(int(duration)) + "s"+"_testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))
                PlotTitle = "All Forces and Torques, with a "  + str(rampsteps) + " step Ramp, thrust for " + str(int(duration)) + "s. Cutoff " + cutoff+", testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))

                snippetName = "Snippet" + "Ramp_" + str(rampsteps) + "steps_Cutoff" + cutoff + "_Rate" + str(
                    int(1. / (testRate * macros.NANO2SEC)))  + "_Cutoff" + cutoff
                texSnippet = "We test the ramped thrust with " + str(rampsteps) + " incremental steps. The single thruster is set at the default " +str(int(long_angle))+"$^\circ$ off the x-axis " +str(int(lat_angle))+"$^\circ$ off the z-axis, at $\\bm r = \left(" + \
                             str(loc1[0]) + "," + str(loc1[1]) + "," + str(loc1[2]) + \
                             "\\right)$. The thrust is set for " + \
                             str(duration) + " seconds with a test rate of " + str(
                    int(1. / (testRate * macros.NANO2SEC))) + " steps per second. The Cutoff test is " + cutoff
                unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

                plt.figure(55)
                plt.clf()
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
                plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
                plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
                plt.legend()
                plt.xlabel('Time(s)')
                plt.ylim(-1.5, 2)
                plt.legend(loc=2)
                unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
                if show_plots == True:
                    plt.show()
                    plt.close('all')

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

                # Create expected Force to test against thrForce
                expMDot = np.zeros([np.shape(np.array(mDotData))[0], 1])
                for i in range(np.shape(RampFunction)[0]- (int(round(thrStartTime/testRate))+1)):
                    if (i>0 and RampFunction[i + int(round(thrStartTime/testRate))+1]!=0.):
                        expMDot[i, 0] = thrustNumber / (g * Isp)

                for i in range(np.shape(thrForce)[0]):# Thrust fires 2 times steps after the pause of sim and restart
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                        expectedpoints[0:3, i] = dir1*RampFunction[i]

                # Modify expected values for comparison and define errorTolerance
                TruthForce = np.transpose(expectedpoints)
                ErrTolerance = 10E-9

                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                           testFailCount, testMessages)
                for i in range(0, len(np.array(mDotData))):
                    if not unitTestSupport.isArrayEqual(np.array(mDotData)[i, :], expMDot[i, :], 1, ErrTolerance):
                        testFailCount += 1
                        testMessages.append('M dot failure')

                # Create expected Torque to test against thrTorque
                expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
                for i in range(np.shape(thrForce)[0]): # Thrust fires 2 times steps after the pause of sim and restart
                    if (i > int(round(thrStartTime / testRate)) + 1 and i < int(round((thrStartTime + thrDurationTime+ ramplength*1.0/macros.NANO2SEC) / testRate)) + 2):
                            expectedpointstor[0:3, i] = np.cross(loc1,dir1)*RampFunction[i]

                # Define errorTolerance
                TruthTorque = np.transpose(expectedpointstor)
                ErrTolerance = 10E-9

                # Compare Torque values
                # Compare Force values
                testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                           "Torque", testFailCount, testMessages)

        if rampDown == "ON":
            RDrestart = 0.2
            RDstart = 0.5
            RDlength = 1.5

            executeSimRun(TotalSim, thrusterSet, testRate, int(thrStartTime))
            ThrustMessage.OnTimeRequest = [RDstart]
            TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate, ThrustMessage)
            executeSimRun(TotalSim, thrusterSet, testRate, int((RDstart+ RDrestart) * 1.0 / macros.NANO2SEC))
            ThrustMessage.OnTimeRequest = [RDlength]
            TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", thrMessageSize, TotalSim.TotalSim.CurrentNanos+testRate,
                                               ThrustMessage)  # Need to change 0 to 1 to chain the message
            executeSimRun(TotalSim, thrusterSet, testRate, int(RDlength * 1.0 / macros.NANO2SEC + sparetime))

            # Extract log variables and plot the results
            thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
            thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')
            mDotData = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')

            PlotName = "Ramp_" + str(rampsteps) + "steps_Cutoff" + cutoff + "rampDown" + rampDown+"_testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))
            PlotTitle = "All Forces and Torques, with a " + str(rampsteps) + " step Ramp, Cutoff " + cutoff + ", RampDown" + rampDown +" testRate" + str(
                int(1. / (testRate * macros.NANO2SEC)))

            snippetName = "Snippet" + "Ramp_" + str(rampsteps) + "steps_Cutoff" + cutoff + "_Rate" + str(
                int(1. / (testRate * macros.NANO2SEC)))+ "rampDown" + rampDown
            texSnippet = "We test the ramped thrust with " + str(
                rampsteps) + " incremental steps. The single thruster is set at the default "+str(int(long_angle))+"$^\circ$ off the x-axis " +str(int(lat_angle))+"$^\circ$ off the z-axis, at $\\bm r = \left(" + \
                         str(loc1[0]) + "," + str(loc1[1]) + "," + str(loc1[2]) + \
                         "\\right)$. The thrust is set for " + \
                         str(RDstart) + " seconds initially with a test rate of " + str(
                int(1. / (testRate * macros.NANO2SEC))) + " steps per second. The Cutoff test is " + cutoff + \
                        " the RampDown test is " + rampDown + "."
            unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)

            plt.figure(55)
            plt.clf()
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 1], 'b', label='x Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 1], 'b--', label='x Torque')
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 2], 'g', label='y Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 2], 'g--', label='y Torque')
            plt.plot(thrForce[:, 0] * 1.0E-9, thrForce[:, 3], 'r', label='z Force')
            plt.plot(thrTorque[:, 0] * 1.0E-9, thrTorque[:, 3], 'r--', label='z Torque')
            plt.legend()
            plt.xlabel('Time(s)')
            plt.ylim(-1.5, 2)
            plt.legend(loc=2)
            unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
            if show_plots == True:
                plt.show()
                plt.close('all')

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

            # Create expected Force to test against thrForce
            expMDot = np.zeros([np.shape(np.array(mDotData))[0], 1])
            for i in range(1,np.shape(RampFunction)[0] - (int(round(thrStartTime / testRate))+1)):
                if (RampFunction[i + int(round(thrStartTime / testRate))+1] != 0.):
                    expMDot[i, 0] = thrustNumber / (g * Isp)
                    expMDot[i+1, 0] = thrustNumber / (g * Isp) # The way the last ramp is set up, we need another mdot value

            PlotName = "Ramp_function"
            PlotTitle = "Example of ramp function"

            plt.figure(11)
            plt.clf()
            plt.plot(thrForce[:, 0] * macros.NANO2SEC, RampFunction)
            plt.xlabel('Time(s)')
            plt.ylabel('Ramp(-)')
            plt.ylim(-1.5, 2)
            unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
            if show_plots == True:
                plt.show()
                plt.close('all')

            for i in range(np.shape(thrForce)[0]): # Thrust fires 2 times steps after the pause of sim and restart
                if (i > int(round(thrStartTime / testRate)) + 1 and i < int(
                        round((thrStartTime + thrDurationTime + ramplength * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    expectedpoints[0:3, i] = dir1*RampFunction[i]


            # Modify expected values for comparison and define errorTolerance
            TruthForce = np.transpose(expectedpoints)
            ErrTolerance = 10E-9

            # Compare Force values
            testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                                       testFailCount, testMessages)
            for i in range(0, len(np.array(mDotData))):
                if not unitTestSupport.isArrayEqual(np.array(mDotData)[i, :], expMDot[i, :], 1, ErrTolerance):
                    testFailCount += 1
                    testMessages.append('M dot failure')

            # Create expected Torque to test against thrTorque
            expectedpointstor = np.zeros([3, np.shape(thrTorque)[0]])
            for i in range(np.shape(thrForce)[0]): # Thrust fires 2 times steps after the pause of sim and restart
                if (i > int(round(thrStartTime / testRate)) + 1 and i < int(
                        round((thrStartTime + thrDurationTime + ramplength * 1.0 / macros.NANO2SEC) / testRate)) + 2):
                    expectedpointstor[0:3, i] = np.cross(loc1, dir1)*RampFunction[i]


            # Define errorTolerance
            TruthTorque = np.transpose(expectedpointstor)
            ErrTolerance = 10E-9

            # Compare Torque values
            # Compare Force values
            testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance,
                                                                       "Torque", testFailCount, testMessages)



    if testFailCount == 0:
        print "PASSED"
        testFixture.PassFail.append("PASSED")
    else:
        testFixture.PassFail.append("FAILED")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    unitThrusters(ResultsStore(), False, "ON", 1, 5.0, 30.,  15.,[[1.125], [0.5], [2.0]], 1E8, "ON", "ON")

