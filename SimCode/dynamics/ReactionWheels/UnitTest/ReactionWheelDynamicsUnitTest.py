# this is the initial version of the thruster unit test
#Very simple simulation.

from ctypes import*

import sys, os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

#Import all of the modules that we are going to call in this simulation
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import reactionwheel_dynamics

def executeSimulationRun(stopTime, stepTime, TotalSim, RWDynObject, 
    massPropsData, outputState):
    newStop = TotalSim.TotalSim.CurrentNanos
    while(TotalSim.TotalSim.CurrentNanos < stopTime):
        RWDynObject.ComputeDynamics(massPropsData, outputState,
            newStop*1.0E-9 - stepTime*1.0E-9)
        RWDynObject.ComputeDynamics(massPropsData, outputState,
            newStop*1.0E-9 - stepTime/2*1.0E-9)
        RWDynObject.ComputeDynamics(massPropsData, outputState,
            newStop*1.0E-9 - stepTime/2*1.0E-9)
        RWDynObject.ComputeDynamics(massPropsData, outputState,
            newStop*1.0E-9)
        newStop += threadCallPeriod
        TotalSim.TotalSim.SingleStepProcesses()
        TotalSim.RecordLogVars()


# get dyn object from RW class
RWDynObject = reactionwheel_dynamics.ReactionWheelDynamics()

#Configure a single reaction wheel
RWDynObject.ModelTag = "RWDynamics"

#number of reaction wheels need to know for buffer size
numReactionWheels = 3

#x-axis reaction wheel
RW1 = reactionwheel_dynamics.ReactionWheelConfigData()
RW1.r_S = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
RW1.gsHat_S = reactionwheel_dynamics.DoubleVector([1.0, 0.0, 0.0])
RW1.u_max = 1.0
RW1.u_min = 0.1
RW1.u_f = 0.01
RWDynObject.AddReactionWheel(RW1)

#y-axis reaction wheel
RW2 = reactionwheel_dynamics.ReactionWheelConfigData()
RW2.r_S = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
RW2.gsHat_S = reactionwheel_dynamics.DoubleVector([0.0, 1.0, 0.0])
RW2.u_max = 1.0
RW2.u_min = 0.1
RW2.u_f = 0.01
RWDynObject.AddReactionWheel(RW2)

#z-axis reaction wheel
RW3 = reactionwheel_dynamics.ReactionWheelConfigData()
RW3.r_S = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
RW3.gsHat_S = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 1.0])
RW3.u_max = 1.0
RW3.u_min = 0.1
RW3.u_f = 0.01
RWDynObject.AddReactionWheel(RW3)


#Set the test step (threadCallPeriod) and the various stop times (stopTime1-var4)
threadCallPeriod = int(1E7) # call the thread at 1000 Hz
stopTime1 = int(30*1E9) # stop after 60 seconds
stopTime2  = int(60*1E9) # next stop stop after 80 seconds

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass()

#Create a new process for the unit test task and add the module to tasking
RWUnitTestProc = TotalSim.CreateNewProcess("RWUnitTestProcess")
RWUnitTestProc.addTask(TotalSim.CreateNewTask("reactionwheelbasic", threadCallPeriod))
TotalSim.AddModelToTask("reactionwheelbasic", RWDynObject)

#Setup variables for logging
TotalSim.AddVectorForLogging('RWDynamics.tau_S', 'double', 0, 2, int(1E2))


#Configure the vehicle mass properties to use in the simulation
massPropsData = six_dof_eom.MassPropsData()
CoMLoc = [1.0, 0.0, 0.0]
SimulationBaseClass.SetCArray(CoMLoc, 'double', massPropsData.CoM)
T_str2Bdy = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
SimulationBaseClass.SetCArray(T_str2Bdy, 'double', massPropsData.T_str2Bdy)
outputState = six_dof_eom.OutputStateData()


cmdArray = sim_model.new_doubleArray(3)
sim_model.doubleArray_setitem(cmdArray, 0, 0.09)
sim_model.doubleArray_setitem(cmdArray, 1, 0.20)
sim_model.doubleArray_setitem(cmdArray, 2, 1.30)


#Configure command message
TotalSim.TotalSim.CreateNewMessage("RWUnitTestProcess", "reactionwheel_cmds", 8*numReactionWheels, 2)

##Step the simulation for a bit to get clear of any init funnies
TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(int(10*1E9-threadCallPeriod))
TotalSim.ExecuteSimulation()

#Write firing command message and step the simulation
TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8*numReactionWheels, 0, cmdArray );
executeSimulationRun(stopTime1, threadCallPeriod, TotalSim, RWDynObject,
    massPropsData, outputState)

cmdArray2 = sim_model.new_doubleArray(3)
sim_model.doubleArray_setitem(cmdArray2, 0, -0.09)
sim_model.doubleArray_setitem(cmdArray2, 1, -0.20)
sim_model.doubleArray_setitem(cmdArray2, 2, -1.30)

TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8*numReactionWheels, 0, cmdArray2 );
executeSimulationRun(stopTime2, threadCallPeriod, TotalSim, RWDynObject,
                     massPropsData, outputState)

#Extract log variables and plot the results
StrTorque = TotalSim.GetLogVariableData('RWDynamics.tau_S')

#Plot the results
plt.figure(1)
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,1], 'b', label='x Str')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,2], 'g', label='y Str')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,3], 'r', label = 'z Str')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.show()

print('goin'' to mars')
