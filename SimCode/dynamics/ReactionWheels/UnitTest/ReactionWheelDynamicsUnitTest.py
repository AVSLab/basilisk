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

def v3DoubleSet(v1,v2,v3):
 vout = sim_model.new_doubleArray(36)
 sim_model.doubleArray_setitem(vout, 0, v1)
 sim_model.doubleArray_setitem(vout, 1, v2)
 sim_model.doubleArray_setitem(vout, 2, v3)
 return vout

def defaultReactionWheel():
 RW = reactionwheel_dynamics.ReactionWheelConfigData()
 RW.r_S = v3DoubleSet(0.9,0.0,0.0)
 RW.gsHat_S = v3DoubleSet(1.0,0.0,0.0)
 RW.ggHat0_S = v3DoubleSet(0.0,1.0,0.0)
 RW.gtHat0_S = v3DoubleSet(0.0,0.0,1.0)
 RW.u_max = 1.0 # arbitrary
 RW.u_min = 0.1 # arbitrary
 RW.u_f = 0.01 # arbitrary
 RW.U_s = 8.5E-6; # kg-m, Honeywell HR-16 100 Nms standard balance option EOL
 RW.U_s *= 1000
 RW.U_d = 28.3E-7; # kg-m^2, Honeywell HR-16 100 Nms standard balance option EOL
 RW.U_d *= 1000
 RW.usingRWJitter = True
 return RW


# get dyn object from RW class
RWDynObject = reactionwheel_dynamics.ReactionWheelDynamics()

#Configure a single reaction wheel
RWDynObject.ModelTag = "RWDynamics"

#number of reaction wheels need to know for buffer size
numReactionWheels = 3

#x-axis reaction wheel
RW1 = defaultReactionWheel()
RWDynObject.AddReactionWheel(RW1)

#y-axis reaction wheel
RW2 = defaultReactionWheel()
RW2.r_S = v3DoubleSet(0.0,0.9,0.0)
RW2.gsHat_S = v3DoubleSet(0.0,1.0,0.0)
RW2.ggHat0_S = v3DoubleSet(1.0,0.0,0.0)
RW2.gtHat0_S = v3DoubleSet(0.0,0.0,-1.0)
RWDynObject.AddReactionWheel(RW2)

#z-axis reaction wheel
RW3 = defaultReactionWheel()
RW3.r_S = v3DoubleSet(0.0,0.0,0.9)
RW3.gsHat_S = v3DoubleSet(0.0,0.0,1.0)
RW3.ggHat0_S = v3DoubleSet(1.0,0.0,0.0)
RW3.gtHat0_S = v3DoubleSet(0.0,1.0,0.0)
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
TotalSim.AddVectorForLogging('RWDynamics.sumTau_B', 'double', 0, 2, int(1E2))


#Configure the vehicle mass properties to use in the simulation
massPropsData = six_dof_eom.MassPropsData()
CoMLoc = [1.0, 0.0, 0.0]
SimulationBaseClass.SetCArray(CoMLoc, 'double', massPropsData.CoM)
T_str2Bdy = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
SimulationBaseClass.SetCArray(T_str2Bdy, 'double', massPropsData.T_str2Bdy)
outputState = six_dof_eom.OutputStateData()

#first set of commands
cmdArray = v3DoubleSet(0.09,0.20,1.30)

#Configure command message
TotalSim.TotalSim.CreateNewMessage("RWUnitTestProcess", "reactionwheel_cmds", 8*36, 2)

##Step the simulation for a bit to get clear of any init funnies
TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(int(10*1E9-threadCallPeriod))
TotalSim.ExecuteSimulation()

#Write firing command message and step the simulation
TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8*36, 1, cmdArray );
executeSimulationRun(stopTime1, threadCallPeriod, TotalSim, RWDynObject,
    massPropsData, outputState)

#second set of commands
cmdArray2 = v3DoubleSet(-0.09,-0.20,-1.30)

TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8*36, 2, cmdArray2 );
executeSimulationRun(stopTime2, threadCallPeriod, TotalSim, RWDynObject,
                     massPropsData, outputState)

#Extract log variables and plot the results
StrTorque = TotalSim.GetLogVariableData('RWDynamics.sumTau_B')

#Plot the results
plt.figure(1)
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,1], 'b', label='x Str')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,2], 'g', label='y Str')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,3], 'r', label = 'z Str')
plt.legend()
plt.xlabel('Time  [ s ]')
plt.ylabel('Torque  [ N-m ]')
plt.show()

print('goin'' to mars')
