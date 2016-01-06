# this is the initial version of the thruster unit test
#Very simple simulation.
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
#import thruster_dynamics

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
        newStop += var1
        TotalSim.TotalSim.SingleStepProcesses()
        TotalSim.RecordLogVars()


# get dyn object from RW class
RWDynObject = reactionwheel_dynamics.ReactionWheelDynamics()

#Configure a single reaction wheel
RWDynObject.ModelTag = "RWDynamics"

RW1 = reactionwheel_dynamics.ReactionWheelConfigData()
#RW1.ReactionWheelLocation = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
RW1.ReactionWheelDirection = reactionwheel_dynamics.DoubleVector([1.0, 0.0, 0.0])
RW1.MaxTorque = 1.0
RWDynObject.AddReactionWheel(RW1)

RW2 = reactionwheel_dynamics.ReactionWheelConfigData()
#RW2.ReactionWheelLocation = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
RW2.ReactionWheelDirection = reactionwheel_dynamics.DoubleVector([0.0, 1.0, 0.0])
RW2.MaxTorque = 1.0
RWDynObject.AddReactionWheel(RW2)
#
#RW3 = reactionwheel_dynamics.ReactionWheelConfigData()
##RW3.ReactionWheelLocation = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 0.0])
#RW3.ReactionWheelDirection = reactionwheel_dynamics.DoubleVector([0.0, 0.0, 1.0])
#RW3.MaxTorque = 1.0
#RWDynObject.AddReactionWheel(RW3)

##ACSpropCM = [0.0, 0.0, 1.0]
##ACSpropMass = 1.0 #Made up!!!!
##ACSpropRadius = 46.0/2.0/3.2808399/12.0
##sphereInerita = 2.0/5.0*ACSpropMass*ACSpropRadius*ACSpropRadius
##ACSInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita]
##RWDynObject.objProps.Mass = ACSpropMass
##SimulationBaseClass.SetCArray(ACSpropCM, 'double', 
##    RWDynObject.objProps.CoM)
##SimulationBaseClass.SetCArray(ACSInertia, 'double', 
##    RWDynObject.objProps.InertiaTensor)
#
##earthGrav = 9.80665
##mDotExpect = RW1.MaxThrust/(RW1.steadyIsp*earthGrav)
##print mDotExpect
#






#Set the test step (var1) and the various stop times (var2-var4)
var1 = int(1E7) # call the thread at 1000 Hz, let the thread do stuff at 10 Hz
var2 = int(60*1E9) # stop after 60 seconds
var3  = int(80*1E9) # next stop
#var4  = int(100*1E9) # next stop
#var5 = int(((110)*1E9)) #next stop
#var6 = int((110 + 0.02)*1E9) #next stop
#var7 = int(((120)*1E9)) #next stop
#var8 = int(((120+0.04)*1E9)) #next stop
#var9 = int(((130)*1E9)) #next stop
#var10 = int(((140)*1E9)) #next stop

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass()
#Create a new process for the unit test task and add the module to tasking
RWUnitTestProc = TotalSim.CreateNewProcess("RWUnitTestProcess")
RWUnitTestProc.addTask(TotalSim.CreateNewTask("reactionwheelbasic", var1))
TotalSim.AddModelToTask("reactionwheelbasic", RWDynObject)

##Configure the vehicle mass properties to use in the simulation
#massPropsData = six_dof_eom.MassPropsData()
#CoMLoc = [1.0, 0.0, 0.0]
#SimulationBaseClass.SetCArray(CoMLoc, 'double', massPropsData.CoM)
#T_str2Bdy = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
#SimulationBaseClass.SetCArray(T_str2Bdy, 'double', massPropsData.T_str2Bdy)
#outputState = six_dof_eom.OutputStateData()
#
#
#
#
#
#
#
#
#
#
#
#
#


#Setup variables for logging
TotalSim.AddVectorForLogging('RWDynamics.StrForce', 'double', 0, 2, int(1E2))
TotalSim.AddVectorForLogging('RWDynamics.BodyForce', 'double', 0, 2, int(1E2))
TotalSim.AddVectorForLogging('RWDynamics.StrTorque', 'double', 0, 2, int(1E2))
TotalSim.AddVectorForLogging('RWDynamics.BodyTorque', 'double', 0, 2, int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor', int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.objProps.Mass', int(1E2))
#TotalSim.AddVariableForLogging('ACSThrusterDynamics.mDotTotal', int(1E2))





#Configure a single thruster firing, create a message for it
RWMessage = reactionwheel_dynamics.RWCmdStruct()
RWMessage.TorqueRequest = 10.0;
#RWMessage.TorqueRequest = reactionwheel_dynamics.DoubleVector([10.0, 20.0, 30.0])
TotalSim.TotalSim.CreateNewMessage("RWUnitTestProcess", "reactionwheel_cmds", 8, 2) # what does this do



#Step the simulation for a bit to get clear of any init funnies
TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(int(10*1E9-var1))
TotalSim.ExecuteSimulation()


#Configure the vehicle mass properties to use in the simulation
massPropsData = six_dof_eom.MassPropsData()
CoMLoc = [1.0, 0.0, 0.0]
SimulationBaseClass.SetCArray(CoMLoc, 'double', massPropsData.CoM)
T_str2Bdy = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
SimulationBaseClass.SetCArray(T_str2Bdy, 'double', massPropsData.T_str2Bdy)
outputState = six_dof_eom.OutputStateData()


#Write firing command message and step the simulation
TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 0, RWMessage);
executeSimulationRun(var2, var1, TotalSim, RWDynObject,
    massPropsData, outputState)
   
#Configure a shorter firing command, execute it, and step simulation 
RWMessage.TorqueRequest = 20.0;
#RWMessage.TorqueRequest = reactionwheel_dynamics.DoubleVector([1.0, 2.0, 3.0])
TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, int(30E9), RWMessage);
executeSimulationRun(var3, var1, TotalSim, RWDynObject, massPropsData, outputState)




##Setup thruster ramp on and ramp off configuration
#rampOnList = []
#rampOffList = []
##Note that this ramp is totally linear and ramps up 30 ms using 30 steps
#for i in range(30):
#    newElement = reactionwheel_dynamics.ThrusterTimePair()
#    newElement.TimeDelta = (i+1)*0.001
#    newElement.ThrustFactor = (i+1.0)/30.0
#    newElement.IspFactor = (i+1.0)/30.0
#    rampOnList.append(newElement)
#    newElement = reactionwheel_dynamics.ThrusterTimePair()
#    newElement.TimeDelta = (i+1)*0.001
#    newElement.ThrustFactor = 1.0-(i+1.0)/30.0
#    newElement.IspFactor = newElement.ThrustFactor
#    rampOffList.append(newElement)
#
##Set up the ramps 
#RWDynObject.ThrusterData[0].ThrusterOnRamp = \
#    reactionwheel_dynamics.ThrusterTimeVector(rampOnList)
#RWDynObject.ThrusterData[0].ThrusterOffRamp = \
#    reactionwheel_dynamics.ThrusterTimeVector(rampOffList)
#
##Execute a new firing that will use the thruster ramps
#ThrustMessage.OnTimeRequest = 0.5;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 1, ThrustMessage);
#executeSimulationRun(var4, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.015;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 2, ThrustMessage);
#executeSimulationRun(var5, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.5;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 3, ThrustMessage);
#executeSimulationRun(var6, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.005;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 4, ThrustMessage);
#executeSimulationRun(var7, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.025;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 5, ThrustMessage);
#executeSimulationRun(var8, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.2;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 6, ThrustMessage);
#executeSimulationRun(var9, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#
#ThrustMessage.OnTimeRequest = 0.001;
#TotalSim.TotalSim.WriteMessageData("reactionwheel_cmds", 8, 7, ThrustMessage);
#executeSimulationRun(var10, var1, TotalSim, RWDynObject,
#    massPropsData, outputState)
#





#Extract log variables and plot the results
StrForce = TotalSim.GetLogVariableData('RWDynamics.StrForce')
BodyForce = TotalSim.GetLogVariableData('RWDynamics.BodyForce')
StrTorque = TotalSim.GetLogVariableData('RWDynamics.StrTorque')
BodyTorque = TotalSim.GetLogVariableData('RWDynamics.BodyTorque')
#thrBodForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.BodyForce')
#thrFactor = TotalSim.GetLogVariableData('ACSThrusterDynamics.ThrusterData[0].ThrustOps.ThrustFactor')
#thrMass = TotalSim.GetLogVariableData('ACSThrusterDynamics.objProps.Mass')
#thrMDot = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')
plt.figure(1)
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,1], 'b', label='x Str')
plt.plot(BodyTorque[:,0]*1.0E-9, BodyTorque[:,1], 'b--', label='x Body')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,2], 'g', label='y Str')
plt.plot(BodyTorque[:,0]*1.0E-9, BodyTorque[:,2], 'g--', label='y Body')
plt.plot(StrTorque[:,0]*1.0E-9, StrTorque[:,3], 'r', label = 'z Str')
plt.plot(BodyTorque[:,0]*1.0E-9, BodyTorque[:,3], 'r--', label='z Body')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')

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

#RWDynObject.InputCmds = "dummy_fire_not_work"
#TotalSim.InitializeSimulation()
#TotalSim.ConfigureStopTime(int(2E6))
#TotalSim.ExecuteSimulation()
##
##if(len(sys.argv) > 1):
##   if(sys.argv[1] == 'True'):
##      plt.show()

plt.show()

print('goin'' to mars')
