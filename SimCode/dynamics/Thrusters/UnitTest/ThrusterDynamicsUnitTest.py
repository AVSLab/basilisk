# this is the initial version of the thruster unit test
#Very simple simulation.
import sys, os
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')

#Import all of the modules that we are going to call in this simulation
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import thruster_dynamics

ACSThrusterDynObject = thruster_dynamics.ThrusterDynamics()



ACSThrusterDynObject.ModelTag = "ACSThrusterDynamics"
Thruster1 = thruster_dynamics.ThrusterConfigData()
Thruster1.ThrusterLocation = thruster_dynamics.DoubleVector([1.125, 0.0, 2.0])
Thruster1.ThrusterDirection = thruster_dynamics.DoubleVector([
    math.cos(45.0*math.pi/180.0), math.sin(45.0*math.pi/180.0), 0.0])
Thruster1.MaxThrust = 0.9

ACSThrusterDynObject.ThrusterData = thruster_dynamics.ThrusterConfigVector([Thruster1])

var1 = int(1E8) # call the thread at 10 Hz, let the thread do stuff at 10 Hz

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass()
TotalSim.CreateNewTask("thrusterbasic", var1)

TotalSim.AddModelToTask("thrusterbasic", ACSThrusterDynObject)

var2 = int(60*144.0*1E9) # stop after 144 minutes


TotalSim.TotalSim.CreateNewMessage("acs_thruster_cmds", 8, 2)

ThrustMessage = thruster_dynamics.ThrustCmdStruct()
ThrustMessage.OnTimeRequest = 10.0;

TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 0, ThrustMessage);

TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(var2)
TotalSim.ExecuteSimulation()


ThrustMessage.OnTimeRequest = 100.0;
TotalSim.TotalSim.WriteMessageData("acs_thruster_cmds", 8, 1, ThrustMessage);

var2 = int(60*344.0*1E9) # stop after 344-144=200 minutes

TotalSim.ConfigureStopTime(var2)
TotalSim.ExecuteSimulation()




print "making sure this works"
