import sys, os, inspect #following lines are to get all necessary files on search path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
sys.path.append(path + '/../IntegratedTests/')
import AVSSim  # simulation type we are running
import matplotlib.pyplot as plt  # plotting functions
import MonteCarloBaseClass as mbc  # monte-carlo module for running dispersed simulations
import AVSSafeCapture  # Startup script we are using
import numpy as np  # Who doesn't like numpy?

# instantiate a monte-carlo handler so that we can run a dispersed scenario
monteCarloContainer = mbc.MonteCarloBaseClass()

# Define the simulation type and the script that we will use to execute the simulations
simulationModule = AVSSim.AVSSim
executionModule = AVSSafeCapture.executeAVSSafeCapture

# Configure the monte-carlo handler with the necessary parameter for a run
monteCarloContainer.setSimulationObject(simulationModule)  # simulation type to use
monteCarloContainer.setExecutionCount(20)  # Number of simulations to run
monteCarloContainer.setRetainSimulationData(True)  # Archive simulations as we go along
monteCarloContainer.setExecutionModule(executionModule)  # Define the script to use for the run

# Default initialization in this case is a gaussian distribution from -0.5, 0.5
attitudeDisp = mbc.NormalVectorCartDispersion('VehDynObject.AttitudeInit', 0.0, 0.5, ([-1.0, 1.0]))
monteCarloContainer.addNewDispersion(attitudeDisp)

rateDisp = mbc.NormalVectorCartDispersion('VehDynObject.AttRateInit', 0.0, 0.017, ([-0.5, 0.5]))
monteCarloContainer.addNewDispersion(rateDisp)

tankMassDisp = mbc.UniformDispersion('DVThrusterDynObject.objProps.Mass', ([500, 800]))
monteCarloContainer.addNewDispersion(tankMassDisp)

dryMassDisp = mbc.UniformDispersion('VehDynObject.baseMass', ([400, 700]))
monteCarloContainer.addNewDispersion(dryMassDisp)

inertiaDisp = mbc.InertiaTensorDispersion("VehDynObject.baseInertiaInit", 1.0, [-2.0, 2.0])
monteCarloContainer.addNewDispersion(inertiaDisp)

# Generate thruster dispersions
thrustInd = 0
while thrustInd < 8:
    # Thrusters thrust directions are dispersed by angle in radians off center of nozzle bore line.
    tmpVarName = "ACSThrusterDynObject.ThrusterData[" + str(thrustInd) + "].thrusterDirectionDisp"
    tmpThrusterDisp = mbc.NormalThrusterUnitDirectionVectorDispersion(tmpVarName, thrustInd, 0.1745, ([-0.1745, 0.1745]))
    monteCarloContainer.addNewDispersion(tmpThrusterDisp)

    # Thrusters magnitudes are dispersed by percentage of max thrust.
    # E.g. a dispersion of 1.0 is 100% error.
    tmpVarName = "ACSThrusterDynObject.ThrusterData[" + str(thrustInd) + "].thrusterMagDisp"
    tmpThrusterDisp = mbc.NormalDispersion(tmpVarName, 0.0, 0.1, ([-0.1, 0.1]))
    monteCarloContainer.addNewDispersion(tmpThrusterDisp)

    thrustInd += 1

# Command to go and execute the simulation configuration we have defined
monteCarloContainer.executeSimulations()

# Set up to plot the archived data from each simulation run
plt.figure(1)

for sim in monteCarloContainer.simList:
    solarArrayMiss = sim.pullMessageLogData("solar_array_sun_bore.missAngle") #Amount we've missed the sun by
    FSWsHat = sim.pullMessageLogData("css_wls_est.sHatBdy", range(3)) #estimated sun vector point in body frame
    DataCSSTruth = sim.GetLogVariableData('CSSPyramid1HeadA.sHatStr') #Truth sun vector point in spacecraft structure
    plt.subplot(3,1,1)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b')
    plt.subplot(3,1,2)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'b')
    plt.subplot(3,1,3)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'b')
    print FSWsHat[-1, :]


plt.subplot(3,1,1)
plt.xlabel('Time (s)')
plt.ylabel('Sun point x (-)')
plt.subplot(3,1,2)
plt.xlabel('Time (s)')
plt.ylabel('Sun point y (-)')
plt.subplot(3,1,3)
plt.xlabel('Time (s)')
plt.ylabel('Sun point z (-)')
plt.show()
