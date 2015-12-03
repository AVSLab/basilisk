import sys, os, inspect #following lines are to get all necessary files on search path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
sys.path.append(path + '/../IntegratedTests/')
import EMMSim #simulation type we are running
import matplotlib.pyplot as plt #plotting functions
import MonteCarloBaseClass #monte-carlo module for running dispersed simulations
import EMMSafeCapture #Startup script we are using
import numpy #Who doesn't like numpy?
import math #Got to have some math

#instantiate a monte-carlo handler so that we can run a dispersed scenario
monteCarloContainer = MonteCarloBaseClass.MonteCarloBaseClass()

#Default initialization in this case is a gaussian distribution from -1,1
b1AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[0]')
b2AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[1]')
b3AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[2]')
monteCarloContainer.addNewDispersion(b1AttitudeDisp)
monteCarloContainer.addNewDispersion(b2AttitudeDisp)
monteCarloContainer.addNewDispersion(b3AttitudeDisp)

b1RateDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttRateInit[0]',
    0.0, (math.pi/180.0,))
b2RateDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttRateInit[1]',
    0.0, (math.pi/180.0,))
b3RateDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttRateInit[2]',
    0.0, (math.pi/180.0,))
monteCarloContainer.addNewDispersion(b1RateDisp)
monteCarloContainer.addNewDispersion(b2RateDisp)
monteCarloContainer.addNewDispersion(b3RateDisp)


#Define the simulation type and the script that we will use to execute the simulations
simulationModule = EMMSim.EMMSim
executionModule = EMMSafeCapture.executeEMMSafeCapture

#Configure the monte-carlo handler with the necessary parameter for a run
monteCarloContainer.setSimulationObject(simulationModule) #simulation type to use
monteCarloContainer.setExecutionCount(100) #Number of simulations to run
monteCarloContainer.setRetainSimulationData(True) #Archive simulations as we go along
monteCarloContainer.setExecutionModule(executionModule) #Define the script to use for the run

#Command to go and execute the simulation configuration we have defined
monteCarloContainer.executeSimulations()

#Set up to plot the archived data from each simulation run
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
