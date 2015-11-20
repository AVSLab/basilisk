import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
sys.path.append(path + '/../IntegratedTests/')
import EMMSim
import matplotlib.pyplot as plt
import MonteCarloBaseClass
import EMMSafeCapture
import numpy
import ctypes

monteCarloContainer = MonteCarloBaseClass.MonteCarloBaseClass()

b1AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[0]')
b2AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[1]')
b3AttitudeDisp = MonteCarloBaseClass.VariableDispersion('VehDynObject.AttitudeInit[2]')
monteCarloContainer.addNewDispersion(b1AttitudeDisp)
monteCarloContainer.addNewDispersion(b2AttitudeDisp)
monteCarloContainer.addNewDispersion(b3AttitudeDisp)

simulationModule = EMMSim.EMMSim
executionModule = EMMSafeCapture.executeEMMSafeCapture

monteCarloContainer.setSimulationObject(simulationModule)
monteCarloContainer.setExecutionCount(300)
monteCarloContainer.setRetainSimulationData(True)
monteCarloContainer.setExecutionModule(executionModule)

monteCarloContainer.executeSimulations()
plt.figure(1)

for sim in monteCarloContainer.simList:
    PosVec = ctypes.cast(sim.CSSWlsEstFSWConfig.OutputData.sHatBdy.__long__(),
                     ctypes.POINTER(ctypes.c_double))
    print PosVec[0], PosVec[1], PosVec[2]
    solarArrayMiss = sim.pullMessageLogData("solar_array_sun_bore.missAngle")
    posVector = sim.pullMessageLogData("inertial_state_output.r_N", range(3))
    FSWsHat = sim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
    DataCSSTruth = sim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
    plt.subplot(3,1,1)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b')
    plt.subplot(3,1,2)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'b')
    plt.subplot(3,1,3)
    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'b')
    print FSWsHat[-1, :]
    print posVector[-1, :]
#    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'g',
#        DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
#    plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'r',
#        DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')


plt.show()
