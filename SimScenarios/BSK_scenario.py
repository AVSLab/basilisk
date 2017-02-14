import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

import BSK_sim as BSKSim
import BSK_plotting as BSKPlt
import orbitalMotion
import macros as mc
import unitTestSupport as sp


if __name__ == "__main__":
    TheBSKSim = BSKSim.BSKSim()

    # DATA LOGGING
    simulationTime = mc.min2nano(10.)
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.mrpFeedbackData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.trackingErrorData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.simpleNavObject.outputTransName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.simpleNavObject.outputAttName, samplingTime)

    # Initialize Simulation
    TheBSKSim.InitializeSimulation()
    # The next call ensures that the FSW and Dynamics Message that have the same
    # name are copied over every time the simulation ticks forward.  This function
    # has to be called after the simulation is initialized to ensure that all modules
    # have created their own output/input messages declarations.
    TheBSKSim.dyn2FSWInterface.discoverAllMessages()
    TheBSKSim.fsw2DynInterface.discoverAllMessages()

    # Initialize Spacecraft States within the state manager.
    # This must occur after the initialization
    posRef = TheBSKSim.scObject.dynManager.getStateObject("hubPosition")
    velRef = TheBSKSim.scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = TheBSKSim.scObject.dynManager.getStateObject("hubSigma")
    omegaRef = TheBSKSim.scObject.dynManager.getStateObject("hubOmega")

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    mu = TheBSKSim.earthGravBody.mu
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * mc.D2R
    oe.Omega = 48.2 * mc.D2R
    oe.omega = 347.8 * mc.D2R
    oe.f = 85.3 * mc.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    posRef.setState(sp.np2EigenVectorXd(rN))  # r_BN_N [m]
    velRef.setState(sp.np2EigenVectorXd(vN))  # r_BN_N [m]
    sigmaRef.setState([[0.1], [0.2], [-0.3]])  # sigma_BN_B
    omegaRef.setState([[0.001], [-0.01], [0.03]])  # omega_BN_B [rad/s]

    # Configure a simulation stop time time and execute the simulation run

    TheBSKSim.modeRequest = 'hillPoint'
    TheBSKSim.ConfigureStopTime(simulationTime)
    TheBSKSim.ExecuteSimulation()

    # Retrieve the logged data
    dataLr = TheBSKSim.pullMessageLogData(TheBSKSim.mrpFeedbackData.outputDataName + ".torqueRequestBody", range(3))
    dataSigmaBR = TheBSKSim.pullMessageLogData(TheBSKSim.trackingErrorData.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = TheBSKSim.pullMessageLogData(TheBSKSim.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
    dataPos = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputTransName + ".r_BN_N", range(3))
    dataVel = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputTransName + ".v_BN_N", range(3))
    dataSigmaBN = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputAttName + ".sigma_BN", range(3))

    BSKPlt.plotResults(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN)


