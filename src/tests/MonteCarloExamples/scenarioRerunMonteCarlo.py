
import inspect
import os, sys

from Basilisk.utilities.MonteCarlo.Controller import Controller
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy

filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

sys.path.append(path+"/../bskSimScenarios/scenarios/")

def main(time=None):
    '''
    Instructions:
    1) Change the scenario name
    2) Provide the number of processes to spawn
    3) Provide the run numbers you wish to rerun
    4) Add any new retention policies to the bottom
    '''

    # Step 1-3: Change to the relevant scenario
    scenarioName = "scenario_AttFeedback"

    monteCarlo = Controller()
    monteCarlo.numProcess = 3 # Specify number of processes to spawn
    runsList = [1]  # Specify the run numbers to be rerun

    #
    # # Generic initialization
    # exec("import " + scenarioName)
    icName = path + "/" + scenarioName + "MC/"
    newDataDir = path + "/" + scenarioName + "MC/rerun"

    exec("simulationModule = "+scenarioName + "." + scenarioName) # ex. scenarioMonteCarlo.scenarioMonteCarlo
    if time is not None:
        exec (scenarioName + '.' + scenarioName + '.simBaseTime = time')  # ex. scenarioMonteCarlo.scenarioMonteCarlo.simBaseTime = time
    exec("executionModule =" + scenarioName + ".runScenario") # ex. scenarioMonteCarlo.run

    monteCarlo.setSimulationFunction(simulationModule)
    monteCarlo.setExecutionFunction(executionModule)
    monteCarlo.setICDir(icName)
    monteCarlo.setICRunFlag(True)
    monteCarlo.setArchiveDir(newDataDir)
    monteCarlo.setExecutionCount(len(runsList))
    monteCarlo.setShouldDisperseSeeds(False)
    monteCarlo.shouldArchiveParameters = False


    # Step 4: Add any additional retention policies desired
    retentionPolicy = RetentionPolicy()
    retentionPolicy.logRate = int(2E9)
    retentionPolicy.addMessageLog("eclipse_data_0", [("shadowFactor", range(1))]) # 1.0 is no eclipse
    monteCarlo.addRetentionPolicy(retentionPolicy)


    failed = monteCarlo.runInitialConditions(runsList)
    assert len(failed) == 0, "Should run ICs successfully"



if __name__ == "__main__":
    main()

