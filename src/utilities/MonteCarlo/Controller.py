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
# MonteCarlo module. Please read the accompanying README.md for usage information.
#
# Purpose:  This module is used to run a simulation with varying initial parameters.
# Author:   Nathan Bellowe
# Creation Date:  July. 20, 2017
#

import os
import random
import traceback
import shutil
import matplotlib.pyplot as plt

import copy

import gzip
import json
import cPickle as pickle

from multiprocessing import Pool, cpu_count
import signal

import numpy as np


class Controller:
    """
    The MonteCarloController class is used to run a monte carlo simulation.
    It is used to execute multiple runs of a simulation with varying initial parameters. Data from each run is retained
    in order to analyze differences in the simulation runs and the parameters used.
    """

    def __init__(self):
        self.executionCount = 0
        self.ICrunFlag = False
        self.icDirectory = ""
        self.numProcess = cpu_count()
        self.simParams = SimulationParameters(
            creationFunction=None,
            executionFunction=None,
            configureFunction=None,
            retentionPolicies=[],
            shouldArchiveParameters=False,
            shouldDisperseSeeds=False,
            dispersions=[],
            filename="",
            icfilename=""
        )

    @staticmethod
    def load(runDirectory):
        """ Load a previously completed MonteCarlo simulation
        Args:
            The path to the MonteCarlo.data file that contains the archived MonteCarlo run
        """
        filename = os.path.abspath(runDirectory) + "/MonteCarlo.data"

        with gzip.open(filename) as pickledData:
            data = pickle.load(pickledData)
            if data.simParams.verbose:
                print "Loading montecarlo at", filename
            return data

    def setExecutionFunction(self, newModule):
        """ Set an execution function that executes a simulation instance.
        Args:
            executionFunction: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creationFunction and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.simParams.executionFunction = newModule

    def setConfigureFunction(self, newModule):
        """ Set an execution function that executes a simulation instance.
        Args:
            executionFunction: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creationFunction and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.simParams.configureFunction = newModule

    def setSimulationFunction(self, newObject):
        """ Set the function that creates the simulation instance.
        Args:
            creationFunction: () => SimulationBaseClass
                A function with no parameters, that returns a simulation instance.
        """
        self.simParams.creationFunction = newObject

    def setShouldDisperseSeeds(self, seedDisp):
        """ Disperse the RNG seeds of each run in the MonteCarlo
        Args:
            seedDisp: bool
                Whether to disperse the RNG seeds in each run of the simulation
        """
        self.simParams.shouldDisperseSeeds = seedDisp

    def setExecutionCount(self, newCount):
        """ Set the number of runs for the MonteCarlo simulation
        Args:
            newCount: int
                The number of runs to use for the simulation
        """
        self.executionCount = newCount

    def addDispersion(self, disp):
        """ Add a dispersion to the simulation.
        Args:
            disp: Dispersion
                The dispersion to add to the simulation.
        """
        self.simParams.dispersions.append(disp)

    def addRetentionPolicy(self, policy):
        """ Add a retention policy to the simulation.
        Args:
            disp: RetentionPolicy
                The retention policy to add to the simulation.
                This defines variables to be logged and saved
        """
        self.simParams.retentionPolicies.append(policy)

    def setThreadCount(self, threads):
        """ Set the number of threads to use for the monte carlo simulation
        Args:
            threads: int
                Number of threads to execute the montecarlo run on.
        """
        self.numProcess = threads

    def setVerbose(self, verbose):
        """ Use verbose output for this MonteCarlo run
        Args:
            verbose: bool
                Whether to print verbose information during this MonteCarlo sim.
        """
        self.simParams.verbose = verbose

    def setShouldArchiveParameters(self, shouldArchiveParameters):
        self.simParams.shouldArchiveParameters = shouldArchiveParameters

    def setArchiveDir(self, dirName):
        """ Set-up archives for this MonteCarlo run
        Args:
            dirName: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.archiveDir = os.path.abspath(dirName) + "/"
        self.simParams.shouldArchiveParameters = dirName is not None
        self.simParams.filename = self.archiveDir

    def setICDir(self, dirName):
        """ Set-up archives containing IC data
        Args:
            dirName: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.icDirectory = os.path.abspath(dirName) + "/"
        self.simParams.shouldArchiveParameters = True
        self.simParams.icfilename = self.icDirectory

    def setICRunFlag(self, bool):
        """ Set the number of threads to use for the monte carlo simulation
        Args:
            threads: int
                Number of threads to execute the montecarlo run on.
        """
        self.ICrunFlag = bool

    def getRetainedData(self, case):
        """ Get the data that was retained for a run, or list of runs.
        Args:
            cases: int The desired case to get data from.
        Returns:
            The retained data for that run is returned.
        """
        if self.ICrunFlag:
            oldRunDataFile = self.icDirectory + "run" + str(case) + ".data"
        else:
            oldRunDataFile = self.archiveDir + "run" + str(case) + ".data"

        with gzip.open(oldRunDataFile) as pickledData:
            data = pickle.load(pickledData)
            return data

    def getRetainedDatas(self, cases):
        """ Get the data that was retained for a list of runs.
        Args:
            cases: int[] The desired cases to get data from.
        Returns:
            A generator is returned, which will yield, in-order, the retained data for each of these cases
        """

        for case in cases:
            yield self.getRetainedData(case)  # call this method recursively, yielding the result

    def getParameters(self, caseNumber):
        """ Get the parameters used for a particular run of the montecarlo
        Args:
            caseNumber: int
                The number of the run to get the parameters used for.
        Returns:
            A dictionary of the parameters of the simulation
            For example:
            {
                "keyForSim": parameterValue,
                'TaskList[0].TaskModels[0].RNGSeed': 1674764759
            }
        """
        if self.ICrunFlag:
            filename = self.icDirectory + "run" + str(caseNumber) + ".json"
        else:
            filename = self.archiveDir + "run" + str(caseNumber) + ".json"
        with open(filename, "r") as dispersionFile:
            dispersions = json.load(dispersionFile)
            return dispersions

    def reRunCases(self, caseList):
        """ Rerun some cases from a MonteCarlo run. Does not run in parallel
        Args:
            caseList: int[]
                The list of runs to repeat, a list of numbers.
        Returns:
            failures: int[]
                The list of failed runs.
        """
        # the list of failures
        failed = []

        for caseNumber in caseList:
            if self.simParams.verbose:
                print "Rerunning", caseNumber

            oldRunFile = self.archiveDir + "run" + str(caseNumber) + ".json"
            if not os.path.exists(oldRunFile):
                print "ERROR re-running case: " + oldRunFile
                continue

            # use old simulation parameters, modified slightly.
            simParams = copy.deepcopy(self.simParams)
            simParams.index = caseNumber
            # don't redisperse seeds, we want to use the ones saved in the oldRunFile
            simParams.shouldDisperseSeeds = False
            # don't retain any data so remove all retention policies
            simParams.retentionPolicies = []

            with open(oldRunFile, "r") as runParameters:
                simParams.modifications = json.load(runParameters)

            # execute simulation with dispersion
            executor = SimulationExecutor()
            success = executor(simParams)

            if not success:
                print "Error re-executing run", caseNumber
                failed.append(caseNumber)

        if len(failed) > 0:
            failed.sort()
            print "Failed rerunning cases:", failed

        return failed

    def runInitialConditions(self, caseList):
        """ Run initial conditions given in a file
        Args:
            caseList: int[]
                The list of runs to repeat, a list of numbers.
        Returns:
            failures: int[]
                The list of failed runs.
        """
        # the list of failures
        failed = []

        assert self.icDirectory is not "", "No initial condition directory was given"
        assert self.ICrunFlag is not False, "IC run flag was not set"

        if self.simParams.verbose:
            print "Beginning simulation with {0} runs on {1} threads".format(self.executionCount, self.numProcess)

        if self.simParams.shouldArchiveParameters:
            if not os.path.exists(self.icDirectory):
                print "Cannot run initial conditions: the directory given does not exist"
            if self.simParams.verbose:
                print "Archiving a copy of this simulation before running it in 'MonteCarlo.data'"
            try:
                with gzip.open(self.icDirectory + "MonteCarlo.data", "w") as pickleFile:
                    pickle.dump(self, pickleFile)  # dump this controller object into a file.
            except Exception as e:
                print "Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e

        simGenerator = self.generateICSims(caseList)
        jobsFinished = 0  # keep track of what simulations have finished

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker threads with each worker's simulation parameters
        simulationExecutor = SimulationExecutor()
        #
        if self.numProcess == 1:  # don't make child thread
            if self.simParams.verbose:
                print "Executing sequentially..."
            i = 0
            for sim in simGenerator:
                try:
                    simulationExecutor(sim)
                except:
                    failed.append(i)
                i += 1
        else:
            pool = Pool(self.numProcess)
            try:
                # yields results *as* the workers finish jobs
                for result in pool.imap_unordered(simulationExecutor, simGenerator):
                    if result[0] is not True:  # workers return True on success
                        failed.append(result[1])  # add failed jobs to the list of failures
                        print "Job", result[1], "failed..."

                    jobsFinished += 1
                pool.close()
            except KeyboardInterrupt as e:
                print "Ctrl-C was hit, closing pool"
                # failed.extend(range(jobsFinished, numSims))  # fail all potentially running jobs...
                pool.terminate()
                raise e
            except Exception as e:
                print "Unknown exception while running simulations:", e
                # failed.extend(range(jobsFinished, numSims))  # fail all potentially running jobs...
                traceback.print_exc()
                pool.terminate()
            finally:
                pool.join()

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.simParams.verbose:
                print "Failed", failed, "saving to 'failures.txt'"

            if self.simParams.shouldArchiveParameters:
                # write a file that contains log of failed runs
                with open(self.icDirectory + "failures.txt", "w") as failFile:
                    failFile.write(str(failed))

        return failed

    def generateICSims(self, caseList):
        ''' Generator function to clone a baseSimulation for IC run
        Args:
            baseSimulation: SimulationParams
                A base simulation to clone.
            numSims: int[]
                The desired runs to generate.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        '''

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for caseNumber in caseList:
            if self.simParams.verbose:
                print "Running IC ", caseNumber

            oldRunFile = self.icDirectory + "run" + str(caseNumber) + ".json"
            if not os.path.exists(oldRunFile):
                print "ERROR running IC case: " + oldRunFile
                continue

            # use old simulation parameters, modified slightly.
            simParams = copy.deepcopy(self.simParams)
            simParams.index = caseNumber
            # don't redisperse seeds, we want to use the ones saved in the oldRunFile
            simParams.shouldDisperseSeeds = False

            simParams.icfilename = self.icDirectory + "run" + str(caseNumber)
            with open(oldRunFile, "r") as runParameters:
                simParams.modifications = json.load(runParameters)

            yield simParams

    def generateSims(self, simNumList):
        ''' Generator function to clone a baseSimulation
        Args:
            baseSimulation: SimulationParams
                A base simulation to clone.
            numSims: int[]
                The desired runs to generate.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        '''

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for i in simNumList:
            simClone = copy.deepcopy(self.simParams)
            simClone.index = i
            simClone.filename += "run" + str(i)

            yield simClone

    def executeCallbacks(self, rng=None, retentionPolicies=[]):
        """ Execute retention policy callbacks after running a monteCarlo sim.
        rng: A list of simulations to execute callbacks on
        retentionPolicies: the retention policies to execute
        """

        if rng is None:
            rng = range(self.executionCount)

        if retentionPolicies == []:
            retentionPolicies = self.simParams.retentionPolicies

        for simIndex in rng:
            data = self.getRetainedData(simIndex)
            for retentionPolicy in retentionPolicies:
                retentionPolicy.executeCallback(data)
                

    def executeSimulations(self):
        ''' Execute simulations in parallel
        Args: None
        Returns:
            failed: int[]
                A list of the indices of all failed simulation runs.
        '''

        if self.simParams.verbose:
            print "Beginning simulation with {0} runs on {1} threads".format(self.executionCount, self.numProcess)

        if self.simParams.shouldArchiveParameters:
            if os.path.exists(self.archiveDir):
                shutil.rmtree(self.archiveDir)
            os.mkdir(self.archiveDir)
            if self.simParams.verbose:
                print "Archiving a copy of this simulation before running it in 'MonteCarlo.data'"
            try:
                with gzip.open(self.archiveDir + "MonteCarlo.data", "w") as pickleFile:
                    pickle.dump(self, pickleFile)  # dump this controller object into a file.
            except Exception as e:
                print "Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e

        numSims = self.executionCount

        # Avoid building a full list of all simulations to run in memory,
        # instead only generating simulations right before they are needed by a waiting worker
        # This is accomplished using a generator and pool.imap, -- simulations are only built
        # when they are about to be passed to a worker, avoiding memory overhead of first building simulations
        # There is a system-dependent chunking behavior, sometimes 10-20 are generated at a time.
        simGenerator = self.generateSims(range(numSims))
        failed = []  # keep track of the indices of failed simulations
        jobsFinished = 0  # keep track of what simulations have finished

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker threads with each worker's simulation parameters
        simulationExecutor = SimulationExecutor()

        if self.numProcess == 1:  # don't make child thread
            if self.simParams.verbose:
                print "Executing sequentially..."
            i = 0
            for sim in simGenerator:
                try:
                    simulationExecutor(sim)
                except:
                    failed.append(i)
                i += 1
        else:
            pool = Pool(self.numProcess)
            try:
                # yields results *as* the workers finish jobs
                for result in pool.imap_unordered(simulationExecutor, simGenerator):
                    if result[0] is not True:  # workers return True on success
                        failed.append(result[1])  # add failed jobs to the list of failures
                        print "Job", result[1], "failed..."

                    jobsFinished += 1
                    if self.simParams.verbose:
                        if jobsFinished % max(1, numSims / 20) == 0:  # print percentage after every ~5%
                            print "Finished", jobsFinished, "/", numSims, \
                                  "\t-- {}%".format(int(100 * float(jobsFinished) / numSims))
                pool.close()
            except KeyboardInterrupt as e:
                print "Ctrl-C was hit, closing pool"
                failed.extend(range(jobsFinished, numSims))  # fail all potentially running jobs...
                pool.terminate()
                raise e
            except Exception as e:
                print "Unknown exception while running simulations:", e
                failed.extend(range(jobsFinished, numSims))  # fail all potentially running jobs...
                traceback.print_exc()
                pool.terminate()
            finally:
                pool.join()

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.simParams.verbose:
                print "Failed", failed, "saving to 'failures.txt'"

            if self.simParams.shouldArchiveParameters:
                # write a file that contains log of failed runs
                with open(self.archiveDir + "failures.txt", "w") as failFile:
                    failFile.write(str(failed))

        return failed


class SimulationParameters():
    '''
    This class represents the run parameters for a simulation, with information including
     - a function that creates the simulation
     - a function that executes the simulation
     - the dispersions to use on that simulation
     - parameters describing the data to be retained for a simulation
     - whether randomized seeds should be applied to the simulation
     - whether data should be archived
    '''

    def __init__(self, creationFunction, executionFunction, configureFunction,
                 retentionPolicies, dispersions, shouldDisperseSeeds,
                 shouldArchiveParameters, filename, icfilename, index=None, verbose=False, modifications={}):
        self.index = index
        self.creationFunction = creationFunction
        self.executionFunction = executionFunction
        self.configureFunction = configureFunction
        self.retentionPolicies = retentionPolicies
        self.dispersions = dispersions
        self.shouldDisperseSeeds = shouldDisperseSeeds
        self.shouldArchiveParameters = shouldArchiveParameters
        self.filename = filename
        self.icfilename = icfilename
        self.verbose = verbose
        self.modifications = modifications


class VariableRetentionParameters:
    """
    Represents a variable's logging parameters.
    """

    def __init__(self, varName, varRate, startIndex=0, stopIndex=0, varType='double'):
        self.varName = varName
        self.varRate = varRate
        self.startIndex = startIndex
        self.stopIndex = stopIndex
        self.varType = varType


class MessageRetentionParameters:
    """
    Represents a message's logging parameters.
    Args:
        messageName: the name of the message to log
        messageRate: rate to log the message at.
        dataType: the dataType to pull
    """

    def __init__(self, messageName, messageRate, retainedVars):
        self.messageName = messageName
        self.messageRate = messageRate
        self.retainedVars = retainedVars


class RetentionPolicy():

    def __init__(self, rate=int(1E10)):
        self.logRate = rate
        self.messageLogList = []
        self.varLogList = []
        self.dataCallback = None
        self.retentionFunctions = []

    def addMessageLog(self, messageName, retainedVars, logRate=None):
        if logRate is None:
            logRate = self.logRate
        self.messageLogList.append(MessageRetentionParameters(messageName, logRate, retainedVars))

    def addVariableLog(self, variableName, startIndex=0, stopIndex=0, varType='double', logRate=None):
        if logRate is None:
            logRate = self.logRate
        varContainer = VariableRetentionParameters(variableName, logRate, startIndex, stopIndex, varType)
        self.varLogList.append(varContainer)

    def addLogsToSim(self, simInstance):
        for message in self.messageLogList:
            simInstance.TotalSim.logThisMessage(message.messageName, message.messageRate)
        for variable in self.varLogList:
            simInstance.AddVariableForLogging(variable.varName, variable.varRate,
                                              variable.startIndex, variable.stopIndex, variable.varType)

    def addRetentionFunction(self, function):
        self.retentionFunctions.append(function)

    def setDataCallback(self, dataCallback):
        self.dataCallback = dataCallback

    def executeCallback(self, data):
        if self.dataCallback is not None:
            self.dataCallback(data, self)

    @staticmethod
    def addRetentionPoliciesToSim(simInstance, retentionPolicies):
        """ Adds logs for variables and messages to a simInstance
        Args:
            simInstance: The simulation instance to add logs to.
            retentionPolicies: RetentionPolicy[] list that defines the data to log.
        """

        for retentionPolicy in retentionPolicies:
            retentionPolicy.addLogsToSim(simInstance)

        # TODO handle duplicates somehow?

    @staticmethod
    def getDataForRetention(simInstance, retentionPolicies):
        """ Returns the data that should be retained given a simInstance and the retentionPolicies
        Args:
            simInstance: The simulation instance to retrive data from
            retentionPolicies: A list of RetentionPolicy objects defining the data to retain
        Returns:
            Retained Data: In the form of a dictionary with two sub-dictionaries for messages and variables:
            {
                "messages": {
                    "messageName": [value1,value2,value3]
                },
                "variables": {
                    "variableName": [value1,value2,value3]
                }
            }
        """

        data = {}
        data["messages"] = {}
        data["variables"] = {}
        data["custom"] = {}

        for retentionPolicy in retentionPolicies:
            for message in retentionPolicy.messageLogList:
                for (param, dataType) in message.retainedVars:
                    name = message.messageName + "." + param
                    if dataType is None:
                        msg = simInstance.pullMessageLogData(name)
                    else:
                        msg = simInstance.pullMessageLogData(name, dataType)
                    data["messages"][name] = msg

            for variable in retentionPolicy.varLogList:
                data["variables"][variable.varName] = simInstance.GetLogVariableData(variable.varName)

            for func in retentionPolicy.retentionFunctions:
                tmpModuleData = func(simInstance)
                for (key, value) in tmpModuleData.iteritems():
                    data["custom"][key] = value

        return data


class SimulationExecutor():
    '''
    This class is used to execute a simulation in a worker thread.
    To use, create an instance of this class, and then call the instance with the simulation parameters to run them in.

    executor = SimulationExecutor()
    simParams = SimulationParameters()
    successFlag = executor(simParams)

    This class can be used to execute a simulation on a different thread, by using this class as the processes target.
    Note, this class has no instance variables, it is used essentially as a collection of static methods.
    '''

    @classmethod
    def __call__(cls, simParams):
        ''' In each worker process, we execute this function (by calling this object)
        Args:
            simParams: SimulationParameters
                The simulation parameters for the simulation to be executed.
        Returns:
            success: bool
                (True, simParams.index) if simulation run was successful
                (False, simParams.index) if simulation run was unsuccessful
        '''

        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)  # On ctrl-c ignore the signal... let the parent deal with it.

            # must make new random seed on each new thread.
            np.random.seed(simParams.index * 10)
            random.seed(simParams.index * 10)

            # create the users sim by calling their supplied creationFunction
            simInstance = simParams.creationFunction()

            # build a list of the parameter and random seed modifications to make
            modifications = simParams.modifications

            # we may want to disperse random seeds
            if simParams.shouldDisperseSeeds:
                # generate the random seeds for the model (but don't apply them yet)
                randomSeedDispersions = cls.disperseSeeds(simInstance)
                for name, value in randomSeedDispersions.items():
                    modifications[name] = value

            # we may want to disperse parameters
            for disp in simParams.dispersions:
                name = disp.getName()
                if name not in modifications:  # could be using a saved parameter.
                    modifications[name] = disp.generateString(simInstance)

            # if archiving, this run's parameters and random seeds are saved in its own json file
            if simParams.shouldArchiveParameters:
                # save the dispersions and random seeds for this run
                if simParams.icfilename != "":
                    with open(simParams.icfilename + ".json", 'w') as outfile:
                        json.dump(modifications, outfile)
                else:
                    with open(simParams.filename + ".json", 'w') as outfile:
                        json.dump(modifications, outfile)

            if simParams.configureFunction is not None:
                if simParams.verbose:
                    print "Configuring sim"
                simParams.configureFunction(simInstance)

            # apply the dispersions and the random seeds
            for variable, value in modifications.items():
                disperseStatement = "simInstance." + variable + "=" + value
                if simParams.verbose:
                    print "Executing parameter modification -> ", disperseStatement
                exec disperseStatement

            # setup data logging
            if len(simParams.retentionPolicies) > 0:
                if simParams.verbose:
                    print "Adding retained data"
                RetentionPolicy.addRetentionPoliciesToSim(simInstance, simParams.retentionPolicies)

            if simParams.verbose:
                print "Executing simulation"
            # execute the simulation, with the user-supplied executionFunction
            simParams.executionFunction(simInstance)

            if len(simParams.retentionPolicies) > 0:
                if simParams.icfilename != "":
                    retentionFile = simParams.icfilename + ".data"
                else:
                    retentionFile = simParams.filename + ".data"

                if simParams.verbose:
                    print "Retaining data for run in", retentionFile

                with gzip.open(retentionFile, "w") as archive:
                    retainedData = RetentionPolicy.getDataForRetention(simInstance, simParams.retentionPolicies)
                    retainedData["index"] = simParams.index # add run index
                    pickle.dump(retainedData, archive)

            if simParams.verbose:
                print "Terminating simulation"

            # terminate the simulation
            simInstance.terminateSimulation()

            if simParams.verbose:
                print "Thread", os.getpid(), "Job", simParams.index, "finished successfully"

            return (True, simParams.index)  # this function returns true only if the simulation was successful

        except Exception as e:
            print "Error in worker thread", e
            traceback.print_exc()
            return (False, simParams.index)  # there was an error

    @staticmethod
    def disperseSeeds(simInstance):
        """  disperses the RNG seeds of all the tasks in the sim, and returns a statement that contains the seeds
        Args:
            simInstance: SimulationBaseClass
                A basilisk simulation to set random seeds on
        Returns:
            statement: string
                A dictionary with the random seeds that should be applied to the sim:
                Example:
                ""
                {
                    '.TaskList[0].TaskModels[1]': 1934586,
                    '.TaskList[0].TaskModels[2]': 3450093,
                    '.TaskList[1].TaskModels[0]': 2221934,
                    '.TaskList[2].TaskModels[0]': 1123244
                }
                ""
        """

        randomSeeds = {}
        for i, task in enumerate(simInstance.TaskList):
            for j, model in enumerate(task.TaskModels):
                taskVar = 'TaskList[' + str(i) + '].TaskModels' + '[' + str(j) + '].RNGSeed'
                rand = str(random.randint(0, 1 << 32 - 1))
                try:
                    execStatement = "simInstance." + taskVar + "=" + str(rand)
                    exec execStatement  # if this fails don't add to the list of modification
                    randomSeeds[taskVar] = rand
                except:
                    pass
        return randomSeeds
