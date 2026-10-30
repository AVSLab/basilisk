
 # ISC License
 #
 # Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 #
 # Permission to use, copy, modify, and/or distribute this software for any
 # purpose with or without fee is hereby granted, provided that the above
 # copyright notice and this permission notice appear in all copies.
 #
 # THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 # WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 # MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 # ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 # WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 # ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 # OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


#
# MonteCarlo module. Please read the accompanying README.md for usage information.
#
# Purpose:  This module is used to run a simulation with varying initial parameters.
# Author:   Nathan Bellowe
# Creation Date:  July. 20, 2017
#

import os
import random
import shutil
import traceback
import warnings
import ast

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)
import copy
import gzip
import json
import signal
from numbers import Integral
from operator import index
from queue import Full
import numpy as np
import multiprocessing as mp
import pickle as pickle
from Basilisk.utilities.MonteCarlo.DataWriter import DataWriter
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy
from Basilisk.utilities.simulationProgessBar import SimulationProgressBar


_WRITER_POLL_INTERVAL = 0.1  # [s]


def _check_writer(writer):
    """Raise if a writer exits before receiving its end marker."""
    if writer is not None and writer.exitcode is not None:
        raise RuntimeError(
            f"Monte Carlo data writer exited unexpectedly (exit code {writer.exitcode})"
        )


def _initialize_worker():
    """Let the controller handle keyboard interrupts for pool workers."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)


class _CheckedRetentionQueue:
    """Check writer health while the calling process waits for queue space."""

    def __init__(self, queue, writer):
        self.queue = queue
        self.writer = writer

    def put(self, item):
        """Enqueue an item or raise if the writer has exited."""
        while True:
            _check_writer(self.writer)
            try:
                self.queue.put(item, timeout=_WRITER_POLL_INTERVAL)
                return
            except Full:
                continue


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
        self.archiveDir = None
        self.varCast = None
        self.numProcess = mp.cpu_count()
        self.maxTasksPerChild = 8
        self.retentionQueueSize = None
        self.multiProcManager = None
        self.dataOutQueue = None
        self.dataWriter = None

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

    def _start_data_writer(self, logDir, workers):
        """Start a writer with bounded retention storage when requested."""
        if logDir is None:
            return
        self.multiProcManager = mp.Manager()
        capacity = getattr(self, "retentionQueueSize", None) or 2 * workers
        self.dataOutQueue = self.multiProcManager.Queue(maxsize=capacity)
        self.dataWriter = DataWriter(self.dataOutQueue)
        self.dataWriter.daemon = False
        self.dataWriter.setLogDir(logDir)
        self.dataWriter.setVarCast(self.varCast)
        self.dataWriter.start()

    def _shutdown_data_writer(self, abort=False):
        """Finish retained data, or stop immediately when aborting a campaign.

        :param bool abort: Stop the writer without consolidating partial data.
        :raises RuntimeError: The writer exits before completing its output.
        """
        writer = self.dataWriter
        try:
            if writer is not None and writer.pid is not None and not abort:
                queue = _CheckedRetentionQueue(self.dataOutQueue, writer)
                queue.put((None, None, True))
                writer.join()
                if writer.exitcode != 0:
                    raise RuntimeError(
                        f"Monte Carlo data writer failed (exit code {writer.exitcode})"
                    )
        finally:
            try:
                if writer is not None and writer.pid is not None:
                    if writer.is_alive():
                        writer.terminate()
                    writer.join()
            finally:
                try:
                    if self.multiProcManager is not None:
                        self.multiProcManager.shutdown()
                finally:
                    self.dataWriter = None
                    self.dataOutQueue = None
                    self.multiProcManager = None

    def _execute_cases(self, caseList, generator, logDir):
        """Execute bounded batches in one pool and supervise retained output."""
        if len(caseList) == 0:
            return []
        workers = min(self.numProcess, len(caseList))
        if workers < 1:
            raise ValueError("Monte Carlo worker count must be positive")
        failed = []
        finished = 0
        pool = None
        progressBar = SimulationProgressBar(len(caseList), self.simParams.showProgressBar)
        try:
            self._start_data_writer(logDir, workers)
            executor = SimulationExecutor()
            if self.numProcess > 1:
                pool = mp.Pool(
                    workers,
                    initializer=_initialize_worker,
                    maxtasksperchild=getattr(self, "maxTasksPerChild", 8),
                )
            queue = self.dataOutQueue
            if pool is None and queue is not None:
                queue = _CheckedRetentionQueue(queue, self.dataWriter)
            for start in range(0, len(caseList), workers):
                _check_writer(self.dataWriter)
                # Normalize NumPy indices without materializing the entire campaign.
                cases = [index(case) for case in caseList[start:start + workers]]
                try:
                    batch = [(sim, queue) for sim in generator(cases)]
                except Exception as error:
                    _check_writer(self.dataWriter)
                    print("Error preparing Monte Carlo simulations:", error)
                    traceback.print_exc()
                    # Nothing in this batch has been submitted yet.
                    failed.extend(cases)
                    finished += len(cases)
                    progressBar.update(finished)
                    continue
                if pool is None:
                    results = map(executor, batch)
                else:
                    results = pool.imap_unordered(executor, batch, chunksize=1)
                pending = [sim.index for sim, _ in batch]
                remaining = len(batch)
                while remaining:
                    _check_writer(self.dataWriter)
                    try:
                        if pool is None:
                            result = next(results)
                        else:
                            result = results.next(_WRITER_POLL_INTERVAL)
                    except mp.TimeoutError:
                        continue
                    except Exception as error:
                        if pool is None:
                            raise
                        # A submission error consumes one result without its
                        # run index. Collect the missing indices after draining
                        # the rest of this batch, including successful runs.
                        print("Error receiving Monte Carlo worker result:", error)
                        traceback.print_exc()
                    else:
                        pending.remove(result[1])
                        if not result[0]:
                            failed.append(result[1])
                    _check_writer(self.dataWriter)
                    remaining -= 1
                    finished += 1
                    progressBar.update(finished)
                failed.extend(pending)
            if pool is not None:
                pool.close()
                pool.join()
                pool = None
            self._shutdown_data_writer()
            progressBar.markComplete()
        except BaseException:
            try:
                if pool is not None:
                    pool.terminate()
                    pool.join()
            finally:
                self._shutdown_data_writer(abort=True)
            raise
        finally:
            progressBar.close()
        return sorted(failed)

    def setShowProgressBar(self, value):
        """
        To enable or disable progress bar to show simulation progress
        Args:
            value: boolean value, decide to show/hide progress bar
        """
        self.simParams.showProgressBar = value

    @staticmethod
    def load(runDirectory):
        """
        Load a previously completed MonteCarlo simulation
        Args:
            The path to the MonteCarlo.data file that contains the archived MonteCarlo run
        """
        filename = os.path.abspath(runDirectory) + "/MonteCarlo.data"

        with gzip.open(filename) as pickledData:
            data = pickle.load(pickledData)
            if data.simParams.verbose:
                print("Loading montecarlo at", filename)
            # Defer multiprocessing resources until they are required.
            data.multiProcManager = None
            data.dataOutQueue = None
            data.dataWriter = None
            return data

    def setExecutionFunction(self, newModule):
        """
        Set an execution function that executes a simulation instance.

        Args:
            executionFunction: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creationFunction and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.simParams.executionFunction = newModule

    def setConfigureFunction(self, newModule):
        """
        Set an execution function that executes a simulation instance.

        Args:
            executionFunction: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creationFunction and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.simParams.configureFunction = newModule

    def setSimulationFunction(self, newObject):
        """
        Set the function that creates the simulation instance.

        Args:
            creationFunction: () => SimulationBaseClass
                A function with no parameters, that returns a simulation instance.
        """
        self.simParams.creationFunction = newObject

    def setShouldDisperseSeeds(self, seedDisp):
        """
        Disperse the RNG seeds of each run in the MonteCarlo

        Args:
            seedDisp: bool
                Whether to disperse the RNG seeds in each run of the simulation
        """
        self.simParams.shouldDisperseSeeds = seedDisp

    def setExecutionCount(self, newCount):
        """
        Set the number of runs for the MonteCarlo simulation

        Args:
            newCount: int
                The number of runs to use for the simulation
        """
        self.executionCount = newCount

    def addDispersion(self, disp):
        """
        Add a dispersion to the simulation.

        Args:
            disp: Dispersion
                The dispersion to add to the simulation.
        """
        self.simParams.dispersions.append(disp)

    def addRetentionPolicy(self, policy):
        """
        Add a retention policy to the simulation.

        Args:
            disp: RetentionPolicy
                The retention policy to add to the simulation.
                This defines variables to be logged and saved
        """
        self.simParams.retentionPolicies.append(policy)

    def setThreadCount(self, threads):
        """
        Set the number of threads to use for the monte carlo simulation

        Args:
            threads: int
                Number of threads to execute the montecarlo run on.
        """
        self.numProcess = threads

    def setMaxTasksPerChild(self, count):
        """Set how many simulations each pool worker runs before replacement.

        The default is eight simulations. One restores a fresh process for
        each simulation; ``None`` keeps workers for the entire campaign.
        This setting does not affect the one-worker path, which runs in the
        calling process.

        :param int count: Positive simulation count, or ``None`` for no limit.
        :raises ValueError: The count is neither a positive integer nor ``None``.
        """
        if count is not None and (
            isinstance(count, bool) or not isinstance(count, Integral) or count < 1
        ):
            raise ValueError("Worker task limit must be a positive integer or None")
        self.maxTasksPerChild = None if count is None else int(count)

    def setRetentionQueueSize(self, count):
        """Set the maximum number of retained results awaiting the writer.

        ``None`` selects twice the active worker count. The limit counts
        simulation results, not bytes; large retained arrays still require
        corresponding memory. Producers wait when the queue is full.

        :param int count: Positive queue capacity, or ``None`` for automatic sizing.
        :raises ValueError: The capacity is neither a positive integer nor ``None``.
        """
        if count is not None and (
            isinstance(count, bool) or not isinstance(count, Integral) or count < 1
        ):
            raise ValueError("Retention queue size must be a positive integer or None")
        self.retentionQueueSize = None if count is None else int(count)

    def setVerbose(self, verbose):
        """
        Use verbose output for this MonteCarlo run

        Args:
            verbose: bool
                Whether to print verbose information during this MonteCarlo sim.
        """
        self.simParams.verbose = verbose

    def setDispMagnitudeFile(self, magnitudes):
        """
        Save .txt with the magnitude of each dispersion in % or sigma away from mean

        Args:
            magnitudes: bool
                Whether to save extra files for analysis.
        """
        self.simParams.saveDispMag = magnitudes

    def setShouldArchiveParameters(self, shouldArchiveParameters):
        self.simParams.shouldArchiveParameters = shouldArchiveParameters

    def setArchiveDir(self, dirName):
        """
        Set-up archives for this MonteCarlo run

        Args:
            dirName: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.archiveDir = os.path.abspath(dirName) + "/"
        self.simParams.shouldArchiveParameters = dirName is not None
        self.simParams.filename = self.archiveDir

    def setVarCast(self, varCast):
        """
        Set the variable type to downcast the data to

        :param varCast: 'float', 'integer', 'signed', 'unsigned' (see pandas.to_numeric documentation)
        :return:
        """
        self.varCast = varCast

    def setICDir(self, dirName):
        """
        Set-up archives containing IC data

        Args:
            dirName: string
                The name of the directory to archive runs in.
                None, if no archive desired.
        """
        self.icDirectory = os.path.abspath(dirName) + "/"
        self.simParams.shouldArchiveParameters = True
        self.simParams.icfilename = self.icDirectory

    def setICRunFlag(self, bool):
        """
        Set the number of threads to use for the monte carlo simulation

        Args:
            threads: int
                Number of threads to execute the montecarlo run on.
        """
        self.ICrunFlag = bool

    def getRetainedData(self, case):
        """
        Get the data that was retained for a run, or list of runs.

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
        """
        Get the data that was retained for a list of runs.

        Args:
            cases: int[] The desired cases to get data from.
        Returns:
            A generator is returned, which will yield, in-order, the retained data for each of these cases
        """

        for case in cases:
            yield self.getRetainedData(case)  # call this method recursively, yielding the result

    def getParameters(self, caseNumber):
        """
        Get the parameters used for a particular run of the montecarlo

        :param caseNumber: The number of the run to get the parameters used for.
        :type caseNumber: int

        :return: A dictionary of the parameters of the simulation
                 For example:
                 {"keyForSim": parameterValue, 'TaskList[0].TaskModels[0].RNGSeed': 1674764759}
        """
        if self.ICrunFlag:
            filename = self.icDirectory + "run" + str(caseNumber) + ".json"
        else:
            filename = self.archiveDir + "run" + str(caseNumber) + ".json"
        with open(filename, "r") as dispersionFile:
            dispersions = json.load(dispersionFile)
            return dispersions

    def reRunCases(self, caseList):
        """
        Rerun some cases from a MonteCarlo run. Does not run in parallel

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
                print("Rerunning", caseNumber)

            oldRunFile = self.archiveDir + "run" + str(caseNumber) + ".json"
            if not os.path.exists(oldRunFile):
                print("ERROR re-running case: " + oldRunFile)
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
            success = executor([simParams, self.dataOutQueue])

            if not success:
                print("Error re-executing run", caseNumber)
                failed.append(caseNumber)

        if len(failed) > 0:
            failed.sort()
            print("Failed rerunning cases:", failed)

        return failed

    def runInitialConditions(self, caseList):
        """
        Execute saved initial conditions with the configured worker lifecycle.

        :param caseList: Sequence of integer indices of the saved simulations to execute.
        :return: Sorted indices of failed simulations.
        :rtype: list[int]
        :raises RuntimeError: The data writer exits before finishing the archive.
        :raises ValueError: The output archive would overwrite the input archive.
        """
        assert self.icDirectory != "", "No initial condition directory was given"
        assert self.ICrunFlag is not False, "IC run flag was not set"

        if self.simParams.verbose:
            print("Beginning simulation with {0} runs on {1} threads".format(self.executionCount, self.numProcess))

        if self.simParams.shouldArchiveParameters:
            if not os.path.exists(self.icDirectory):
                print("Cannot run initial conditions: the directory given does not exist")

            if self.simParams.verbose:
                print("Archiving a copy of this simulation before running it in 'MonteCarlo.data'")
            try:
                with gzip.open(self.icDirectory + "MonteCarlo.data", "w") as pickleFile:
                    pickle.dump(self, pickleFile)  # dump this controller object into a file.
            except Exception as e:
                print("Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e)

        # Preserve the input archive when saving consolidated rerun data.
        if self.archiveDir is not None:
            if self.archiveDir == self.icDirectory:
                raise ValueError(
                    "The archive directory must differ from the initial-condition directory"
                )
            if os.path.exists(self.archiveDir):
                shutil.rmtree(self.archiveDir)
            os.mkdir(self.archiveDir)

        failed = self._execute_cases(caseList, self.generateICSims, self.archiveDir)

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.simParams.verbose:
                print("Failed", failed, "saving to 'failures.txt'")

            if self.simParams.shouldArchiveParameters:
                # write a file that contains log of failed runs
                with open(self.icDirectory + "failures.txt", "w") as failFile:
                    failFile.write(str(failed))

        return failed

    def generateICSims(self, caseList):
        """
        Generator function to clone a baseSimulation for IC run

        Args:
            baseSimulation: SimulationParams
                A base simulation to clone.
            numSims: int[]
                The desired runs to generate.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for caseNumber in caseList:
            if self.simParams.verbose:
                print("Running IC ", caseNumber)

            oldRunFile = self.icDirectory + "run" + str(caseNumber) + ".json"
            if not os.path.exists(oldRunFile):
                print("ERROR running IC case: " + oldRunFile)
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
        """
        Generator function to clone a baseSimulation

        Args:
            baseSimulation: SimulationParams
                A base simulation to clone.
            numSims: int[]
                The desired runs to generate.
        Returns:
            generator<SimulationParams>
                A generator that yields that number of cloned simulations
        """

        # make a list of simulations to execute by cloning the base-simulation and
        # changing each clone's index and filename to make a list of
        # simulations to execute
        for i in simNumList:
            simClone = copy.deepcopy(self.simParams)
            simClone.index = i
            simClone.filename += "run" + str(i)

            yield simClone

    def executeCallbacks(self, rng=None, retentionPolicies=[]):
        """
        Execute retention policy callbacks after running a monteCarlo sim.

        Args:
            rng: A list of simulations to execute callbacks on
            retentionPolicies: the retention policies to execute
        """

        if rng is None:
            rng = list(range(self.executionCount))

        if retentionPolicies == []:
            retentionPolicies = self.simParams.retentionPolicies

        for simIndex in rng:
            data = self.getRetainedData(simIndex)
            for retentionPolicy in retentionPolicies:
                retentionPolicy.executeCallback(data)

    def executeSimulations(self):
        """
        Execute simulations with a reusable worker pool and bounded retention.

        :return: Sorted indices of failed simulations.
        :rtype: list[int]
        :raises RuntimeError: The data writer exits before finishing the archive.
        """

        if self.simParams.verbose:
            print("Beginning simulation with {0} runs on {1} threads".format(self.executionCount, self.numProcess))

        if self.simParams.shouldArchiveParameters:
            if os.path.exists(self.archiveDir):
                shutil.rmtree(self.archiveDir, ignore_errors=True)
            os.mkdir(self.archiveDir)
            if self.simParams.verbose:
                print("Archiving a copy of this simulation before running it in 'MonteCarlo.data'")
            try:
                with gzip.open(self.archiveDir + "MonteCarlo.data", "wb") as pickleFile:
                    pickle.dump(self, pickleFile)  # dump this controller object into a file.
            except Exception as e:
                print("Unknown exception while trying to pickle monte-carlo-controller... \ncontinuing...\n\n", e)

        failed = self._execute_cases(
            range(self.executionCount), self.generateSims, self.archiveDir
        )

        # if there are failures
        if len(failed) > 0:
            failed.sort()

            if self.simParams.verbose:
                print("Failed", failed, "saving to 'failures.txt'")

            if self.simParams.shouldArchiveParameters:
                # write a file that contains log of failed runs
                with open(self.archiveDir + "failures.txt", "w") as failFile:
                    failFile.write(str(failed))

        return failed


class SimulationParameters():
    """
    This class represents the run parameters for a simulation, with information including

     - a function that creates the simulation
     - a function that executes the simulation
     - the dispersions to use on that simulation
     - parameters describing the data to be retained for a simulation
     - whether randomized seeds should be applied to the simulation
     - whether data should be archived
    """

    def __init__(self, creationFunction, executionFunction, configureFunction,
                 retentionPolicies, dispersions, shouldDisperseSeeds,
                 shouldArchiveParameters, filename, icfilename, index=None, verbose=False, modifications={},
                 showProgressBar=False):
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
        self.dispersionMag = {}
        self.saveDispMag = False
        self.showProgressBar = showProgressBar



class SimulationExecutor:
    """
    This class is used to execute a simulation in a worker thread.
    To use, create an instance of this class, and then call the instance with the simulation parameters to run them in::

        executor = SimulationExecutor()
        simParams = SimulationParameters()
        successFlag = executor(simParams)

    This class can be used to execute a simulation on a different thread, by using this class as the processes target.
    """
    #

    @classmethod
    def __call__(cls, params):
        """
        In each worker process, we execute this function (by calling this object)

        Args:
            params [simParams, data out queue]:
                A SimulationParameters object for the simulation to be executed and the output data queue
                for the data writer.
        Returns:
            success: bool
                (True, simParams.index) if simulation run was successful
                (False, simParams.index) if simulation run was unsuccessful
        """
        simParams = params[0]
        dataOutQueue = params[1]

        try:
            # must make new random seed on each new thread.
            np.random.seed(simParams.index * 10)
            random.seed(simParams.index * 10)

            # create the users sim by calling their supplied creationFunction
            simInstance = simParams.creationFunction()

            # build a list of the parameter and random seed modifications to make
            modifications = simParams.modifications
            magnitudes = simParams.dispersionMag

            # we may want to disperse random seeds
            if simParams.shouldDisperseSeeds:
                # generate the random seeds for the model (but don't apply them yet)
                # Note: This sets the RNGSeeds before all other modifications
                randomSeedDispersions = cls.disperseSeeds(simInstance)
                for name, value in randomSeedDispersions.items():
                    modifications[name] = value

            # used if rerunning ICs from a .json file, modifications will contain the
            # RNGSeeds that need to be set before selfInit()
            cls.populateSeeds(simInstance, modifications)

            # we may want to disperse parameters
            for disp in simParams.dispersions:
                try:
                    name = disp.getName()
                    if name not in modifications:  # could be using a saved parameter.
                        modifications[name] = disp.generateString(simInstance)
                        if simParams.saveDispMag:
                            magnitudes[name] = disp.generateMagString()
                except TypeError:
                    # This accomodates dispersion variables that are co-dependent
                    disp.generate()
                    for i in range(1, disp.numberOfSubDisps+1):
                        name = disp.getName(i)
                        if name not in modifications:  # could be using a saved parameter.
                            modifications[name] = disp.generateString(i, simInstance)
                            if simParams.saveDispMag:
                                magnitudes[name] = disp.generateMagString()

            # if archiving, this run's parameters and random seeds are saved in its own json file
            if simParams.shouldArchiveParameters:
                # save the dispersions and random seeds for this run
                if simParams.icfilename != "":
                    with open(simParams.icfilename + ".json", 'w') as outfile:
                        json.dump(modifications, outfile)
                else:
                    with open(simParams.filename + ".json", 'w') as outfile:
                        json.dump(modifications, outfile)
                    if simParams.saveDispMag:
                        with open(simParams.filename + "mag.txt", 'w') as outfileMag:
                            for k in sorted(magnitudes.keys()):
                                outfileMag.write("'%s':'%s', \n" % (k, magnitudes[k]))

            if simParams.configureFunction is not None:
                if simParams.verbose:
                    print("Configuring sim")
                simParams.configureFunction(simInstance)

            # apply the dispersions and the random seeds
            for variable, value in list(modifications.items()):
                if simParams.verbose:
                    print(f"Setting attribute {variable} to {value} on simInstance")
                cls.applyModification(simInstance, variable, value)

            # setup data logging
            if len(simParams.retentionPolicies) > 0:
                if simParams.verbose:
                    print("Adding retained data")
                RetentionPolicy.addRetentionPoliciesToSim(simInstance, simParams.retentionPolicies)

            if simParams.verbose:
                print("Executing simulation")
            # execute the simulation, with the user-supplied executionFunction
            try:
                simParams.executionFunction(simInstance)
            except TypeError:
                simParams.executionFunction(simInstance, simParams.filename)

            if len(simParams.retentionPolicies) > 0:
                if simParams.icfilename != "":
                    retentionFile = simParams.icfilename + ".data"
                else:
                    retentionFile = simParams.filename + ".data"

                if simParams.verbose:
                    print("Retaining data for run in", retentionFile)

                retainedData = RetentionPolicy.getDataForRetention(simInstance, simParams.retentionPolicies)
                # Manager.Queue.put() completes before returning; the controller
                # joins the writer before shutting down the queue manager.
                if dataOutQueue is not None:
                    dataOutQueue.put((retainedData, simParams.index, None))

                with gzip.open(retentionFile, "w") as archive:
                    retainedData["index"] = simParams.index # add run index
                    pickle.dump(retainedData, archive)

            if simParams.verbose:
                print("Terminating simulation")

            if simParams.verbose:
                print("Thread", os.getpid(), "Job", simParams.index, "finished successfully")

            return (True, simParams.index)  # this function returns true only if the simulation was successful

        except Exception as e:
            print("Error in worker thread", e)
            traceback.print_exc()
            return (False, simParams.index)  # there was an error

    @staticmethod
    def _parseAttributePath(attributeString):
        """
        Split a nested attribute path into accessors and integer indices.

        :param attributeString: Attribute path to parse.
        :type attributeString: str
        :return: Parsed attribute names, zero-argument method calls, and list indices.
        :rtype: list
        """
        if not isinstance(attributeString, str) or attributeString == "":
            raise ValueError("Attribute path must be a non-empty string")

        pathParts = []
        attributeName = ""
        indexString = ""
        parsingIndex = False

        for character in attributeString:
            if parsingIndex:
                if character == "]":
                    if indexString == "":
                        raise ValueError("Attribute path contains an empty index")
                    try:
                        pathParts.append(int(indexString))
                    except ValueError as err:
                        raise ValueError("Attribute path indices must be integers") from err
                    indexString = ""
                    parsingIndex = False
                else:
                    indexString += character
            elif character == ".":
                if attributeName != "":
                    pathParts.append(attributeName)
                    attributeName = ""
            elif character == "[":
                if attributeName != "":
                    pathParts.append(attributeName)
                    attributeName = ""
                parsingIndex = True
            else:
                attributeName += character

        if parsingIndex:
            raise ValueError("Attribute path contains an unterminated index")
        if attributeName != "":
            pathParts.append(attributeName)
        if len(pathParts) == 0:
            raise ValueError("Attribute path does not contain any parts")

        return pathParts

    @staticmethod
    def _resolvePathPart(currentObj, pathPart):
        """
        Resolve one intermediate Monte Carlo path part.

        :param currentObj: Object where this path part starts.
        :param pathPart: Attribute name, zero-argument method call, or integer index.
        :return: The object resolved by ``pathPart``.
        """
        if isinstance(pathPart, int):
            return currentObj[pathPart]

        if pathPart.endswith("()"):
            methodName = pathPart[:-2]
            if not methodName.isidentifier():
                raise ValueError(
                    "Only zero-argument method calls are supported in Monte Carlo paths"
                )
            method = getattr(currentObj, methodName)
            if not callable(method):
                raise ValueError("Monte Carlo path method part is not callable")
            return method()

        return getattr(currentObj, pathPart)

    @staticmethod
    def parseModificationValue(value):
        """
        Convert an archived Monte Carlo modification value to a Python value.

        Dispersion values are commonly stored as strings so they can be written
        to JSON.  If the value is already typed, it is returned unchanged.

        :param value: The modification value to parse.
        :return: The parsed Python value.
        """
        if not isinstance(value, str):
            return value

        try:
            return ast.literal_eval(value)
        except (SyntaxError, ValueError):
            return value

    @classmethod
    def getNestedAttr(cls, obj, attrString):
        """
        Get a nested attribute or indexed item using a Monte Carlo path string.

        :param obj: Object where path resolution starts.
        :param attrString: Attribute path to get.
        :type attrString: str
        :return: Value found at ``attrString``.
        """
        currentObj = obj
        for pathPart in cls._parseAttributePath(attrString):
            currentObj = cls._resolvePathPart(currentObj, pathPart)
        return currentObj

    @classmethod
    def _assignPathPart(cls, currentObj, pathPart, value):
        """
        Assign one final Monte Carlo path part.

        :param currentObj: Object where this path part starts.
        :param pathPart: Attribute name or integer index to assign.
        :param value: Value to assign at ``pathPart``.
        """
        if isinstance(pathPart, int):
            currentObj[pathPart] = value
        elif pathPart.endswith("()"):
            raise ValueError("Monte Carlo paths cannot assign to method calls")
        else:
            setattr(currentObj, pathPart, value)

    @classmethod
    def setNestedAttr(cls, obj, attrString, value):
        """
        Set a nested attribute or indexed item using a Monte Carlo path string.

        :param obj: Object where path resolution starts.
        :param attrString: Attribute path to set.
        :type attrString: str
        :param value: Value to set at ``attrString``.
        """
        pathParts = cls._parseAttributePath(attrString)
        finalPathPart = pathParts[-1]
        if isinstance(finalPathPart, int):
            trailingIndexStart = len(pathParts) - 1
            while (
                trailingIndexStart > 0
                and isinstance(pathParts[trailingIndexStart - 1], int)
            ):
                trailingIndexStart -= 1
            if trailingIndexStart == 0:
                raise ValueError("Indexed Monte Carlo paths require an owning attribute")

            containerOwner = obj
            for pathPart in pathParts[:trailingIndexStart - 1]:
                containerOwner = cls._resolvePathPart(containerOwner, pathPart)

            containerPathPart = pathParts[trailingIndexStart - 1]
            container = cls._resolvePathPart(containerOwner, containerPathPart)
            indexedOwners = []
            indexedObj = container
            for indexPathPart in pathParts[trailingIndexStart:-1]:
                indexedOwners.append((indexedObj, indexPathPart))
                indexedObj = cls._resolvePathPart(indexedObj, indexPathPart)

            indexedObj[finalPathPart] = value
            for indexedOwner, indexPathPart in reversed(indexedOwners):
                indexedOwner[indexPathPart] = indexedObj
                indexedObj = indexedOwner

            if not (
                isinstance(containerPathPart, str) and containerPathPart.endswith("()")
            ):
                cls._assignPathPart(containerOwner, containerPathPart, container)
        else:
            currentObj = obj
            for pathPart in pathParts[:-1]:
                currentObj = cls._resolvePathPart(currentObj, pathPart)
            cls._assignPathPart(currentObj, finalPathPart, value)

    @classmethod
    def applyModification(cls, simInstance, variable, value):
        """
        Apply one Monte Carlo modification to the simulation instance.

        :param simInstance: Simulation object to modify.
        :param variable: Nested attribute path to modify.
        :type variable: str
        :param value: Archived or typed modification value.
        """
        cls.setNestedAttr(simInstance, variable, cls.parseModificationValue(value))

    @staticmethod
    def disperseSeeds(simInstance):
        """
        Disperses the RNG seeds of all the tasks in the sim, and returns a statement that contains the seeds.
        Example return dictionary::

             {
                '.TaskList[0].TaskModels[1]': 1934586,
                '.TaskList[0].TaskModels[2]': 3450093,
                '.TaskList[1].TaskModels[0]': 2221934,
                '.TaskList[2].TaskModels[0]': 1123244
             }

        :param simInstance: A basilisk simulation to set random seeds on
        :type simInstance: SimulationBaseClass
        :return: A dictionary with the random seeds that should be applied to the sim
                        """

        randomSeeds = {}
        for i, task in enumerate(simInstance.TaskList):
            for j, model in enumerate(task.TaskModels):
                taskVar = 'TaskList[' + str(i) + '].TaskModels' + '[' + str(j) + '].RNGSeed'
                if hasattr(model, "RNGSeed"):
                    rand = str(random.randint(0, (1 << 32) - 1))
                    randomSeeds[taskVar] = rand
        return randomSeeds

    @classmethod
    def populateSeeds(cls, simInstance, modifications):
        """
        only populate the RNG seeds of all the tasks in the sim

        Args:
            simInstance: SimulationBaseClass
                A basilisk simulation to set random seeds on
            modifications:
                A dictionary containing RNGSeeds to be populate for the sim, among other sim modifications.
        """
        for variable, value in modifications.items():
            if variable.endswith(".RNGSeed"):
                cls.applyModification(simInstance, variable, value)
