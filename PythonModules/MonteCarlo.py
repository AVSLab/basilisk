''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import sys, os, inspect  # Don't worry about this, standard stuff plus file discovery
import random
import abc
import numpy as np
import shutil
import imp
from multiprocessing import Pool, cpu_count
import cPickle as pickle
import copy
import gzip
import signal
import json

random.seed(0x1badcad1)
np.random.seed(0x1badcad1)


class SingleVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, varName, bounds):
        self.varName = varName
        self.bounds = bounds

    @abc.abstractmethod
    def generate(self, sim):
        pass

    def checkBounds(self, value):
        if self.bounds is None:
            return value

        if value <= self.bounds[0]:
            value = self.bounds[0]
        if value >= self.bounds[1]:
            value = self.bounds[1]
        return value

    def getName(self):
        return self.varName

    def generateString(self):
        return str(self.generate())

class UniformDispersion(SingleVariableDispersion):
    def __init__(self, varName, bounds=None):
        SingleVariableDispersion.__init__(self, varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        dispValue = random.uniform(self.bounds[0], self.bounds[1])
        return dispValue


class NormalDispersion(SingleVariableDispersion):
    def __init__(self, varName, mean=0.0, stdDeviation=0.5, bounds=None):
        SingleVariableDispersion.__init__(self, varName, bounds)
        self.mean = mean
        self.stdDeviation = stdDeviation

    def generate(self, sim):
        dispValue = random.gauss(self.mean, self.stdDeviation)
        if self.bounds is not None:
            dispValue = self.checkBounds(dispValue)
        return dispValue


class VectorVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, varName, bounds):
        self.varName = varName
        self.bounds = bounds
        return

    @abc.abstractmethod
    def generate(self, sim=None):
        pass

    def perturbVectorByAngle(self, vector, angle):
        rndVec = np.random.random(3)
        if np.dot(rndVec, vector) > 0.95:
            rndVec[0] *= -1
        eigenAxis = np.cross(vector, rndVec)
        thrusterMisalignDCM = self.eigAxisAndAngleToDCM(eigenAxis, angle)
        return np.dot(thrusterMisalignDCM, vector)

    def perturbCartesianVectorUniform(self, vector):
        dispValues = np.zeros(3)
        for i in range(len(vector)):
            dispValues[i] = random.uniform(self.bounds[0], self.bounds[1])
        return dispValues

    def perturbCartesianVectorNormal(self, vector):
        dispValues = np.zeros(3)
        for i in range(len(vector)):
            dispValues[i] = random.gauss(self.mean, self.stdDeviation)
        return dispValues

    @staticmethod
    def eigAxisAndAngleToDCM(axis, angle):
        axis = axis / np.linalg.norm(axis)
        sigma = 1 - np.cos(angle)
        dcm = np.zeros((3, 3))
        dcm[0, 0] = axis[0] ** 2 * sigma + np.cos(angle)
        dcm[0, 1] = axis[0] * axis[1] * sigma + axis[2] * np.sin(angle)
        dcm[0, 2] = axis[0] * axis[2] * sigma - axis[1] * np.sin(angle)
        dcm[1, 0] = axis[1] * axis[0] * sigma - axis[2] * np.sin(angle)
        dcm[1, 1] = axis[1] ** 2 * sigma + np.cos(angle)
        dcm[1, 2] = axis[1] * axis[2] * sigma + axis[0] * np.sin(angle)
        dcm[2, 0] = axis[2] * axis[0] * sigma + axis[1] * np.sin(angle)
        dcm[2, 1] = axis[2] * axis[1] * sigma - axis[0] * np.sin(angle)
        dcm[2, 2] = axis[2] ** 2 * sigma + np.cos(angle)
        return dcm

    # @TODO This should be a @classmethod.
    @staticmethod
    def checkBounds(value, bounds):
        if value < bounds[0]:
            value = bounds[0]
        if value > bounds[1]:
            value = bounds[1]
        return value

    def generateString(self):
        # TODO does this actually behave differently then str(nextValue)?
        nextValue = self.generate(simInstance)
        val = '['
        for i in range(3):
            val += str(nextValue[i]) + ','
        val = val[0:-1] + ']'

        return val


class UniformVectorDispersion(VectorVariableDispersion):
    def __init__(self, varName, bounds=None):
        VectorVariableDispersion.__init__(self, varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        vector = eval('sim.' + self.varName)
        dispValue = self.perturbCartesianVectorUniform(vector)
        return dispValue

class NormalVectorDispersion(VectorVariableDispersion):
    def __init__(self, varName, mean=0.0, stdDeviation=0.5, bounds=None):
        VectorVariableDispersion.__init__(self, varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        vector = eval('sim.' + self.varName)
        dispValue = self.perturbCartesianVectorNormal(vector,self.mean,self.stdDeviation)
        return dispValue


class UniformVectorAngleDispersion(VectorVariableDispersion):
    def __init__(self, varName, phiBounds=None, thetaBounds=None):
        super(UniformVectorAngleDispersion, self).__init__(varName, None)
        # @TODO these bounds are not currently being applied to the generated values
        self.phiBounds = phiBounds
        if phiBounds is None:
            self.phiBounds = ([0.0, 2 * np.pi])
        self.thetaBounds = thetaBounds
        if thetaBounds is None:
            self.thetaBounds = self.phiBounds

    def generate(self, sim=None):
        vector = eval('sim.' + self.varName)
        phiRnd = random.uniform(self.phiBounds[0], self.phiBounds[1])
        thetaRnd = random.uniform(self.thetaBounds[0], self.thetaBounds[1])
        dispVec = np.array(vector) + np.array([[np.sin(phiRnd) * np.cos(thetaRnd)],
                    [np.sin(phiRnd) * np.sin(thetaRnd)],
                    [phiRnd]])
        dispVec = dispVec/np.linalg.norm(dispVec)
        return dispVec


class UniformEulerAngleMRPDispersion(VectorVariableDispersion):
    def __init__(self, varName, bounds=None):
        """
        Args:
            varName (str): A string representation of the variable to be dispersed
                e.g. 'VehDynObject.AttitudeInit'.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values radians.
        """
        super(UniformEulerAngleMRPDispersion, self).__init__(varName, bounds)
        if self.bounds is None:
            self.bounds = ([0, 2 * np.pi])

    def generate(self, sim=None):
        rndAngles = np.zeros((3, 1))
        for i in range(3):
            rndAngles[i] = (self.bounds[1] - self.bounds[0]) * np.random.random() + self.bounds[0]
        dispMRP = rbk.euler3232MRP(rndAngles)
        dispMRP = dispMRP.reshape(3)
        return dispMRP


class NormalThrusterUnitDirectionVectorDispersion(VectorVariableDispersion):
    def __init__(self, varName, thrusterIndex=0, phiStd=0.1745, bounds=None):
        """
        Args:
            varName (str): A string representation of the variable to be dispersed
                e.g. 'ACSThrusterDynObject.ThrusterData[0].thrusterDirectionDisp'.
            thrusterIndex (int): The index of the thruster to be used in array references.
            phiStd (float): The 1 sigma standard deviation of the dispersion angle in radians.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values.
        """
        super(NormalThrusterUnitDirectionVectorDispersion, self).__init__(varName, bounds)
        self.varNameComponents = self.varName.split(".")
        self.phiStd = phiStd  # (rad) angular standard deviation
        # Limit dispersion to a hemisphere around the vector being dispersed
        # if self.bounds is None:
        #     self.bounds = ([-np.pi/2, np.pi/2])
        self.thrusterIndex = thrusterIndex


    def getName(self):
        return '.'.join(self.varNameComponents[0:-1]) + '.thrDir_B'

    def generateString(self):
        # TODO does this actually behave differently then str(nextValue)?
        nextValue = self.generate(simInstance)

        val = '['
        for i in range(3):
            val += str(nextValue[i])
            if (i < 2):
                val += ', '
        val += ']'

        return val


    def generate(self, sim=None):
        if sim is None:
            print("No simulation object parameter set in '" + self.generate.__name__ + "()'"
                                                                                       " dispersions will not be set for variable " + self.varName)
            return
        else:
            separator = '.'
            thrusterObject = getattr(sim, self.varNameComponents[0])
            totalVar = separator.join(self.varNameComponents[0:-1])
            dirVec = eval('sim.' + totalVar + '.thrDir_B')
            angle = np.random.normal(0, self.phiStd, 1)
            dirVec = np.array(dirVec).reshape(3).tolist()
            dispVec = self.perturbVectorByAngle(dirVec, angle)
        return dispVec


class UniformVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, varName, bounds=None):
        super(UniformVectorCartDispersion, self).__init__(varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])

    def generate(self, sim=None):
        dispVec = []
        for i in range(3):
            rnd = random.uniform(self.bounds[0], self.bounds[1])
            rnd = self.checkBounds(rnd, self.bounds)
            dispVec.append(rnd)
        return dispVec


class NormalVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, varName, mean=0.0, stdDeviation=0.0, bounds=None):
        super(NormalVectorCartDispersion, self).__init__(varName, bounds)
        self.mean = mean
        self.stdDeviation = stdDeviation

    def generate(self, sim=None):
        dispVec = []
        for i in range(3):
            if isinstance(self.stdDeviation, collections.Sequence):
                rnd = random.gauss(self.mean[i], self.stdDeviation[i])
            else:
                rnd = random.gauss(self.mean, self.stdDeviation)
            if self.bounds is not None:
                rnd = self.checkBounds(rnd, self.bounds)
            dispVec.append(rnd)
        return dispVec


class InertiaTensorDispersion:
    def __init__(self, varName, stdDiag=None, boundsDiag=None, stdAngle=None):
        """
        Args:
            varName (str): A string representation of the variable to be dispersed
                e.g. 'LocalConfigData.I'.
            stdDeviation (float): The 1 sigma standard deviation of the diagonal element dispersions in kg*m^2.
            bounds (Array[float, float]): defines lower and upper cut offs for generated dispersion values kg*m^2.
        """
        self.varName = varName
        self.varNameComponents = self.varName.split(".")
        self.stdDiag = stdDiag
        self.stdAngle = stdAngle
        self.bounds = boundsDiag
        if self.stdDiag is None:
            self.stdDiag = 1.0
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])
        if self.stdAngle is None:
            self.stdAngle = 0.0

    def generate(self, sim=None):
        if sim is None:
            print("No simulation object parameter set in '" + self.generate.__name__ + "()'"
                                                                                       " dispersions will not be set for variable " + self.varName)
            return
        else:
            vehDynObject = getattr(sim, self.varNameComponents[0])
            I = np.array(eval('sim.' + self.varName)).reshape(3, 3)

            # generate random values for the diagonals
            temp = []
            for i in range(3):
                rnd = random.gauss(0, self.stdDiag)
                rnd = self.checkBounds(rnd)
                temp.append(rnd)
            dispIdentityMatrix = np.identity(3) * temp
            # generate random values for the similarity transform to produce off-diagonal terms
            angles = np.random.normal(0, self.stdAngle, 3)
            disp321Matrix = rbk.euler3212C(angles)

            # disperse the diagonal elements
            dispI = I + dispIdentityMatrix
            # disperse the off diagonals with a slight similarity transform of the inertia tensor
            dispI = np.dot(np.dot(disp321Matrix, dispI), disp321Matrix.T)

        return dispI

    def checkBounds(self, value):
        if value < self.bounds[0]:
            value = self.bounds[0]
        if value > self.bounds[1]:
            value = self.bounds[1]
        return value


    def generateString(self):
        nextValue = self.generate(simInstance)
        # TODO does this actually behave differently then str(nextValue)?
        val = '['
        for i in range(3):
            val += '[' + str(nextValue[i][0]) + ', ' \
                              + str(nextValue[i][1]) + ', ' \
                              + str(nextValue[i][2]) + ']'
            if i is not 2:
                val += ','
        val = val[0:] + ']'
        return val


class MonteCarloController:

    def __init__(self):
        self.executionCount = 0
        self.numProcess = cpu_count()
        self.dispersions = []
        self.simParams = SimulationParameters(
            creationFunction=None,
            executionFunction=None,
            retentionParameters=None,
            shouldArchive=False,
            shouldDisperseSeeds=False,
            dispersions=None,
            filename=""
        )

    @staticmethod
    def load(runDirectory):
        """ Load a previously completed MonteCarlo simulation
        Args:
            The path to the MonteCarlo.data file that contains the archived MonteCarlo run
        """
        filename = os.path.abspath(runDirectory) + "/MonteCarlo.data"
        print "Loading montecarlo at", filename

        with gzip.open(filename) as pickledData:
            data = pickle.load(pickledData)
            return data

    def setRetentionParameters(self, retentionParameters):
        """ Set an execution function that chooses what data to retain from a simulation instance.
        Args:
            retainFunction: (sim: SimulationBaseClass) => any
                A function with one parameter, a simulation instance.
                The function must return all data the user wishes to retain from the simulation execution.
                The function will be called after the executionFunction in each simulation run.
                It must extract infomation from the simulation instance, and return it as a picklable python object.
        """
        self.simParams.retentionParameters = retentionParameters

    def setExecutionModule(self, newModule):
        """ Set an execution function that executes a simulation instance.
        Args:
            executionFunction: (sim: SimulationBaseClass) => None
                A function with one parameter, a simulation instance.
                The function will be called after the creationFunction and configurationFunction in each simulation run.
                It must execute the simulation.
                Its return value is not used.
        """
        self.simParams.executionFunction = newModule

    def setSimulationObject(self, newObject):
        """ Set the function that creates the simulation instance.
        Args:
            creationFunction: () => SimulationBaseClass
                A function with no parameters, that returns a simulation instance.
        """
        self.simParams.creationFunction = newObject

    def setDisperseSeeds(self, seedDisp):
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

    def addNewDispersion(self, disp):
        """ Add a dispersion to the simulation.
        Args:
            disp: Dispersion
                The dispersion to add to the simulation.
        """
        self.dispersions.append(disp)

    def setDisperseSeeds(self, seedDisp):
        """ Disperse the RNG seeds of each run in the MonteCarlo
        Args:
            seedDisp: bool
                Whether to disperse the RNG seeds in each run of the simulation
        """
        self.simParams.shouldDisperseSeeds = seedDisp

    def setThreadCount(self, threads):
        self.numProcess = threads

    def setVerbose(self, verbose):
        """ Use verbose output for this MonteCarlo run
        Args:
            verbose: bool
                Whether to print verbose information during this MonteCarlo sim.
        """
        self.simParams.verbose = verbose

    def archiveICs(self, dirName):
        """ Set-up archives for this MonteCarlo run
        Args:
            dirName: string
                The name of the directory to archive runs in.
        """
        self.archiveDir = os.path.abspath(dirName) + "/"
        self.simParams.shouldArchive = True
        self.simParams.filename = self.archiveDir

    def getRetainedData(self, case):
        """ Get the data that was retained for a run, or list of runs.
        Args:
            cases: int The desired case to get data from.
        Returns:
            The retained data for that run is returned.
        """

        oldRunDataFile = self.archiveDir + "run" + str(case) + ".data"
        with gzip.open(oldRunDataFile) as pickledData:
            data = pickle.load(pickledData)
            return data

    def getRetainedDatas(self, cases):
        """ Get the data that was retained for a run, or list of runs.
        Args:
            cases: int[] The desired cases to get data from.
        Returns:
            A generator is returned, which will yield, in-order, the retained data for each of these cases
        """

        for case in cases:
            yield self.getRetainedData(case) # call this method recursively, yielding the result

    def reRunCases(self, caseList):
        """ Rerun some cases from a MonteCarlo run. Does not run in parallel
        Args:
            caseList: int[]
                The list of runs to repeat, a list of numbers.
        Returns:
            failures: int[]
                The list of failed runs.
        """

        for caseNumber in caseList:
            print "Rerunning", caseNumber

            oldRunFile = self.archiveDir + "run" + str(caseNumber) + ".json"
            if not os.path.exists(oldRunFile):
                print "ERROR re-running case: " + oldRunFile
                continue

            # use same simulation parameters but don't archive
            simParams = copy.deepcopy(self.simParams)
            simParams.shouldArchive = False
            simParams.index = caseNumber
            # don't redisperse seeds, we want to use the ones saved in the oldRunFile
            simParams.shouldDisperseSeeds = False
            with open(oldRunFile, "r") as runParameters:
                simParams.dispersions = json.load(runParameters)
                print "rerunning with ", simParams.dispersions

            #execute simulation with dispersion
            executor = SimulationExecutor()
            success = executor(simParams)

            if not success:
                print "Error re-executing run", caseNumber

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
            simClone.dispersions = {}

            # generate the desired parameters
            for disp in self.dispersions:
                # for each dispersion in the MonteCarlo, generate parameters for each simulation run
                variable = disp.getName()
                value = disp.generateString(disp)
                simClone.dispersions[variable] = value

            yield simClone

    def executeSimulations(self):
        ''' Execute simulations in parallel
        Args: None
        Returns:
            failed: int[]
                A list of the indices of all failed simulation runs.
        '''

        print "Beginning simulation with {0} runs on {1} threads".format(self.executionCount, self.numProcess)

        if self.simParams.shouldArchive:
            if os.path.exists(self.archiveDir):
                shutil.rmtree(self.archiveDir)
            os.mkdir(self.archiveDir)
            if self.simParams.verbose:
                print "Archiving a copy of this simulation before running it in 'MonteCarlo.data'"
            with gzip.open(self.archiveDir + "MonteCarlo.data", "w") as pickleFile:
                pickle.dump(self, pickleFile)

        numSims = self.executionCount

        # Avoid building a full list of all simulations to run in memory,
        # instead only generating simulations right before they are needed by a waiting worker
        # This is accomplished using a generator and pool.imap, -- simulations are only built
        # when they are about to be passed to a worker, avoiding memory overhead of first building simulations
        # There is a chunking behavior, something like 10-20 are generated at a time.
        pool = Pool(self.numProcess)
        simGenerator = self.generateSims(range(numSims))

        # The simulation executor is responsible for executing simulation given a simulation's parameters
        # It is called within worker threads with each worker's simulation parameters
        simulationExecutor = SimulationExecutor()

        failed = []  # keep track of the indices of failed simulations
        jobsFinished = 0  # keep track of what simulations have finished

        try:
            for result in pool.imap(simulationExecutor, simGenerator): # yields results *as* the workers finish jobs
                if result != True:  # workers return True on success
                    failed.append(jobsFinished) # add failed jobs to the list of failures
                jobsFinished += 1

                if jobsFinished % max(1, numSims / 20) == 0:  # print percentage after every ~5%
                    print "Finished", jobsFinished, "/", numSims, \
                          "\t-- {}%".format(int(100 * float(jobsFinished) / numSims))
            pool.close()
        except KeyboardInterrupt:
            print "Ctrl-C was hit, closing pool"
            pool.terminate()
        except Exception as e:
            print "Unknown exception while running simulations:", e
            pool.terminate()
        finally:
            failed.extend(range(jobsFinished + 1, numSims)) # fail all jobs after the last finished one
            pool.join()

        # if there are failures
        if len(failed) > 0:
            print "Failed", failed, "saving to 'failures.txt'"
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
    '''
    def __init__(self, creationFunction, executionFunction, retentionParameters, dispersions, shouldDisperseSeeds, shouldArchive, filename, index=None, verbose=False):
        self.index = index
        self.creationFunction = creationFunction
        self.executionFunction = executionFunction
        self.retentionParameters = retentionParameters
        self.dispersions = dispersions
        self.shouldDisperseSeeds = shouldDisperseSeeds
        self.shouldArchive = shouldArchive
        self.filename = filename
        self.verbose = verbose


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
                True if simulation run was successful
        '''
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN) # On ctrl-c ignore the signal... let the parent deal with it.

            # create the users sim by calling their supplied creationFunction
            simInstance = simParams.creationFunction()

            # we may also want to randomize the randomSeeds for each of the runs of the MonteCarlo
            if simParams.shouldDisperseSeeds:
                # generate the random seeds for the model (but don't apply them yet)
                randomSeedDispersions = cls.disperseSeeds(simInstance)
                for name, value in randomSeedDispersions.items():
                    simParams.dispersions[name] = value

            # if archiving, this run's parameters and random seeds are saved in its own json file
            if simParams.shouldArchive:
                # save the dispersions and random seeds for this run
                with open(simParams.filename + ".json", 'w') as outfile:
                    json.dump(simParams.dispersions, outfile)

            # apply the dispersions and the random seeds
            for variable, value in simParams.dispersions.items():
                evalStatement = "simInstance" + variable + "=" + value
                exec evalStatement

            # execute the simulation, with the user-supplied executionFunction
            simParams.executionFunction(simInstance)

            if simParams.retentionParameters != None:
                retentionFile = simParams.filename + ".data"
                #if simParams.verbose:
                #    print "Retaining data for run in", retentionFile
                with gzip.open(retentionFile, "w") as archive:
                    retainedData = cls.getDataForRetention(simInstance, simParams.retentionParameters)
                    pickle.dump(retainedData, archive)

            # terminate the simulation
            simInstance.terminateSimulation()

            if simParams.verbose:
                print "Thread", os.getpid(), "Job", simParams.index, "finished successfully"

            return True  # this function returns true only if the simulation was successful

        except Exception as e:
            print "Error in worker thread", e
            return False  # there was an error

    @staticmethod
    def disperseSeeds(simInstance):
        """  disperses the RNG seeds of all the tasks in the model, and returns a statement that contains the seeds
        Args:
            simInstance: SimulationBaseClass
                A basilisk simulation to set random seeds on
        Returns:
            statement: string
                A dictionary with the random seeds that should be applied:
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
        i = 0
        for Task in simInstance.TaskList:
            j = 0
            for model in Task.TaskModels:
                taskVar = '.TaskList[' + str(i) + '].TaskModels' + '[' + str(j) + '].RNGSeed'
                try:
                    randomSeeds[taskVar] = str(random.randint(0, 1 << 32 - 1))
                except ValueError:
                    randomSeeds.pop(taskVar, None) # remove it if it failed
                    j += 1
                    continue
                j += 1
            i += 1
        return randomSeeds

    @staticmethod
    def getDataForRetention(simInstance, retentionParameters):
        """ Returns the data that should be retained given a simInstance and the retentionParameters
        Args:
            simInstance: The simulation instance to retrive data from
            retentionParameters: A dictionary that states what data to retrieve from the simInstance
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
        if "messages" in retentionParameters:
            data["messages"] = {}
            for (message, dataType) in retentionParameters["messages"].items():
                data["messages"][message] = simInstance.pullMessageLogData(message, dataType)

        if "variables" in retentionParameters:
            data["variables"] = {}
            for variable, dataType in retentionParameters["variables"].items():
                data["variables"][variable] = simInstance.pullMessageLogData("inertial_state_output.v_BN_N", range(3))

        return data
