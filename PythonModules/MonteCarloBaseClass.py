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
import SimulationBaseClass
import random
import abc
import numpy as np
import RigidBodyKinematics as rbk
import shutil
import imp

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
        return np.dot(thrusterMisalignDCM,vector)

    @staticmethod
    def eigAxisAndAngleToDCM(axis, angle):
        axis = axis/np.linalg.norm(axis)
        sigma = 1 - np.cos(angle)
        dcm = np.zeros((3, 3))
        dcm[0,0] = axis[0]**2 * sigma + np.cos(angle)
        dcm[0,1] = axis[0] * axis[1] * sigma + axis[2] * np.sin(angle)
        dcm[0,2] = axis[0] * axis[2] * sigma - axis[1] * np.sin(angle)
        dcm[1,0] = axis[1] * axis[0] * sigma - axis[2] * np.sin(angle)
        dcm[1,1] = axis[1]**2 * sigma + np.cos(angle)
        dcm[1,2] = axis[1] * axis[2] * sigma + axis[0] * np.sin(angle)
        dcm[2,0] = axis[2] * axis[0] * sigma + axis[1] * np.sin(angle)
        dcm[2,1] = axis[2] * axis[1] * sigma - axis[0] * np.sin(angle)
        dcm[2,2] = axis[2]**2 * sigma + np.cos(angle)
        return dcm

    # @TODO This should be a @classmethod.
    @staticmethod
    def checkBounds(value, bounds):
        if value < bounds[0]:
            value = bounds[0]
        if value > bounds[1]:
            value = bounds[1]
        return value


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
        phiRnd = random.uniform(self.phiBounds[0], self.phiBounds[1])
        thetaRnd = random.uniform(self.thetaBounds[0], self.thetaBounds[1])
        dispVec = ([np.sin(phiRnd) * np.cos(thetaRnd),
                    np.sin(phiRnd) * np.sin(thetaRnd),
                    phiRnd])
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
            self.bounds = ([0, 2*np.pi])

    def generate(self, sim=None):
        rndAngles = np.zeros((3,1))
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

    def generate(self, sim=None):
        if sim is None:
            print("No simulation object parameter set in '" + self.generate.__name__ + "()'"
                  " dispersions will not be set for variable " + self.varName)
            return
        else:
            separator = '.'
            thrusterObject = getattr(sim, self.varNameComponents[0])
            totalVar = separator.join(self.varNameComponents[0:-1])
            dirVec = eval('sim.' + totalVar + '.inputThrDir_S')
            angle = np.random.normal(0, self.phiStd, 1)
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
            I = np.array(eval('sim.'+self.varName)).reshape(3, 3)

            # generate random values for the diagonals
            temp = []
            for i in range(3):
                rnd = random.gauss(0, self.stdDiag)
                rnd = self.checkBounds(rnd)
                temp.append(rnd)
            dispIdentityMatrix = np.identity(3)*temp
            # generate random values for the similarity transform to produce off-diagonal terms
            angles = np.random.normal(0, self.stdAngle, 3)
            disp321Matrix = rbk.euler3212C(angles)

            # disperse the diagonal elements
            dispI = I + dispIdentityMatrix
            # disperse the off diagonals with a slight similarity transform of the inertia tensor
            dispI = np.dot(np.dot(disp321Matrix, dispI), disp321Matrix.T)

            # return as a single row shape so it's easier for the executeSimulation runner to read
        return dispI.reshape(9)

    def checkBounds(self, value):
        if value < self.bounds[0]:
            value = self.bounds[0]
        if value > self.bounds[1]:
            value = self.bounds[1]
        return value


class MonteCarloBaseClass:
    def __init__(self):
        self.simList = []
        self.varDisp = []
        self.randomizeSeeds = False
        self.executionModule = None
        self.simulationObject = None
        self.configureModule = None
        self.executionCount = 0
        self.retainSimulationData = False
        self.disperseSeeds = False
        self.archiveSettings = False

    def setRandomizeSeeds(self, seedSet):
        self.randomizeSeeds = seedSet

    def setExecutionModule(self, newModule):
        self.executionModule = newModule
    
    def setConfigureModule(self, newModule):
        self.configureModule = newModule

    def setSimulationObject(self, newObject):
        self.simulationObject = newObject

    def setExecutionCount(self, newCount):
        self.executionCount = newCount

    def setRetainSimulationData(self, retainData):
        self.retainSimulationData = retainData

    def addNewDispersion(self, disp):
        self.varDisp.append(disp)
    def setDisperseSeeds(self, seedDisp):
        self.disperseSeeds = seedDisp
    def archiveICs(self, dirName):
        self.archiveDir = dirName + '_MonteCarloICs'
        self.archiveSettings = True
        
    def reRunCases(self, caseList):
        previousSimulation = None
        for caseNumber in caseList:
            if not os.path.exists(self.archiveDir + "/Run" +str(caseNumber) + ".py"):
                print "ERROR re-running case: " + self.archiveDir + "/Run" + \
                    str(caseNumber) + ".py"
                continue
            if previousSimulation is not None:
                previousSimulation.terminateSimulation()
        
            newSim = self.simulationObject()
            if self.configureModule is not None:
                self.configureModule(newSim)
            updateModule = imp.load_source("disperseVariables",
                self.archiveDir + "/Run" +str(caseNumber) + ".py")
            updateModule.disperseVariables(newSim)
            self.executionModule(newSim)
            if self.retainSimulationData:
                self.simList.append(newSim)
                
            previousSimulation = newSim
            print caseNumber


    def executeSimulations(self):
        simRunCounter = 0
        previousSimulation = None

        if(self.archiveSettings):
            if(os.path.exists(self.archiveDir)):
                shutil.rmtree(self.archiveDir)
            os.mkdir(self.archiveDir)
        while simRunCounter < self.executionCount:
            fHandle = None
            if previousSimulation is not None:
                previousSimulation.terminateSimulation()
            if(self.archiveSettings):
                fHandle = open(self.archiveDir + '/Run' + str(simRunCounter) + '.py', 'w')
            newSim = self.simulationObject()
            
            if self.configureModule is not None:
                self.configureModule(newSim)

            execString = "def disperseVariables(newSim): \n"
            if fHandle is not None:
                fHandle.write(execString + '\n')
            for disp in self.varDisp:
                nextValue = disp.generate(newSim)
                if isinstance(disp, NormalThrusterUnitDirectionVectorDispersion):
                    separator = '.'
                    execString = 'newSim.' + separator.join(disp.varNameComponents[0:-1]) + '.inputThrDir_S = ['
                    for i in range(3):
                        execString += str(nextValue[i])
                        if(i<2):
                            execString += ', '
                    execString += ']'
                    exec(execString)
                    if fHandle is not None:
                        fHandle.write('    ' + execString + '\n')
                elif isinstance(disp, InertiaTensorDispersion):
                    for i in range(9):
                        execString = 'newSim.' + disp.varName + '[' + str(i) + '] = ' + str(nextValue[i])
                        exec(execString)
                        if fHandle is not None:
                            fHandle.write('    ' + execString + '\n')
                elif isinstance(disp, VectorVariableDispersion):
                    for i in range(3):
                        execString = 'newSim.' + disp.varName + '[' + str(i) + '] = ' + str(nextValue[i])
                        exec(execString)
                        if fHandle is not None:
                            fHandle.write('    ' + execString + '\n')
                else:
                    execString = 'newSim.' + disp.varName + ' = ' + str(nextValue)
                    exec(execString)
                    if fHandle is not None:
                        fHandle.write('    ' + execString + '\n')

            if self.disperseSeeds == True:
                i=0
                for Task in newSim.TaskList:
                    j=0;
                    for model in Task.TaskModels:
                        execString = 'newSim.TaskList[' + str(i) + '].TaskModels'
                        execString += '[' + str(j) + '].RNGSeed = '
                        try:
                            model.RNGSeed = random.randint(0, 1<<32-1)
                            execString += str(model.RNGSeed)
                        except ValueError:
                            execString = []
                            j += 1
                            continue
                        if fHandle is not None:
                            fHandle.write('    ' + execString + '\n')
                        j+=1
                    i+=1
            if fHandle is not None:
               fHandle.close()

            self.executionModule(newSim)

            if self.retainSimulationData:
                self.simList.append(newSim)

            previousSimulation = newSim
            simRunCounter += 1
            print simRunCounter
