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

import random
import numpy as np
import abc
from Basilisk.utilities import RigidBodyKinematics as rbk
import collections


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

    def generateString(self, sim):
        return str(self.generate(sim))


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

    def generateString(self, sim):
        # TODO does this actually behave differently then str(nextValue)?
        nextValue = self.generate(sim)
        val = '['
        for i in range(3):
            val += str(nextValue[i]) + ','
        val = val[0:-1] + ']'
        return val

    def getName(self):
        return self.varName


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
        dispValue = self.perturbCartesianVectorNormal(vector, self.mean, self.stdDeviation)
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
        dispVec = dispVec / np.linalg.norm(dispVec)
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

    def generateString(self, sim):
        # TODO does this actually behave differently then str(nextValue)?
        nextValue = self.generate(sim)

        val = '['
        for i in range(3):
            val += str(nextValue[i])
            if (i < 2):
                val += ', '
        val += ']'

        return val

    def generate(self, sim=None):
        if sim is None:
            print("No simulation object parameter set in '" + self.generate.__name__
                  + "()' dispersions will not be set for variable " + self.varName)
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
            print("No simulation object parameter set in '" + self.generate.__name__
                  + "()' dispersions will not be set for variable " + self.varName)
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

    def generateString(self, sim):
        nextValue = self.generate(sim)
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

    def getName(self):
        return self.varName
