
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



import abc
import collections
import random

import numpy as np
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import orbitalMotion


class SingleVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, varName, bounds):
        self.varName = varName
        self.bounds = bounds
        self.magnitude = []

    @abc.abstractmethod
    def generate(self, sim):
        pass

    def getDispersionMag(self):
        return self.magnitude

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

    def generateMagString(self):
        return str(self.getDispersionMag())


class UniformDispersion(SingleVariableDispersion):
    def __init__(self, varName, bounds=None):
        SingleVariableDispersion.__init__(self, varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        dispValue = random.uniform(self.bounds[0], self.bounds[1])

        mid = (self.bounds[1] + self.bounds[0])/2.
        scale = self.bounds[1] - mid
        self.magnitude.append(str(round((dispValue - mid)/scale*100,2)) + " %")
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
        if self.stdDeviation !=0 :
            self.magnitude.append(str(round((dispValue - self.mean)/self.stdDeviation,2)) + " sigma")
        return dispValue


class VectorVariableDispersion(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, varName, bounds):
        self.varName = varName
        self.bounds = bounds
        self.magnitude = []
        return

    @abc.abstractmethod
    def generate(self, sim=None):
        pass

    def getDispersionMag(self):
        return self.magnitude

    def perturbVectorByAngle(self, vector, angle):
        rndVec = np.random.random(3)
        if np.dot(rndVec, vector) > 0.95:
            rndVec[0] *= -1
        eigenAxis = np.cross(vector, rndVec)
        thrusterMisalignDCM = self.eigAxisAndAngleToDCM(eigenAxis, angle)
        self.magnitude.append(str(round(np.arccos(np.dot(rndVec, vector)/np.linalg.norm(rndVec)/np.linalg.norm(vector))*180./np.pi,2)) + " deg")
        return np.dot(thrusterMisalignDCM, vector)

    def perturbCartesianVectorUniform(self, vector):
        dispValues = np.zeros(3)
        for i in range(len(vector)):
            dispValues[i] = random.uniform(self.bounds[0], self.bounds[1])
            mid = (self.bounds[1] + self.bounds[0])
            scale = self.bounds[1] - mid
            self.magnitude.append(str(round((dispValues[i] - mid)/scale*100,2)) + " %")
        return dispValues

    def perturbCartesianVectorNormal(self, vector):
        dispValues = np.zeros(3)
        for i in range(len(vector)):
            dispValues[i] = random.gauss(self.mean, self.stdDeviation)
            if self.stdDeviation != 0 :
                self.magnitude.append(str(round((dispValues[i] - self.mean)/self.stdDeviation,2)) + r" $\sigma$")
        return dispValues

    def cart2Spherical(self, cartVector):
        # Spherical Coordinate Set: [rho, theta, phi]
        x = cartVector[0]
        y = cartVector[1]
        z = cartVector[2]

        rho = np.linalg.norm(cartVector)
        phi = np.arctan2(y, x)[0]
        theta = np.arccos(z)[0]

        return [rho, phi, theta]

    def spherical2Cart(self, spherVec):
        rho = spherVec[0]
        phi = spherVec[1]
        theta = spherVec[2]

        x = rho * np.sin(theta) * np.cos(phi)
        y = rho * np.sin(theta) * np.sin(phi)
        z = rho * np.cos(theta)

        return [x,y,z]

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

    def generateMagString(self):
        nextValue = self.getDispersionMag()
        val = '['
        for i in range(len(self.magnitude)):
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
        vector = getattr(sim, self.varName)
        dispValue = self.perturbCartesianVectorUniform(vector)
        return dispValue


class NormalVectorDispersion(VectorVariableDispersion):
    def __init__(self, varName, mean=0.0, stdDeviation=0.5, bounds=None):
        VectorVariableDispersion.__init__(self, varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])  # defines a hard floor/ceiling

    def generate(self, sim):
        vector = getattr(sim, self.varName)
        dispValue = self.perturbCartesianVectorNormal(vector, self.mean, self.stdDeviation)
        return dispValue


class UniformVectorAngleDispersion(VectorVariableDispersion):
    def __init__(self, varName, phiBoundsOffNom=None, thetaBoundsOffNom=None):
        super(UniformVectorAngleDispersion, self).__init__(varName, None)
        # @TODO these bounds are not currently being applied to the generated values

        self.phiBoundsOffNom = phiBoundsOffNom
        self.thetaBoundsOffNom = thetaBoundsOffNom

        if phiBoundsOffNom is None:
            self.phiBoundsOffNom = [-np.pi / 2, np.pi / 2]
        if thetaBoundsOffNom is None:
            self.thetaBoundsOffNom = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        # Note this dispersion is applied off of the nominal
        vectorCart = getattr(sim, self.varName)
        vectorCart = vectorCart/np.linalg.norm(vectorCart)
        vectorSphere = self.cart2Spherical(vectorCart)

        meanPhi = vectorSphere[1] # Nominal phi
        meanTheta = vectorSphere[2] #Nominal theta

        self.phiBounds = [meanPhi + self.phiBoundsOffNom[0], meanPhi + self.phiBoundsOffNom[1]]
        self.thetaBounds = [meanTheta + self.thetaBoundsOffNom[0],  meanTheta + self.thetaBoundsOffNom[1]]

        phiRnd = np.random.uniform(meanPhi+self.phiBounds[0], meanPhi+self.phiBounds[1])
        thetaRnd = np.random.uniform(meanTheta+self.thetaBounds[0], meanTheta+self.thetaBounds[1])

        phiRnd = self.checkBounds(phiRnd, self.phiBounds)
        thetaRnd = self.checkBounds(thetaRnd, self.thetaBounds)

        newVec = self.spherical2Cart([1.0, phiRnd, thetaRnd])
        dispVec = newVec/np.linalg.norm(newVec) # Shouldn't technically need the normalization but doing it for completeness

        midPhi = (self.phiBounds[1] + self.phiBounds[0])/2.
        scalePhi = self.phiBounds[1] - midPhi
        midTheta = (self.thetaBounds[1] + self.thetaBounds[0])/2.
        scaleTheta = self.thetaBounds[1] - midTheta
        self.magnitude.append(str(round((phiRnd - midPhi)/scalePhi*100,2)) + " %")
        self.magnitude.append(str(round((thetaRnd - midTheta)/scaleTheta*100,2)) + " %")

        return dispVec


class NormalVectorAngleDispersion(VectorVariableDispersion):
    def __init__(self, varName, thetaStd = np.pi/3.0, phiStd=np.pi/3.0, thetaBoundsOffNom=None, phiBoundsOffNom=None):
        super(NormalVectorAngleDispersion, self).__init__(varName, None)
        # @TODO these bounds are not currently being applied to the generated values

        self.thetaStd = thetaStd
        self.phiStd = phiStd

        self.phiBoundsOffNom = phiBoundsOffNom
        self.thetaBoundsOffNom = thetaBoundsOffNom

        if phiBoundsOffNom is None:
            self.phiBoundsOffNom = [-np.pi/2, np.pi/2]
        if thetaBoundsOffNom is None:
            self.thetaBoundsOffNom = [-np.pi, np.pi]

        self.magnitude = []

    def generate(self, sim=None):
        vectorCart = getattr(sim, self.varName)
        vectorCart = vectorCart/np.linalg.norm(vectorCart)
        vectorSphere = self.cart2Spherical(vectorCart)

        meanPhi = vectorSphere[1] # Nominal phi
        meanTheta = vectorSphere[2] # Nominal theta

        phiRnd = np.random.normal(meanPhi, self.phiStd, 1)
        thetaRnd = np.random.normal(meanTheta, self.thetaStd, 1)

        self.phiBounds = [meanPhi + self.phiBoundsOffNom[0], meanPhi + self.phiBoundsOffNom[1]]
        self.thetaBounds = [meanTheta + self.thetaBoundsOffNom[0],  meanTheta + self.thetaBoundsOffNom[1]]

        phiRnd = self.checkBounds(phiRnd, self.phiBounds)
        thetaRnd = self.checkBounds(thetaRnd, self.thetaBounds)

        newVec = self.spherical2Cart([1.0, phiRnd, thetaRnd])
        dispVec = newVec/np.linalg.norm(newVec) # Shouldn't technically need the normalization but doing it for completeness

        self.magnitude.append(str(round((phiRnd - meanPhi) / self.phiStd, 2)) + r" $\sigma$")
        self.magnitude.append(str(round((thetaRnd - meanTheta) / self.thetaStd, 2)) + r" $\sigma$")

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
        self.magnitude = []

    def generate(self, sim=None):
        rndAngles = np.zeros((3, 1))
        for i in range(3):
            rndAngles[i] = (self.bounds[1] - self.bounds[0]) * np.random.random() + self.bounds[0]
        dispMRP = rbk.euler3232MRP(rndAngles)
        dispMRP = dispMRP.reshape(3)
        for i in range(3):
            self.magnitude.append(str(round((dispMRP[i] - np.pi)/np.pi*100,2))+ " %")
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
        self.magnitude = []

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

    def generateMagString(self):
        nextValue = self.getDispersionMag()

        val = '['
        for i in range(len(self.magnitude)):
            val += str(nextValue[i])
            val += ', '
        val += ']'

        return val

    def generate(self, sim=None):
        if sim is None:
            print(("No simulation object parameter set in '" + self.generate.__name__
                  + "()' dispersions will not be set for variable " + self.varName))
            return
        else:
            separator = '.'
            thrusterObject = getattr(sim, self.varNameComponents[0])
            totalVar = separator.join(self.varNameComponents[0:-1])
            simObject = getattr(sim, totalVar)  # sim.totalVar
            dirVec = getattr(simObject, 'thrDir_B')  # sim.totalVar.thrDir_B
            angle = np.random.normal(0, self.phiStd, 1)
            dirVec = np.array(dirVec).reshape(3).tolist()
            dispVec = self.perturbVectorByAngle(dirVec, angle)
            angleDisp = np.arccos(np.dot(dirVec, dispVec)/np.linalg.norm(dirVec)/np.linalg.norm(dispVec))
            self.magnitude.append(str(round(angleDisp / self.phiStd,2)) + " sigma")
        return dispVec


class UniformVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, varName, bounds=None):
        super(UniformVectorCartDispersion, self).__init__(varName, bounds)
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])
        self.magnitude = []

    def generate(self, sim=None):
        dispVec = []
        for i in range(3):
            rnd = random.uniform(self.bounds[0], self.bounds[1])
            rnd = self.checkBounds(rnd, self.bounds)
            for i in range(3):
                mid = (self.bounds[1] + self.bounds[0])/2.
                scale = self.bounds[1] - mid
                self.magnitude.append(str(round((rnd - mid) / scale * 100,2))+ " %")
            dispVec.append(rnd)
        return dispVec


class NormalVectorCartDispersion(VectorVariableDispersion):
    def __init__(self, varName, mean=0.0, stdDeviation=0.0, bounds=None):
        super(NormalVectorCartDispersion, self).__init__(varName, bounds)
        self.mean = mean
        self.stdDeviation = stdDeviation
        self.magnitude = []

    def generate(self, sim=None):
        dispVec = []
        for i in range(3):
            if isinstance(self.stdDeviation, collections.abc.Sequence):
                rnd = random.gauss(self.mean[i], self.stdDeviation[i])
                if self.stdDeviation[i] != 0:
                    self.magnitude.append(str(round((rnd - self.mean[i])/self.stdDeviation[i],2)) + " sigma")
            else:
                rnd = random.gauss(self.mean, self.stdDeviation)
                if self.stdDeviation != 0:
                    self.magnitude.append(str(round((rnd - self.mean) / self.stdDeviation,2)) + " sigma")
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
        self.magnitude = []
        if self.stdDiag is None:
            self.stdDiag = 1.0
        if self.bounds is None:
            self.bounds = ([-1.0, 1.0])
        if self.stdAngle is None:
            self.stdAngle = 0.0

    def generate(self, sim=None):
        if sim is None:
            print(("No simulation object parameter set in '" + self.generate.__name__
                  + "()' dispersions will not be set for variable " + self.varName))
            return
        else:
            vehDynObject = getattr(sim, self.varNameComponents[0])
            parts = self.varName.split('.')
            attr = sim
            for part in parts:
                attr = getattr(attr, part)
            I = np.array(attr).reshape(3, 3)

            # generate random values for the diagonals
            temp = []
            for i in range(3):
                rnd = random.gauss(0, self.stdDiag)
                rnd = self.checkBounds(rnd)
                temp.append(rnd)
                if self.stdDiag != 0:
                    self.magnitude.append(str(round(rnd/self.stdDiag,2)) + " sigma")
            dispIdentityMatrix = np.identity(3) * temp
            # generate random values for the similarity transform to produce off-diagonal terms
            angles = np.random.normal(0, self.stdAngle, 3)
            for i in range(3):
                if self.stdAngle != 0:
                    self.magnitude.append(str(round(angles[i] / self.stdAngle,2)) + " sigma")
            disp321Matrix = rbk.euler3212C(angles)

            # disperse the diagonal elements
            dispI = I + dispIdentityMatrix
            # disperse the off diagonals with a slight similarity transform of the inertia tensor
            dispI = np.dot(np.dot(disp321Matrix, dispI), disp321Matrix.T)

        return dispI

    def getDispersionMag(self):
        return self.magnitude

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
            if i != 2:
                val += ','
        val = val[0:] + ']'
        return val

    def generateMagString(self):
        nextValue = self.getDispersionMag()
        val = '['
        for i in range(len(self.magnitude)):
            val += str(nextValue[i]) + ','
        val = val[0:-1] + ']'
        return val

    def getName(self):
        return self.varName


class OrbitalElementDispersion:
    def __init__(self, varName1, varName2, dispDict):
        """
        A function that disperses position and velocity of the spacecraft using orbital elements as a dispersion metric.
        Args:
            varName1 (str): A string representation of the position variable to be dispersed
            varName2 (str): A string representation of the velocity variable to be dispersed
            dispDict (dict): A dictionnary containing the dispersions for each of the orbital elements. The values are lists
            with first element 'normal' or 'uniform' followed by mean, std or lower bound, upper bound respectively. If no dispersion
            is added for a specific orbital elemenet, None should be the values for the corresponding key
        """
        self.numberOfSubDisps = 2
        self.varName1 = varName1
        self.varName1Components = self.varName1.split(".")
        self.varName2 = varName2
        self.varName2Components = self.varName2.split(".")
        self.oeDict = dispDict


    def generate(self, sim=None):
        elems = orbitalMotion.ClassicElements
        for key in self.oeDict.keys():
            if self.oeDict[key] is not None and key != "mu":
                distribution = getattr(np.random, self.oeDict[key][0])
                random_value = distribution(self.oeDict[key][1], self.oeDict[key][2])
                setattr(elems, key, random_value)
            else:
                if key != "mu":
                    setattr(elems, key, 0.0)
        if elems.e < 0:
            elems.e = 0
        r, v =orbitalMotion.elem2rv_parab( self.oeDict["mu"], elems)

        self.dispR = r
        self.dispV = v


    def generateString(self, index, sim=None):
        if index == 1:
            nextValue = self.dispR
        if index == 2:
            nextValue = self.dispV
        val = '['
        for i in range(3):
            val += str(nextValue[i]) + ','
        val = val[0:-1] + ']'
        return val

    def getName(self, index):
        if index == 1:
            return self.varName1
        if index == 2:
            return self.varName2

class MRPDispersionPerAxis(VectorVariableDispersion):
    def __init__(self, varName, bounds=None):
        """
        A function that disperses MRPs with specfic bounds per axis.
        Args:
            varName (str): A string representation of the variable to be dispersed
                e.g. 'VehDynObject.AttitudeInit'.
            bounds (list(Array[float, float],Array[float, float],Array[float, float])): defines lower and upper cut offs for generated dispersion values radians.
        """
        super(MRPDispersionPerAxis, self).__init__(varName, bounds)
        if self.bounds is None:
            self.bounds = [[0, 2 * np.pi], [0, 2 * np.pi], [0, 2 * np.pi]]

    def generate(self, sim=None):
        rndAngles = np.zeros((3, 1))
        for i in range(3):
            rndAngles[i] = (self.bounds[i][1] - self.bounds[i][0]) * np.random.random() + self.bounds[i][0]
        dispMRP = rndAngles.reshape(3)
        return dispMRP
