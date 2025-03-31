#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Simulation Setup Utilities for Thruster devices
#

import sys
from collections import OrderedDict

import numpy
from Basilisk.architecture import messaging
from Basilisk.simulation import thrusterDynamicEffector


class thrusterFactory(object):
    """Simulation Thruster Factory Class"""

    def __init__(self):
        self.useMinPulseTime = True
        self.thrusterList = OrderedDict()

    def create(self, thrusterType, r_B, tHat_B, **kwargs):
        """
        This function is called to setup a thruster device in python, and adds it to the of thruster
        factory in ``thrusterList{}``.  The function returns a copy of the device that can be changed if needed.
        The first 3 arguments are required, the remaining arguments are optional with:

        Parameters
        ----------
        thrusterType : string
                thruster manufacturing name.:
        r_B : list
                vector with thruster location in B-frame components:
        tHat_B : list
                vector with thruster force direction unit vector:
        kwargs:
            useMinPulseTime: BOOL
                flag if the thruster model should use a minimum impulse time
            areaNozzle: float
                thruster nozzle exhaust cone exit area
            steadyIsp: float
                thruster fuel efficiency in Isp (seconds)
            MaxThrust: float
                maximum thruster force in Newtons
            thrusterMagDisp: float
                thruster dispersion percentage
            MinOnTime: float
                thruster minimum on time
            cutoffFrequency: float
                frequency of first-order filter dynamics
            swirlTorque: float
                constant momentum from ionic thrusters
            thrBlowDownCoeff: list
                vector with polynomial coefficients for fuel mass to thrust blow down model in descending order
            ispBlowDownCoeff: list
                vector with polynomial coefficients for fuel mass to Isp blow down model in descending order

        """
        # create the blank thruster object
        TH = thrusterDynamicEffector.THRSimConfig()

        # set default thruster values
        TH.areaNozzle = 0.1  # [m^2]
        TH.steadyIsp = 100  # [s]
        TH.MaxThrust = 0.200  # [N]
        TH.thrusterMagDisp = 0.0  # [%]
        TH.MinOnTime = 0.020  # [s]
        TH.cutoffFrequency = 10.0  # [rad/s]
        TH.MaxSwirlTorque = 0.0  # [Nm]

        # populate the thruster object with the type specific parameters
        try:
            getattr(self, thrusterType)(TH)
        except:
            print('ERROR: Thruster type ' + thrusterType + ' is not implemented')
            exit(1)

        # set device states from the input arguments.  Note that these may override what is set in
        # the above function call
        if 'areaNozzle' in kwargs:
            varAreaNozzle = kwargs['areaNozzle']
            if not isinstance(varAreaNozzle, (float)):
                print('ERROR: areaNozzle must be a float argument')
                exit(1)
            else:
                TH.areaNozzle = varAreaNozzle

        if 'steadyIsp' in kwargs:
            varSteadyIsp = kwargs['steadyIsp']
            if not isinstance(varSteadyIsp, (float)):
                print('ERROR: steadyIsp must be a float argument')
                exit(1)
            else:
                TH.steadyIsp = varSteadyIsp

        if 'MaxThrust' in kwargs:
            varMaxThrust = kwargs['MaxThrust']
            if not isinstance(varMaxThrust, (float)):
                print('ERROR: MaxThrust must be a float argument')
                exit(1)
            else:
                TH.MaxThrust = varMaxThrust

        if 'MaxSwirlTorque' in kwargs:
            varMaxSwirlTorque = kwargs['MaxSwirlTorque']
            if not isinstance(varMaxSwirlTorque, (float)):
                print('ERROR: MaxSwirlTorque must be a float argument')
                exit(1)
            else:
                TH.MaxSwirlTorque = varMaxSwirlTorque

        if 'thrusterMagDisp' in kwargs:
            varThrusterMagDisp = kwargs['thrusterMagDisp']
            if not isinstance(varMaxThrust, (float)):
                print('ERROR: varThrusterMagDisp must be a float argument')
                exit(1)
            else:
                TH.thrusterMagDisp = varThrusterMagDisp

        if 'MinOnTime' in kwargs:
            varMinOnTime = kwargs['MinOnTime']
            if not isinstance(varMinOnTime, (float)):
                print('ERROR: MinOnTime must be a float argument')
                exit(1)
            else:
                TH.MinOnTime = varMinOnTime

        if 'cutoffFrequency' in kwargs:
            varCutoffFrequency = kwargs['cutoffFrequency']
            if not isinstance(varCutoffFrequency, (float)):
                print('ERROR: cutoffFrequency must be a float argument')
                exit(1)
            else:
                TH.cutoffFrequency = varCutoffFrequency

        if 'useMinPulseTime' in kwargs:
            varUseMinPulseTime = kwargs['useMinPulseTime']
            if not isinstance(varUseMinPulseTime, (bool)):
                print('ERROR: useMinPulseTime must be a BOOL argument')
                exit(1)
        else:
            varUseMinPulseTime = False  # default value
        if not varUseMinPulseTime:
            TH.MinOnTime = 0.0

        if 'label' in kwargs:
            varLabel = kwargs['label']
            if not isinstance(varLabel, (str)):
                print('ERROR: TH label must be a string')
                exit(1)
            if len(varLabel) > 5:
                print('ERROR: TH label string is longer than 5 characters')
                exit(1)
        else:
            varLabel = 'TH' + str(len(self.thrusterList) + 1)  # default device labeling
        TH.label = varLabel

        if 'thrBlowDownCoeff' in kwargs:
            thrBlowDownCoeff = kwargs['thrBlowDownCoeff']
            if not isinstance(thrBlowDownCoeff, list):
                print('ERROR: thruster blow down coefficients must be a numerical list')
                exit(1)
            else:
                for coeff in thrBlowDownCoeff: TH.thrBlowDownCoeff.push_back(coeff)

        if 'ispBlowDownCoeff' in kwargs:
            ispBlowDownCoeff = kwargs['ispBlowDownCoeff']
            if not isinstance(ispBlowDownCoeff, list):
                print('ERROR: Isp blow down coefficients must be a numerical list')
                exit(1)
            else:
                for coeff in ispBlowDownCoeff: TH.ispBlowDownCoeff.push_back(coeff)

        # set thruster force direction axis
        norm = numpy.linalg.norm(tHat_B)
        if norm > 1e-10:
            tHat_B = tHat_B / norm
        else:
            print(
                'Error: Thruster ' + sys._getframe().f_code.co_name + ' direction tHat input must be non-zero 3x1 vector')
            exit(1)
        TH.thrDir_B = [[tHat_B[0]], [tHat_B[1]], [tHat_B[2]]]

        # set thruster position vector
        TH.thrLoc_B = [[r_B[0]], [r_B[1]], [r_B[2]]]

        # add TH to the list of TH devices
        self.thrusterList[varLabel] = TH
        return TH

    def addToSpacecraft(self, modelTag, thEffector, sc):
        """
            This function should be called after all Thruster devices are created with create()
            It creates the C-class container for the array of TH devices, and attaches
            this container to the spacecraft object

            Parameters
            ----------
            modelTag:  string
                module model tag string
            thEffector: thrusterEffector
                thruster effector handle
            sc: spacecraft
        """

        thEffector.ModelTag = modelTag

        for key, th in list(self.thrusterList.items()):
            thEffector.addThruster(th)

        # Check the type of thruster effector
        thrusterType = str(type(thEffector))
        if 'ThrusterDynamicEffector' in thrusterType:
            sc.addDynamicEffector(thEffector)
        elif 'ThrusterStateEffector' in thrusterType:
            sc.addStateEffector(thEffector)
        else:
            print("This isn't a thruster effector. You did something wrong.")

        return

    def getNumOfDevices(self):
        """
            Returns the number of RW devices setup.

        :return: number of thruster devices

        """
        return len(self.thrusterList)

    def getConfigMessage(self):
        """
            Returns a FSW THRArrayConfigMsg reflecting the current thruster setup.

        :return: thrMessage
        """

        thrMessage = messaging.THRArrayConfigMsgPayload()

        i = 0
        for simThruster in self.thrusterList.values():
            #   Converts from THRConfigSimMsg to THRConfigFswMsg
            fswThruster = messaging.THRConfigMsgPayload()
            fswThruster.maxThrust = simThruster.MaxThrust
            fswThruster.rThrust_B = [val for sublist in simThruster.thrLoc_B for val in sublist]
            fswThruster.tHatThrust_B = [val for sublist in simThruster.thrDir_B for val in sublist]
            messaging.ThrustConfigArray_setitem(thrMessage.thrusters, i, fswThruster)
            i += 1

        thrMessage.numThrusters = len(self.thrusterList.values())

        thrConfigMsg = messaging.THRArrayConfigMsg().write(thrMessage)

        return thrConfigMsg

    #
    #   MOOG Monarc-1
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_1(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 0.9
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 227.5

        TH.areaNozzle = 0.000079  # [m^2]

        return

    #
    #   MOOG Monarc-5
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_5(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 4.5
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 226.1

        TH.areaNozzle = 0.0020  # [m^2]

        return

    #
    #   MOOG Monarc-22-6
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_22_6(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 22.0
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 229.5

        TH.areaNozzle = 0.0045  # [m^2]

        return

    #
    #   MOOG Monarc-22-12
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_22_12(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 22.0
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 228.1

        TH.areaNozzle = 0.0088  # [m^2]

        return

    #
    #   MOOG Monarc-90LT
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_90LT(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 90.0
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 232.1

        TH.areaNozzle = 0.0222  # [m^2]

        return

    #
    #   MOOG Monarc-90HT
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_90HT(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 116.0
        # minimum thruster on time [s]
        TH.MinOnTime = 0.010
        # Isp value [s]
        TH.steadyIsp = 234.0

        TH.areaNozzle = 0.0222  # [m^2]

        return

    #
    #   MOOG Monarc-445
    #
    #   Information Source:
    #   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
    #   http://www.moog.com/content/dam/moog/literature/Space_Defense/Spacecraft/Monopropellant_Thrusters_Rev_0613.pdf
    #
    #   This is a MOOG mono-propellant thruster
    #
    def MOOG_Monarc_445(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 445.0
        # minimum thruster on time [s]
        TH.MinOnTime = 0.025
        # Isp value [s]
        TH.steadyIsp = 234.0

        TH.areaNozzle = 0.06881  # [m^2]

        return

    def SEP(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 0.030
        # minimum thruster on time [s]
        TH.MinOnTime = 0.025
        # Isp value [s]
        TH.steadyIsp = 3000.0
        # maximum swirl torque [Nm]
        TH.MaxSwirlTorque = 0.05

        TH.areaNozzle = 0.06881  # [m^2]

        return

    def TEST_Thruster(self, TH):
        # maximum thrust [N]
        TH.MaxThrust = 0.9
        # minimum thruster on time [s]
        TH.MinOnTime = 0.020
        # Isp value [s]
        TH.steadyIsp = 227.5
        # nozzle area [m^2]
        TH.areaNozzle = 0.07

        return

    def Blank_Thruster(self, TH):
        # this method doesn't set any thruster properties.  Rather, it is assumed that all thruster
        # properties are defined explicitly in the create function, or external to the create function

        return
