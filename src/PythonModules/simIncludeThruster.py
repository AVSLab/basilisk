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
#
#   Simulation Setup Utilities for Thruster devices
#

import sys, os, inspect
import numpy







from Basilisk.simulation import simMessages

class thrusterOptions:
    useMinPulseTime = True


thrusterList = []
options = thrusterOptions()

#
#   This function is called to setup a thruster device in python, and adds it to the of thruster
#   devices in thrusterList[].  This list is accessible from the parent python script that
#   imported this thruster library script, and thus any particular value can be over-ridden
#   by the user.
#
#   There are some thruster options that can be changed.  The defaults are show in the thrusterOptions
#   class definition.  The use can change any particular default value prior to calling
#   the createThruster() commands.  This new option is then applied to all the following
#   createThruster() calls.
#
def create(
        thrusterType,
        r_B,
        tHat_B
    ):
    global thrusterList
    global options

    # create the blank thruster object
    TH = simMessages.THRConfigSimMsg()

    # populate the thruster object with the type specific parameters
    try:
        eval(thrusterType + '(TH)')
    except:
        print 'ERROR: Thruster type ' + thrusterType + ' is not implemented'
        exit(1)

    # set thruster direction axis
    norm = numpy.linalg.norm(tHat_B)
    if norm>1e-10:
        tHat_B = tHat_B / norm
    else:
        print 'Error: Thruster ' + sys._getframe().f_code.co_name +' direction tHat input must be non-zero 3x1 vector'
        exit(1)
    TH.thrDir_B = [[tHat_B[0]], [tHat_B[1]], [tHat_B[2]]]

    # set thruster position vector
    TH.thrLoc_B = [[r_B[0]], [r_B[1]], [r_B[2]]]
    # enforce Thruster options
    if not options.useMinPulseTime:
        TH.MinOnTime = 0.0

    # add thruster to the list of thruster devices
    thrusterList.append(TH)

    return

#
#   This function should be called after all thruster devices are created with createThruster()
#   It creates the C-class container for the array of thruster devices, and attaches
#   this container to the spacecraft object
#
def addToSpacecraft(modelTag, thDynamicEffector, scPlus, fuelTankEffector):
    global thrusterList

    thDynamicEffector.ModelTag = modelTag

    thDynamicEffector.thrusterData = simMessages.ThrusterConfigVector(thrusterList)

    fuelTankEffector.addThrusterSet(thDynamicEffector)

    scPlus.addDynamicEffector(thDynamicEffector)

    return

def clearSetup():
    global thrusterList
    global options

    thrusterList = []
    options = thrusterOptions()

    return

def getNumOfDevices():
    return len(thrusterList)


#
#   MOOG Monarc-1
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_1(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 0.9
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 227.5

    return

#
#   MOOG Monarc-5
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_5(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 4.5
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 226.1

    return

#
#   MOOG Monarc-22-6
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_22_6(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 22.0
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 229.5

    return

#
#   MOOG Monarc-22-12
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_22_12(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 22.0
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 228.1

    return

#
#   MOOG Monarc-90LT
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_90LT(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 90.0
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 232.1

    return

#
#   MOOG Monarc-90HT
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_90HT(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 116.0
    # minimum thruster on time [s]
    TH.MinOnTime = 0.010
    # Isp value [s]
    TH.steadyIsp = 234.0

    return

#
#   MOOG Monarc-445
#
#   Information Source:
#   http://www.moog.com/literature/Space_Defense/Spacecraft/Propulsion/Monopropellant_Thrusters_Rev_0613.pdf
#
#   This is a MOOG mono-propellant thruster
#
def MOOG_Monarc_445(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 445.0
    # minimum thruster on time [s]
    TH.MinOnTime = 0.025
    # Isp value [s]
    TH.steadyIsp = 234.0

    return

def TEST_Thruster(TH):
    global options
    # maximum thrust [N]
    TH.MaxThrust = 0.9
    # minimum thruster on time [s]
    TH.MinOnTime = 0.020
    # Isp value [s]
    TH.steadyIsp = 227.5
    # nozzle area [m^2]
    TH.areaNozzle = 0.07

    return
