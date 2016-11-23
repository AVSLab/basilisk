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
#   Simulation Setup Utilities for RW
#

import sys, os, inspect
import math
import numpy

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import SimulationBaseClass
import macros
import reactionwheel_dynamics

class rwOptions:
    useRWJitter = False
    useRWfriction = False
    useMinTorque = False
    useMaxTorque = True
    maxMomentum = 0.0
    linearFrictionRatio = -1.0          # neg. value turns off this mode by default


rwList = []
options = rwOptions()

#
#   This function is called to setup a RW device in python, and adds it to the of RW
#   devices in rwList[].  This list is accessible from the parent python script that
#   imported this rw library script, and thus any particular value can be over-ridden
#   by the user.
#
#   The first 3 arguments are required, the 4th is optional to set the RW location.  This
#   location is only used if the simple (oneway decoupled) RW jitter model is employed.
#
#   There are some RW options that can be changed.  The defaults are show in the rwOptions
#   class definition.  The use can change any particular default value prior to calling
#   the create() commands.  This new option is then applied to all the following
#   create() calls.
#
def create(
        rwType,
        gsHat_S,
        Omega,
        r_S = [0,0,0]
    ):
    global rwList
    global options

    # create the blank RW object
    RW = reactionwheel_dynamics.ReactionWheelConfigData()

    # populate the RW object with the type specific parameters
    try:
        eval(rwType + '(RW)')
        RW.typeName = rwType
    except:
        print 'ERROR: RW type ' + rwType + ' is not implemented'
        exit(1)

    # spin axis gs inertia [kg*m^2]
    RW.Js = options.maxMomentum / (RW.Omega_max)
    RW.Jt = 0.5*RW.Js
    RW.Jg = RW.Jt

    # set RW spin axis gsHat
    norm = numpy.linalg.norm(gsHat_S)
    if norm>1e-10:
        gsHat_S = gsHat_S / norm
    else:
        print 'Error: RW gsHat input must be non-zero 3x1 vector'
        exit(1)
    RW.gsHat_S = gsHat_S

    # set RW t and g unit axes
    gtHat0_S = numpy.cross(gsHat_S,[1,0,0])
    norm = numpy.linalg.norm(gtHat0_S)
    if norm < 0.01:
        gtHat0_S = numpy.cross(gsHat_S,[0,1,0])
        norm = numpy.linalg.norm(gtHat0_S)
    gtHat0_S = gtHat0_S / norm
    ggHat0_S = numpy.cross(gsHat_S,gtHat0_S)
    RW.gtHat0_S = gtHat0_S
    RW.ggHat0_S = ggHat0_S

    # set RW position vector
    RW.r_S = r_S

    # set initial RW states
    RW.Omega = Omega*macros.RPM
    RW.theta = 0.0*macros.D2R

    # enforce some RW options
    RW.usingRWJitter = options.useRWJitter
    if not options.useRWfriction:
        RW.u_f = 0.0
    if not options.useMaxTorque:
        RW.u_max = -1       # a negative value turns of RW torque saturation
    if not options.useMinTorque:
        RW.u_min = 0.0
    RW.linearFrictionRatio = options.linearFrictionRatio

    # add RW to the list of RW devices
    rwList.append(RW)

    return

#
#   This function should be called after all RW devices are created with createRW()
#   It creates the C-class container for the array of RW devices, and attaches
#   this container to the spacecraft object
#
def addToSpacecraft(modelTag, rwDynObject, VehDynObject):
    global rwList

    rwDynObject.ModelTag = modelTag
    for item in rwList:
        rwDynObject.AddReactionWheel(item)

    VehDynObject.addReactionWheelSet(rwDynObject)

    return

def clearSetup():
    global rwList
    global options

    rwList = []
    options = rwOptions()

    return


def getNumOfDevices():
    return len(rwList)

#
#   Honeywell HR16 (100Nm, 75Nm, 50Nm)
#
#   RW Information Source:
#   http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf
#
#   There are 3 momentum capacity options for this RW type.  The maximum momentum
#   capacity must be set prior to creating the HR16 RW type using
#       options.maxMomentum = 100, 75 or 50
#
def Honeywell_HR16(RW):
    global options
    # maximum allowable wheel speed
    RW.Omega_max = 6000.0*macros.RPM
    # maximum RW torque [Nm]
    RW.u_max = 0.200
    # minimum RW torque [Nm]
    RW.u_min = 0.00001
    # static friction torque [Nm]
    RW.u_f = 0.0005
    # RW rotor mass [kg]
    # Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # static RW imbalance [kg*m]
    # dynamic RW imbalance [kg*m^2]
    large = 100
    medium = 75
    small = 50
    if options.maxMomentum == large:
        RW.mass = 12.0
        RW.U_s = 4.8E-6
        RW.U_d = 15.4E-7
    elif options.maxMomentum == medium:
        RW.mass = 10.4
        RW.U_s = 3.8E-6
        RW.U_d = 11.5E-7
    elif options.maxMomentum == small:
        RW.maxx = 9.0
        RW.U_s = 2.8E-6
        RW.U_d = 7.7E-7
    else:
        if options.maxMomentum > 0:
            print 'ERROR: ' + sys._getframe().f_code.co_name + '() does not have a correct wheel momentum of '\
                  +str(large)+', '+str(medium)+' or '+str(small)+' Nm. Provided ' + str(options.maxMomentum) + ' Nm'
        else:
            print 'ERROR: ' + sys._getframe().f_code.co_name \
                  + '() maxMomentum option must be set prior to calling createRW()'
        exit(1)

    return


#
#   Honeywell HR14 (25Nm, 50Nm, 75Nm)
#
#   RW Information Source:
#   http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf
#
#   There are 3 momentum capacity options for this RW type.  The maximum momentum
#   capacity must be set prior to creating the HR14 RW type using
#       options.maxMomentum = 75, 50 or 25
#
def Honeywell_HR14(RW):
    global options
    # maximum allowable wheel speed
    RW.Omega_max = 6000.0*macros.RPM
    # maximum RW torque [Nm]
    RW.u_max = 0.200
    # minimum RW torque [Nm]
    RW.u_min = 0.00001
    # static friction torque [Nm]
    RW.u_f = 0.0005
    # RW rotor mass [kg]
    # Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # static RW imbalance [kg*m]
    # dynamic RW imbalance [kg*m^2]
    large = 75
    medium = 50
    small = 25
    if options.maxMomentum == large:
        RW.mass = 10.6
        RW.U_s = 4.8E-6
        RW.U_d = 13.7E-7
    elif options.maxMomentum == medium:
        RW.mass = 8.5
        RW.U_s = 3.5E-6
        RW.U_d = 9.1E-7
    elif options.maxMomentum == small:
        RW.maxx = 7.5
        RW.U_s = 2.2E-6
        RW.U_d = 4.6E-7
    else:
        if options.maxMomentum > 0:
            print 'ERROR: ' + sys._getframe().f_code.co_name + '() does not have a correct wheel momentum of '\
                  +str(large)+', '+str(medium)+' or '+str(small)+' Nm. Provided ' + str(options.maxMomentum) + ' Nm'
        else:
            print 'ERROR: ' + sys._getframe().f_code.co_name \
                  + '() maxMomentum option must be set prior to calling createRW()'
        exit(1)

    return


#
#   Honeywell HR12 (12Nm, 25Nm, 50Nm)
#
#   RW Information Source:
#   http://www51.honeywell.com/aero/common/documents/Constellation_Series_Reaction_Wheels.pdf
#
#   There are 3 momentum capacity options for this RW type.  The maximum momentum
#   capacity must be set prior to creating the HR12 RW type using
#       options.maxMomentum = 12, 25 or 50
#
def Honeywell_HR12(RW):
    global options
    # maximum allowable wheel speed
    RW.Omega_max = 6000.0*macros.RPM
    # maximum RW torque [Nm]
    RW.u_max = 0.200
    # minimum RW torque [Nm]
    RW.u_min = 0.00001
    # static friction torque [Nm]
    RW.u_f = 0.0005
    # RW rotor mass [kg]
    # Note: the rotor mass here is set equal to the RW mass of the above spec sheet.
    # static RW imbalance [kg*m]
    # dynamic RW imbalance [kg*m^2]
    large = 50
    medium = 25
    small = 12
    if options.maxMomentum == large:
        RW.mass = 9.5
        RW.U_s = 4.4E-6
        RW.U_d = 9.1E-7
    elif options.maxMomentum == medium:
        RW.mass = 7.0
        RW.U_s = 2.4E-6
        RW.U_d = 4.6E-7
    elif options.maxMomentum == small:
        RW.maxx = 6.0
        RW.U_s = 1.5E-6
        RW.U_d = 2.2E-7
    else:
        if options.maxMomentum > 0:
            print 'ERROR: ' + sys._getframe().f_code.co_name + '() does not have a correct wheel momentum of '\
                  +str(large)+', '+str(medium)+' or '+str(small)+' Nm. Provided ' + str(options.maxMomentum) + ' Nm'
        else:
            print 'ERROR: ' + sys._getframe().f_code.co_name \
                  + '() maxMomentum option must be set prior to calling createRW()'
        exit(1)

    return

