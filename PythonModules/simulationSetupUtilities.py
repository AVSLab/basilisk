'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#   Simulation Setup Utilities
#

import sys, os, inspect
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import SimulationBaseClass
import reactionwheel_dynamics



#
#   add a Reaction Wheel (RW) device to the simulation
#
numRW = 0
def addReactionWheelToSimulation(
        rwDynObject,
        VehDynObject,
        gsHat_S,
        rwType='Honeywell_HR16'):
    global numRW

    if numRW == 0:
        # initialize RW states
        rwDynObject.ModelTag = "ReactionWheels"
        VehDynObject.addReactionWheelSet(rwDynObject)

    numRW += 1

    # create the blank RW object
    RW = reactionwheel_dynamics.ReactionWheelConfigData()

    # populate the RW object with the type specific parameters
    try:
        eval(rwType + '(RW, gsHat_S)')
    except:
        print 'ERROR: RW type ' + rwType + ' is not implemented'
        exit(1)

    # add RW to the list of RW objects
    rwDynObject.AddReactionWheel(RW)

    return

def Honeywell_HR16(RW, gsHat_S):
    # spin axis gs inertia [kg*m^2]
    RW.Js = 100.0 / (6000.0 / 60.0 * math.pi * 2.0)
    # maximum RW torque [Nm]
    RW.u_max = 0.200
    # minimum RW torque [Nm]
    RW.u_min = 0.00001
    # static friction torque [Nm]
    RW.u_f = 0.0005
    # static RW imbalance [kg*m], based on rough industry reference
    RW.U_s = 7.0E-6
    # dynamic RW imbalance [kg*m^2]
    RW.U_d = 20.0E-7
    # set RW spin axis gsHat
    SimulationBaseClass.SetCArray(gsHat_S, 'double', RW.gsHat_S)

    return