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
#   Simulation Setup Utilities for External Force and Torque Disturbance values
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
import ExternalForceTorque

extForceTorqueList = []

#
#   This function is called to setup an external force/torque disturbancein python, and adds it to the
#   list of disturbances in extForceTorqueList[].  This list is accessible from the parent python script that
#   imported this rw library script, and thus any particular value can be over-ridden
#   by the user.
#
#
def createExtForceTorque(
        force_B,
        torque_B,
    ):
    global rwList

    # create the blank RW object
    extFTObject = ExternalForceTorque.ExternalForceTorque()

    extFTObject.force_B = force_B
    extFTObject.torque_B = torque_B

    # add RW to the list of RW devices
    extForceTorqueList.append(extFTObject)

    return

#
#   This function should be called after all external disturbances are created with createRW()
#   It creates the C-class container for the array of RW devices, and attaches
#   this container to the spacecraft object
#
def addExtForceTorqueToSpacecraft(modelTag, rwDynObject, VehDynObject):
    global extForceTorqueList

    rwDynObject.ModelTag = modelTag
    rwDynObject.inputVehProps="spacecraft_mass_props"
    for item in extForceTorqueList:
        rwDynObject.AddReactionWheel(item)
        VehDynObject.addBodyEffector(extForceTorqueList(item))

    return

def clearExtForceTorqueSetup():
    global rwList

    rwList = []

    return
