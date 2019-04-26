
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


#
#   Unit Test Support Script
#
import sys
import os,errno

# import Viz messaging related modules
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.simulation import spice_interface, simFswInterfaceMessages
from Basilisk.utilities import RigidBodyKinematics as rbk

sys.path.append(bskPath + '/../../../vizard/ProtoModels/modules')

youveBeenWarned = False
try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False
    if not youveBeenWarned:
        print 'Could not find vizInterface when import attempted'
        youveBeenWarned = True


def enableUnityVisualization(scSim, simTaskName, processName, fileName, bodyName = 'none'):
    if not vizFound:
        return
    home = os.path.dirname(fileName)
    if len(home)!=0:
        home +='/'
    namePath, name = os.path.split(fileName)
    if not os.path.isdir(home + '_VizFiles'):
        os.mkdir(home + '_VizFiles')
    fileName = home + '_VizFiles/' + name

    vizMessager = vizInterface.VizInterface()
    scSim.AddModelToTask(simTaskName, vizMessager)
    vizMessager.liveStream = 0
    if fileName == "":
        vizMessager.saveFile = 0
    else:
        vizMessager.saveFile = 1
    vizMessager.spiceInMsgName = vizInterface.StringVector([      "earth_planet_data",
                                                                  "mars_planet_data",
                                                                  "mars barycenter_planet_data",
                                                                  "sun_planet_data",
                                                                  "jupiter barycenter_planet_data",
                                                                  "moon_planet_data",
                                                                  "venus_planet_data",
                                                                  "mercury_planet_data",
                                                                  "uranus barycenter_planet_data",
                                                                  "neptune barycenter_planet_data",
                                                                  "pluto barycenter_planet_data",
                                                                  "saturn barycenter_planet_data"])
    vizMessager.planetNames = vizInterface.StringVector(["earth", "mars", "mars barycenter_planet_data", "sun", "jupiter barycenter", "moon", "venus", "mercury", "uranus barycenter", "neptune barycenter", "pluto barycenter", "saturn barycenter"])
    #vizMessager.numRW = 4
    vizMessager.protoFilename = fileName
    VizTaskName = "VizTask"

    if (len(bodyName) == 1):
        ephemData = spice_interface.SpicePlanetStateSimMsg()
        ephemData.J2000Current = 0.0
        ephemData.PositionVector = [0.0, 0.0, 0.0]
        ephemData.VelocityVector = [0.0, 0.0, 0.0]
        ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        ephemData.PlanetName = bodyName[0]
        msgName = bodyName[0] + '_planet_data'
        messageSize = ephemData.getStructSize()
        scSim.TotalSim.CreateNewMessage(processName, msgName, messageSize, 2, "SpicePlanetStateSimMsg")
        scSim.TotalSim.WriteMessageData(msgName, messageSize, 0, ephemData)
        # #
    if (len(bodyName) > 1):
        spiceObject = spice_interface.SpiceInterface()
        spiceObject.planetNames = spice_interface.StringVector(bodyName)
        spiceObject.ModelTag = "SpiceInterfaceData"
        spiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
        spiceObject.outputBufferCount = 100000
        spiceObject.UTCCalInit = '2018 OCT 23 04:35:25.000 (UTC)'
        scSim.AddModelToTask(simTaskName, spiceObject)

    return
