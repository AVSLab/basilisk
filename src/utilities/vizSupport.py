
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
import sys, os
from Basilisk.utilities import unitTestSupport
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.simulation import spice_interface

sys.path.append(bskPath + '/../../../vizard/ProtoModels/modules')

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False


def enableUnityVisualization(scSim, simTaskName, processName, **kwargs):
    if not vizFound:
        print 'Could not find vizInterface when import attempted.  Be sure to build BSK with vizInterface support.'
        return

    # setup the Vizard interface module
    vizMessenger = vizInterface.VizInterface()
    scSim.AddModelToTask(simTaskName, vizMessenger)

    # set spacecraft name
    vizMessenger.spacecraftName = "bsk-Sat"

    # note that the following logic can receive a single file name, or a full path + file name.
    # In both cases a local results are stored in a local sub-folder.
    vizMessenger.saveFile = 0
    if kwargs.has_key('saveFile'):
        fileNamePath = kwargs['saveFile']
        fileName = os.path.splitext(os.path.basename(fileNamePath))[0]
        filePath = os.path.dirname(fileNamePath)
        if filePath == "":
            filePath = "."
        if not os.path.isdir(filePath + '/_VizFiles'):
            os.mkdir(filePath + '/_VizFiles')
        vizFileNamePath = filePath + '/_VizFiles/' + fileName + '_UnityViz.bin'
        vizMessenger.saveFile = 1
        vizMessenger.protoFilename = vizFileNamePath
        print "Saving Viz file to " + vizFileNamePath

    vizMessenger.opNavMode = 0
    if kwargs.has_key('opNavMode'):
        if kwargs['opNavMode'] == True:
            vizMessenger.opNavMode = 1
            vizMessenger.opnavImageOutMsgName = "opnav_circles"

    vizMessenger.spiceInMsgName = vizInterface.StringVector([      "earth_planet_data",
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
    vizMessenger.planetNames = vizInterface.StringVector(["earth", "mars", "mars barycenter", "sun", "jupiter barycenter", "moon", "venus", "mercury", "uranus barycenter", "neptune barycenter", "pluto barycenter", "saturn barycenter"])


    # see if celestial body planet ephemeris messages must be created
    if (kwargs.has_key('gravBodies')):
        gravFactory = kwargs['gravBodies']
        gravBodies = gravFactory.gravBodies
        if (gravBodies):
            for key in gravBodies:
                msgName = key + '_planet_data'
                if (not scSim.TotalSim.IsMsgCreated(msgName)):
                    ephemData = spice_interface.SpicePlanetStateSimMsg()
                    ephemData.J2000Current = 0.0
                    ephemData.PositionVector = [0.0, 0.0, 0.0]
                    ephemData.VelocityVector = [0.0, 0.0, 0.0]
                    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
                    ephemData.PlanetName = key
                    # setting the msg structure name is required below to all the planet msg to be logged
                    unitTestSupport.setMessage(scSim.TotalSim, processName, msgName,
                                               ephemData, "SpicePlanetStateSimMsg")


    if (kwargs.has_key('numRW')):
        vizMessenger.numRW = kwargs['numRW']

    return vizMessenger
