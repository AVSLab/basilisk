
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

from Basilisk.utilities import unitTestSupport

# import Viz messaging related modules
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.simulation import spice_interface

sys.path.append(bskPath + '/../../../vizard/ProtoModels/modules')

youveBeenWarned = False
try:
    import vizInterface
    vizFound = True
except ImportError:
    vizFound = False
    if not youveBeenWarned:
        print 'Could not find vizInterface when import attempted'
        youveBeenWarned = True


def enableUnityVisualization(scSim, simTaskName, processName, fileName, gravFactory = None):
    if not vizFound:
        return
    vizMessager = vizInterface.VizInterface()
    scSim.AddModelToTask(simTaskName, vizMessager)

    vizMessager.spiceInMsgName = vizInterface.StringVector([
                                                                  "earth_planet_data",
                                                                  "mars_planet_data",
                                                                  "sun_planet_data",
                                                                  "jupiter barycenter_planet_data",
                                                                  "moon_planet_data",
                                                                  "venus_planet_data",
                                                                  "mercury_planet_data",
                                                                  "uranus barycenter_planet_data",
                                                                  "neptune barycenter_planet_data",
                                                                  "pluto barycenter_planet_data",
                                                                  "saturn barycenter_planet_data"])
    vizMessager.planetNames = vizInterface.StringVector(["earth", "mars", "sun", "jupiter barycenter", "moon", "venus", "mercury", "uranus barycenter", "neptune barycenter", "pluto barycenter", "saturn barycenter"])
    #vizMessager.numRW = 4
    vizMessager.protoFilename = fileName
    VizTaskName = "VizTask"

    if (gravFactory != None):
        gravBodies = gravFactory.gravBodies
        if (gravBodies):
            for key in gravBodies:
                if gravBodies[key].isCentralBody == True:
                    ephemData = spice_interface.SpicePlanetStateSimMsg()
                    ephemData.J2000Current = 0.0
                    ephemData.PositionVector = [0.0, 0.0, 0.0]
                    ephemData.VelocityVector = [0.0, 0.0, 0.0]
                    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
                    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
                    ephemData.PlanetName = key
                    msgName = key + '_planet_data'
                    unitTestSupport.setMessage(scSim.TotalSim, processName, msgName, ephemData)
                
                    break

    return
