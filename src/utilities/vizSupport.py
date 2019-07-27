
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
from matplotlib import colors
import numpy as np
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

def toRGBA255(color):
    if isinstance(color, basestring):
        # convert color name to 4D array of values with 0-255
        answer = np.array(colors.to_rgba(color)) * 255
    else:
        if not isinstance(color, list):
            print 'ERROR: lineColor must be a 4D array of integers'
            exit(1)
        if max(color) > 255 or min(color)<0:
            print 'ERROR: lineColor values must be between [0,255]'
            exit(1)
        answer = color
    return answer

pointLineList = []
def createPointLine(viz, **kwargs):
    vizElement = vizInterface.PointLine()

    if kwargs.has_key('fromBodyName'):
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print 'ERROR: fromBodyName must be a string'
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = viz.spacecraftName

    if kwargs.has_key('toBodyName'):
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print 'ERROR: toBodyName must be a string'
            exit(1)
        vizElement.toBodyName = toName
    else:
        print 'ERROR: toBodyName must be a specified'
        exit(1)

    if kwargs.has_key('lineColor'):
        vizElement.lineColor = toRGBA255(kwargs['lineColor'])
    else:
        print 'ERROR: lineColor must be a specified'
        exit(1)

    pointLineList.append(vizElement)
    del viz.settings.pointLineList[:] # clear settings list to replace it with updated list
    viz.settings.pointLineList = vizInterface.PointLineConfig(pointLineList)
    return

coneInOutList = []
def createConeInOut(viz, **kwargs):
    vizElement = vizInterface.KeepOutInCone()

    if kwargs.has_key('fromBodyName'):
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print 'ERROR: fromBodyName must be a string'
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = viz.spacecraftName

    if kwargs.has_key('toBodyName'):
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print 'ERROR: toBodyName must be a string'
            exit(1)
        vizElement.toBodyName = toName
    else:
        print 'ERROR: toBodyName must be a specified'
        exit(1)

    if kwargs.has_key('coneColor'):
        vizElement.coneColor = toRGBA255(kwargs['coneColor'])
    else:
        print 'ERROR: coneColor must be a specified'
        exit(1)

    if kwargs.has_key('isKeepIn'):
        keepInFlag = kwargs['isKeepIn']
        if not isinstance(keepInFlag, bool):
            print 'ERROR: isKeepIn must be a BOOL'
            exit(1)
        vizElement.isKeepIn = keepInFlag
    else:
        print 'ERROR: isKeepIn must be a specified'
        exit(1)

    if kwargs.has_key('position_B'):
        pos_B = kwargs['position_B']
        if not isinstance(pos_B, list):
            print 'ERROR: position_B must be a 3D array of doubles'
            exit(1)
        vizElement.position_B = pos_B
    else:
        vizElement.position_B = [0.0, 0.0, 0.0]

    if kwargs.has_key('normalVector_B'):
        n_B = kwargs['normalVector_B']
        if not isinstance(n_B, list):
            print 'ERROR: normalVector_B must be a 3D array of doubles'
            exit(1)
        vizElement.normalVector_B = n_B
    else:
        print 'ERROR: normalVector_B must be a specified'
        exit(1)

    if kwargs.has_key('incidenceAngle'):
        angle = kwargs['incidenceAngle']
        if not isinstance(angle, float):
            print 'ERROR: incidenceAngle must be a float value'
            exit(1)
        vizElement.incidenceAngle = angle
    else:
        print 'ERROR: incidenceAngle must be a specified'
        exit(1)

    if kwargs.has_key('coneHeight'):
        height = kwargs['coneHeight']
        if not isinstance(height, float):
            print 'ERROR: coneHeight must be a float value'
            exit(1)
        vizElement.coneHeight = height
    else:
        print 'ERROR: coneHeight must be a specified'
        exit(1)

    if kwargs.has_key('coneName'):
        coneName = kwargs['coneName']
        if not isinstance(coneName, basestring):
            print 'ERROR: coneName must be a string'
            exit(1)
        vizElement.coneName = coneName
    else:
        vizElement.coneName = ""

    coneInOutList.append(vizElement)
    del viz.settings.coneList[:]  # clear settings list to replace it with updated list
    viz.settings.coneList = vizInterface.KeepOutInConeConfig(coneInOutList)
    return

def createCameraViewPanel(viz, camName, **kwargs):
    if camName == "One":
        cam = viz.settings.cameraOne
    elif camName == "Two":
        cam = viz.settings.cameraTwo
    elif camName == "Planet":
        cam = viz.settings.cameraPlanet
    else:
        print 'ERROR: camera name ' + camName + ' is not know.  Supported camera names are One, Two and Planet'
        exit(1)

    if kwargs.has_key('spacecraftName'):
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print 'ERROR: ' + camName + ' spacecraftName must be a string'
            exit(1)
        cam.spacecraftName = scName
    else:
        cam.spacecraftName = viz.spacecraftName

    if kwargs.has_key('viewPanel'):
        viewPanel = kwargs['viewPanel']
        if not isinstance(viewPanel, bool):
            print 'ERROR: ' + camName + ' viewPanel must be a bool'
            exit(1)
        cam.viewPanel = viewPanel

    if kwargs.has_key('setView'):
        setView = kwargs['setView']
        if not isinstance(setView, int):
            print 'ERROR: ' + camName + ' setView must be an integer'
            exit(1)
        if camName=="Planet":
            if setView < 0 or setView > 2:
                print 'ERROR: ' + camName + ' setView must be a number of [0,5]'
                print '0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track'
                exit(1)
        else:
            if setView < 0 or setView > 5:
                print 'ERROR: ' + camName + ' setView must be a number of [0,5]'
                print '0 -> +X, 1 -> -X, 2 -> +Y, 3 -> -Y, 4 -> +Z, 5 -> -Z'
                exit(1)
        cam.setView = setView
    else:
        print 'ERROR: ' + camName + ' setView must be a specified'
        exit(1)

    if kwargs.has_key('spacecraftVisible'):
        spacecraftVisible = kwargs['spacecraftVisible']
        if not isinstance(spacecraftVisible, bool):
            print 'ERROR: ' + camName + ' spacecraftVisible must be a bool'
            exit(1)
        cam.spacecraftVisible = spacecraftVisible
    else:
        cam.spacecraftVisible = False

    if kwargs.has_key('fieldOfView'):
        fieldOfView = kwargs['fieldOfView']
        if not isinstance(fieldOfView, float):
            print 'ERROR: ' + camName + ' spacecraftVisible must be a float in degrees'
            exit(1)
        cam.fieldOfView = fieldOfView
    else:
        cam.fieldOfView = -1.0

    if kwargs.has_key('targetBodyName'):
        if camName=="Planet":
            targetBodyName = kwargs['targetBodyName']
            if not isinstance(targetBodyName, basestring):
                print 'ERROR: ' + camName + ' targetBodyName must be a string'
                exit(1)
            cam.targetBodyName = targetBodyName
        else:
            print 'WARNING: targetBodyName is not used for camera view One and Two'
    else:
        if camName=="Planet":
            print 'ERROR: targetBodyName must be a specified'
            exit(1)

    return



def enableUnityVisualization(scSim, simTaskName, processName, **kwargs):
    if not vizFound:
        print('Could not find vizInterface when import attempted.  Be sure to build BSK with vizInterface support.')
        return

    # clear the list of point line elements
    del pointLineList[:]

    # setup the Vizard interface module
    vizMessenger = vizInterface.VizInterface()
    scSim.AddModelToTask(simTaskName, vizMessenger)

    # set spacecraft name
    vizMessenger.spacecraftName = "bsk-Sat"

    # note that the following logic can receive a single file name, or a full path + file name.
    # In both cases a local results are stored in a local sub-folder.
    vizMessenger.saveFile = 0
    if 'saveFile' in kwargs:
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
        print("Saving Viz file to " + vizFileNamePath)

    vizMessenger.opNavMode = 0
    if 'opNavMode' in kwargs:
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
    if ('gravBodies' in kwargs):
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

    if 'numRW' in kwargs:
        vizMessenger.numRW = kwargs['numRW']

    if 'thrDevices' in kwargs:
        thrDevices = kwargs['thrDevices']
        thList = []
        for thClusterInfo in thrDevices:
            thSet = vizInterface.ThrClusterMap()
            thSet.thrCount = thClusterInfo[0]
            thSet.thrTag = thClusterInfo[1]
            thList.append(thSet)
        vizMessenger.thrMsgData = vizInterface.VizThrConfig(thList)


    return vizMessenger
