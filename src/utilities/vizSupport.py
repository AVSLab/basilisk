
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

# set the string type that works with Python 2 and 3
try:
  basestring
except NameError:
  basestring = str

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
            print('ERROR: lineColor must be a 4D array of integers')
            exit(1)
        if max(color) > 255 or min(color)<0:
            print('ERROR: lineColor values must be between [0,255]')
            exit(1)
        answer = color
    return answer

pointLineList = []
def createPointLine(viz, **kwargs):
    vizElement = vizInterface.PointLine()

    unitTestSupport.checkMethodKeyword(
        ['fromBodyName', 'toBodyName', 'lineColor'],
        kwargs)

    if 'fromBodyName' in kwargs:
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print('ERROR: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = viz.spacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print('ERROR: toBodyName must be a string')
            exit(1)
        vizElement.toBodyName = toName
    else:
        print('ERROR: toBodyName must be a specified')
        exit(1)

    if 'lineColor' in kwargs:
        vizElement.lineColor = toRGBA255(kwargs['lineColor'])
    else:
        print('ERROR: lineColor must be a specified')
        exit(1)

    pointLineList.append(vizElement)
    del viz.settings.pointLineList[:] # clear settings list to replace it with updated list
    viz.settings.pointLineList = vizInterface.PointLineConfig(pointLineList)
    return

customModelList = []
def createCustomModel(viz, **kwargs):
    vizElement = vizInterface.CustomModel()

    unitTestSupport.checkMethodKeyword(
        ['modelPath', 'simBodiesToModify', 'offset', 'rotation', 'scale', 'customTexturePath',
         'normalMapPath', 'shader'],
        kwargs)

    if 'modelPath' in kwargs:
        modelPathName = kwargs['modelPath']
        if not isinstance(modelPathName, basestring):
            print('ERROR: modelPath must be a string')
            exit(1)
        if len(modelPathName) == 0:
            print('ERROR: modelPath is required and must be specified.')
            exit(1)
        vizElement.modelPath = modelPathName
    else:
        print('ERROR: modelPath is required and must be specified.')
        exit(1)

    if 'simBodiesToModify' in kwargs:
        simBodiesList = kwargs['simBodiesToModify']
        if not isinstance(simBodiesList, list):
            print('ERROR: simBodiesToModify must be a list of strings')
            exit(1)
        if len(simBodiesList) == 0:
            print('ERROR: simBodiesToModify must be a non-empty list of strings')
            exit(1)
        for item in simBodiesList:
            if not isinstance(item, basestring):
                print('ERROR: the simBody name must be a string, not ' + str(item))
                exit(1)
        vizElement.simBodiesToModify = vizInterface.StringVector(simBodiesList)
    else:
        vizElement.simBodiesToModify = vizInterface.StringVector([viz.spacecraftName])

    if 'offset' in kwargs:
        offsetVariable = kwargs['offset']
        if not isinstance(offsetVariable, list):
            print('ERROR: offset must be a list of three floats')
            exit(1)
        if len(offsetVariable) is not 3:
            print('ERROR: offset must be list of three floats')
            exit(1)
        vizElement.offset = offsetVariable
    else:
        vizElement.offset = [0.0, 0.0, 0.0]

    if 'rotation' in kwargs:
        rotationVariable = kwargs['rotation']
        if not isinstance(rotationVariable, list):
            print('ERROR: rotation must be a list of three floats')
            exit(1)
        if len(rotationVariable) is not 3:
            print('ERROR: rotation must be list of three floats')
            exit(1)
        vizElement.rotation = rotationVariable
    else:
        vizElement.rotation = [0.0, 0.0, 0.0]

    if 'scale' in kwargs:
        scaleVariable = kwargs['scale']
        if not isinstance(scaleVariable, list):
            print('ERROR: scale must be a list of three floats')
            exit(1)
        if len(scaleVariable) is not 3:
            print('ERROR: scale must be list of three floats')
            exit(1)
        vizElement.scale = scaleVariable
    else:
        vizElement.scale = [1.0, 1.0, 1.0]

    if 'customTexturePath' in kwargs:
        customTexturePathName = kwargs['customTexturePath']
        if not isinstance(customTexturePathName, basestring):
            print('ERROR: customTexturePath must be a string')
            exit(1)
        vizElement.customTexturePath = customTexturePathName
    else:
        vizElement.customTexturePath = ""

    if 'normalMapPath' in kwargs:
        normalMapPathName = kwargs['normalMapPath']
        if not isinstance(normalMapPathName, basestring):
            print('ERROR: normalMapPath must be a string')
            exit(1)
        vizElement.normalMapPath = normalMapPathName
    else:
        vizElement.normalMapPath = ""

    if 'shader' in kwargs:
        shaderVariable = kwargs['shader']
        if not isinstance(shaderVariable, int):
            print('ERROR: shader must be a an integer.')
            exit(1)
        if abs(shaderVariable) > 1:
            print('ERROR: shader must have a value of -1, 0 or +1.')
            exit(1)

        vizElement.shader = shaderVariable
    else:
        vizElement.shader = -1

    customModelList.append(vizElement)
    del viz.settings.customModelList[:] # clear settings list to replace it with updated list
    viz.settings.customModelList = vizInterface.CustomModelConfig(customModelList)
    return

actuatorGuiSettingList = []
def setActuatorGuiSetting(viz, **kwargs):
    vizElement = vizInterface.ActuatorGuiSettings()

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'viewThrusterPanel', 'viewThrusterHUD', 'viewRWPanel', 'viewRWHUD'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print('ERROR: spacecraftName must be a string')
            exit(1)
        vizElement.spacecraftName = scName
    else:
        vizElement.spacecraftName = viz.spacecraftName

    if 'viewThrusterPanel' in kwargs:
        setting = kwargs['viewThrusterPanel']
        if not isinstance(setting, bool):
            print('ERROR: viewThrusterPanel must be True or False')
            exit(1)
        vizElement.viewThrusterPanel = setting
    else:
        vizElement.viewThrusterPanel = -1

    if 'viewThrusterHUD' in kwargs:
        setting = kwargs['viewThrusterHUD']
        if not isinstance(setting, bool):
            print('ERROR: viewThrusterHUD must be True or False')
            exit(1)
        vizElement.viewThrusterHUD = setting
    else:
        vizElement.viewThrusterHUD = -1

    if 'viewRWPanel' in kwargs:
        setting = kwargs['viewRWPanel']
        if not isinstance(setting, bool):
            print('ERROR: viewRWPanel must be True or False')
            exit(1)
        vizElement.viewRWPanel = setting
    else:
        vizElement.viewRWPanel = -1

    if 'viewRWHUD' in kwargs:
        setting = kwargs['viewRWHUD']
        if not isinstance(setting, bool):
            print('ERROR: viewRWHUD must be an integer value')
            exit(1)
        vizElement.viewRWHUD = setting
    else:
        vizElement.viewRWHUD = -1

    actuatorGuiSettingList.append(vizElement)
    del viz.settings.actuatorGuiSettingsList[:]  # clear settings list to replace it with updated list
    viz.settings.actuatorGuiSettingsList = vizInterface.ActuatorGuiSettingsConfig(actuatorGuiSettingList)
    return


coneInOutList = []
def createConeInOut(viz, **kwargs):
    vizElement = vizInterface.KeepOutInCone()

    unitTestSupport.checkMethodKeyword(
        ['fromBodyName', 'toBodyName', 'coneColor', 'isKeepIn', 'position_B', 'normalVector_B',
         'incidenceAngle', 'coneHeight', 'coneName'],
        kwargs)

    if 'fromBodyName' in kwargs:
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print('ERROR: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = viz.spacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print('ERROR: toBodyName must be a string')
            exit(1)
        vizElement.toBodyName = toName
    else:
        print('ERROR: toBodyName must be a specified')
        exit(1)

    if 'coneColor' in kwargs:
        vizElement.coneColor = toRGBA255(kwargs['coneColor'])
    else:
        print('ERROR: coneColor must be a specified')
        exit(1)

    if 'isKeepIn' in kwargs:
        keepInFlag = kwargs['isKeepIn']
        if not isinstance(keepInFlag, bool):
            print('ERROR: isKeepIn must be a BOOL')
            exit(1)
        vizElement.isKeepIn = keepInFlag
    else:
        print('ERROR: isKeepIn must be a specified')
        exit(1)

    if 'position_B' in kwargs:
        pos_B = kwargs['position_B']
        if not isinstance(pos_B, list):
            print('ERROR: position_B must be a 3D array of doubles')
            exit(1)
        vizElement.position_B = pos_B
    else:
        vizElement.position_B = [0.0, 0.0, 0.0]

    if 'normalVector_B' in kwargs:
        n_B = kwargs['normalVector_B']
        if not isinstance(n_B, list):
            print('ERROR: normalVector_B must be a 3D array of doubles')
            exit(1)
        vizElement.normalVector_B = n_B
    else:
        print('ERROR: normalVector_B must be a specified')
        exit(1)

    if 'incidenceAngle' in kwargs:
        angle = kwargs['incidenceAngle']
        if not isinstance(angle, float):
            print('ERROR: incidenceAngle must be a float value in radians')
            exit(1)
        vizElement.incidenceAngle = angle
    else:
        print('ERROR: incidenceAngle must be a specified')
        exit(1)

    if 'coneHeight' in kwargs:
        height = kwargs['coneHeight']
        if not isinstance(height, float):
            print('ERROR: coneHeight must be a float value')
            exit(1)
        vizElement.coneHeight = height
    else:
        print('ERROR: coneHeight must be a specified')
        exit(1)

    if 'coneName' in kwargs:
        coneName = kwargs['coneName']
        if not isinstance(coneName, basestring):
            print('ERROR: coneName must be a string')
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
        print('ERROR: camera name ' + camName + ' is not know.  Supported camera names are One, Two and Planet')
        exit(1)

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'viewPanel', 'setView', 'spacecraftVisible', 'fieldOfView', 'targetBodyName'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print('ERROR: ' + camName + ' spacecraftName must be a string')
            exit(1)
        cam.spacecraftName = scName
    else:
        cam.spacecraftName = viz.spacecraftName

    if 'viewPanel' in kwargs:
        viewPanel = kwargs['viewPanel']
        if not isinstance(viewPanel, bool):
            print('ERROR: ' + camName + ' viewPanel must be a bool')
            exit(1)
        cam.viewPanel = viewPanel

    if 'setView' in kwargs:
        setView = kwargs['setView']
        if not isinstance(setView, int):
            print('ERROR: ' + camName + ' setView must be an integer')
            exit(1)
        if camName=="Planet":
            if setView < 0 or setView > 2:
                print('ERROR: ' + camName + ' setView must be a number of [0,5]')
                print('0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track')
                exit(1)
        else:
            if setView < 0 or setView > 5:
                print('ERROR: ' + camName + ' setView must be a number of [0,5]')
                print('0 -> +X, 1 -> -X, 2 -> +Y, 3 -> -Y, 4 -> +Z, 5 -> -Z')
                exit(1)
        cam.setView = setView
    else:
        print('ERROR: ' + camName + ' setView must be a specified')
        exit(1)

    if 'spacecraftVisible' in kwargs:
        spacecraftVisible = kwargs['spacecraftVisible']
        if not isinstance(spacecraftVisible, bool):
            print('ERROR: ' + camName + ' spacecraftVisible must be a bool')
            exit(1)
        cam.spacecraftVisible = spacecraftVisible
    else:
        cam.spacecraftVisible = False

    if 'fieldOfView' in kwargs:
        fieldOfView = kwargs['fieldOfView']
        if not isinstance(fieldOfView, float):
            print('ERROR: ' + camName + ' spacecraftVisible must be a float in radians')
            exit(1)
        cam.fieldOfView = fieldOfView
    else:
        cam.fieldOfView = -1.0

    if 'targetBodyName' in kwargs:
        if camName=="Planet":
            targetBodyName = kwargs['targetBodyName']
            if not isinstance(targetBodyName, basestring):
                print('ERROR: ' + camName + ' targetBodyName must be a string')
                exit(1)
            cam.targetBodyName = targetBodyName
        else:
            print('WARNING: targetBodyName is not used for camera view One and Two')
    else:
        if camName=="Planet":
            print('ERROR: targetBodyName must be a specified')
            exit(1)

    return


def createCameraConfigMsg(viz, **kwargs):
    unitTestSupport.checkMethodKeyword(
        ['cameraID', 'parentName', 'fieldOfView', 'resolution', 'renderRate', 'focalLength', 'sensorSize', 'cameraPos_B', 'sigma_CB', 'skyBox'],
        kwargs)

    if 'cameraID' in kwargs:
        val = kwargs['cameraID']
        if not isinstance(val, int) or val < 0:
            print('ERROR: cameraID must be non-negative integer value.')
            exit(1)
        viz.cameraConfigMessage.cameraID = val
    else:
        print('ERROR: cameraID must be defined in createCameraConfigMsg()')
        exit(1)

    if 'parentName' in kwargs:
        val = kwargs['parentName']
        if not isinstance(val, basestring):
            print('ERROR: parentName must be a string')
            exit(1)
        viz.cameraConfigMessage.parentName = val
    else:
        viz.cameraConfigMessage.parentName = viz.spacecraftName

    if 'fieldOfView' in kwargs:
        val = kwargs['fieldOfView']
        if not isinstance(val, float):
            print('ERROR: fieldOfView must be a float in radians')
            exit(1)
        viz.cameraConfigMessage.fieldOfView = val
    else:
        print('ERROR: fieldOfView must be defined in createCameraConfigMsg()')
        exit(1)

    if 'resolution' in kwargs:
        val = kwargs['resolution']
        if not isinstance(val, list):
            print('ERROR: resolution must be a list')
            exit(1)
        if len(val) != 2:
            print('ERROR: resolution list ' + str(val) + 'must be of length 2')
            exit(1)
        if not isinstance(val[0], int) or not isinstance(val[1], int):
            print('ERROR: resolution list ' + str(val) + ' must contain integers')
            exit(1)
        viz.cameraConfigMessage.resolution = val
    else:
        print('ERROR: resolution must be defined in createCameraConfigMsg()')
        exit(1)

    if 'renderRate' in kwargs:
        val = kwargs['renderRate']
        if not isinstance(val, int) or val <= 0:
            print('ERROR: renderRate ' + str(val) + ' must be positive integer value.')
            exit(1)
        viz.cameraConfigMessage.renderRate = val
    else:
        print('ERROR: renderRate must be defined in createCameraConfigMsg()')
        exit(1)

    if 'sensorSize' in kwargs:
        val = kwargs['sensorSize']
        if not isinstance(val, list):
            print('ERROR: sensorSize must be a list')
            exit(1)
        if len(val) != 2:
            print('ERROR: sensorSize list ' + str(val) + 'must be of length 2')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float):
            print('ERROR: sensorSize list ' + str(val) + ' must contain floats')
            exit(1)
        viz.cameraConfigMessage.sensorSize = val
    else:
        print('ERROR: sensorSize must be defined in createCameraConfigMsg()')
        exit(1)

    if 'cameraPos_B' in kwargs:
        val = kwargs['cameraPos_B']
        if not isinstance(val, list):
            print('ERROR: cameraPos_B must be a list')
            exit(1)
        if len(val) != 3:
            print('ERROR: cameraPos_B list ' + str(val) + 'must be of length 3')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float) or not isinstance(val[2], float):
            print('ERROR: cameraPos_B list ' + str(val) + ' must contain floats')
            exit(1)
        viz.cameraConfigMessage.cameraPos_B = val
    else:
        print('ERROR: cameraPos_B must be defined in createCameraConfigMsg()')
        exit(1)

    if 'sigma_CB' in kwargs:
        val = kwargs['sigma_CB']
        if not isinstance(val, list):
            print('ERROR: sigma_CB must be a list')
            exit(1)
        if len(val) != 3:
            print('ERROR: camersigma_CBaPos_B list ' + str(val) + 'must be of length 3')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float) or not isinstance(val[2], float):
            print('ERROR: sigma_CB list ' + str(val) + ' must contain floats')
            exit(1)
        viz.cameraConfigMessage.sigma_CB = val
    else:
        print('ERROR: sigma_CB must be defined in createCameraConfigMsg()')
        exit(1)

    if 'skyBox' in kwargs:
        val = kwargs['skyBox']
        if not isinstance(val, basestring):
            print('ERROR: skyBox must be a string')
            exit(1)
        viz.cameraConfigMessage.skyBox = val
    else:
        viz.cameraConfigMessage.skyBox = ""

    return


def enableUnityVisualization(scSim, simTaskName, processName, **kwargs):
    if not vizFound:
        print('Could not find vizInterface when import attempted.  Be sure to build BSK with vizInterface support.')
        return

    # clear the list of point line elements
    del pointLineList[:]
    del actuatorGuiSettingList[:]
    del coneInOutList[:]

    unitTestSupport.checkMethodKeyword(
        ['saveFile', 'opNavMode', 'gravBodies', 'numRW', 'thrDevices', 'liveStream'],
        kwargs)

    # setup the Vizard interface module
    vizMessenger = vizInterface.VizInterface()
    scSim.AddModelToTask(simTaskName, vizMessenger)

    # set spacecraft name
    vizMessenger.spacecraftName = "bsk-Sat"

    # note that the following logic can receive a single file name, or a full path + file name.
    # In both cases a local results are stored in a local sub-folder.
    vizMessenger.saveFile = False
    if 'saveFile' in kwargs:
        fileNamePath = kwargs['saveFile']
        fileName = os.path.splitext(os.path.basename(fileNamePath))[0]
        filePath = os.path.dirname(fileNamePath)
        if filePath == "":
            filePath = "."
        if not os.path.isdir(filePath + '/_VizFiles'):
            os.mkdir(filePath + '/_VizFiles')
        vizFileNamePath = filePath + '/_VizFiles/' + fileName + '_UnityViz.bin'
        vizMessenger.saveFile = True
        vizMessenger.protoFilename = vizFileNamePath
        print("Saving Viz file to " + vizFileNamePath)

    if 'liveStream' in kwargs:
        val = kwargs['liveStream']
        if not isinstance(val, bool):
            print('ERROR: liveStream must True or False')
            exit(1)
        vizMessenger.liveStream = True
        if 'opNavMode' in kwargs:
            if kwargs['opNavMode'] > 0:
                print('ERROR: do not use liveStream and opNavMode flags at the same time.')
                exit(1)

    vizMessenger.opNavMode = 0
    if 'opNavMode' in kwargs:
        val = kwargs['opNavMode']
        if not isinstance(val, int):
            print('ERROR: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        if val < 0 or val > 2:
            print('ERROR: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        vizMessenger.opNavMode = val
        if val > 0:
            vizMessenger.opnavImageOutMsgName = "opnav_circles"

    vizMessenger.spiceInMsgName = vizInterface.StringVector(["earth_planet_data",
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
    if 'gravBodies' in kwargs:
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
