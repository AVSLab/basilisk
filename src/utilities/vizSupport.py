
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
#   Unit Test Support Script
#
import sys, os
from matplotlib import colors
from matplotlib.colors import is_color_like
import numpy as np
from Basilisk.utilities import unitTestSupport
from Basilisk import __path__
bskPath = __path__[0]
from Basilisk.architecture import messaging

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

firstSpacecraftName = ''

def toRGBA255(color, alpha=None):
    if isinstance(color, basestring):
        # convert color name to 4D array of values with 0-255
        if is_color_like(color):

            answer = np.array(colors.to_rgba(color, alpha=alpha)) * 255
            answer = [round(a) for a in answer]
        else:
            print("toRGBA255() was provided unknown color name " + color)
            exit(1)
    else:
        if not isinstance(color, list):
            print('ERROR: vizSupport: color must be a 4D array of integers')
            exit(1)
        if max(color) > 255 or min(color) < 0:
            print('ERROR: vizSupport: color values must be between [0,255]')
            exit(1)
        answer = color
    return answer

def setSprite(shape, **kwargs):
    """
    Helper function to set the sprite shape and optional sprite color.
    :param shape: Sprite shape, must be either "CIRCLE", "SQUARE", "TRIANGLE", "STAR", or "bskSat"
    :param kwargs: RGBA color, can be either color name string or a 4D list of [0,255] values
    :return: string of the protobuffer sprite setting
    """
    unitTestSupport.checkMethodKeyword(
        ['color'],
        kwargs)
    shapeList = ["CIRCLE", "SQUARE", "TRIANGLE", "STAR", "bskSat"]

    if not isinstance(shape, basestring):
        print("In setSprite() the shape argument must be a string using " + str(shapeList))
        exit(1)

    if (shape not in shapeList):
        print("The setSprite() method was provided this unknown sprite shape primitive: " + shape)
        exit(1)

    answer = shape

    if 'color' in kwargs:
        colorInfo = kwargs['color']

        if shape == "bskSat":
            print("cannot set a color for the bskSat sprite option")
            exit(1)

        colorValues = toRGBA255(colorInfo)

        answer += " " + " ".join(map(str, colorValues))

    return answer


locationList = []
def addLocation(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return

    vizElement = vizInterface.LocationPbMsg()

    unitTestSupport.checkMethodKeyword(
        ['stationName', 'parentBodyName', 'r_GP_P', 'gHat_P', 'fieldOfView', 'color', 'range'],
        kwargs)

    if 'stationName' in kwargs:
        stationName = kwargs['stationName']
        if not isinstance(stationName, basestring):
            print('ERROR: stationName must be a string')
            exit(1)
        vizElement.stationName = stationName
    else:
        print("ERROR: stationName argument must be provided to addLocation")
        exit(0)

    if 'parentBodyName' in kwargs:
        parentBodyName = kwargs['parentBodyName']
        if not isinstance(parentBodyName, basestring):
            print('ERROR: parentBodyName must be a string')
            exit(1)
        vizElement.parentBodyName = parentBodyName
    else:
        print("ERROR: parentBodyName argument must be provided to addLocation")
        exit(1)

    if 'r_GP_P' in kwargs:
        r_GP_P = kwargs['r_GP_P']
        if not isinstance(r_GP_P, list):
            print('ERROR: r_GP_P must be a list of floats')
            print(r_GP_P)
            exit(1)
        if len(r_GP_P) != 3:
            print('ERROR: r_GP_P must be list of three floats')
            exit(1)
        try:
            # check if vector is a list
            vizElement.r_GP_P = r_GP_P
        except:
            try:
                # convert Eigen array to list
                vizElement.r_GP_P = unitTestSupport.EigenVector3d2np(r_GP_P).tolist()
            except:
                pass
    else:
        print("ERROR: r_GP_P argument must be provided to addLocation")
        exit(0)

    if 'gHat_P' in kwargs:
        gHat_P = kwargs['gHat_P']
        if not isinstance(gHat_P, list):
            print('ERROR: gHat_P must be a list of three floats')
            exit(1)
        if len(gHat_P) != 3:
            print('ERROR: gHat_P must be list of three floats')
            exit(1)
        vizElement.gHat_P = gHat_P
    else:
        vizElement.gHat_P = r_GP_P / np.linalg.norm(r_GP_P)

    if 'fieldOfView' in kwargs:
        fieldOfView = kwargs['fieldOfView']
        if not isinstance(fieldOfView, float):
            print('ERROR: fieldOfView must be a float value in radians')
            exit(1)
        if fieldOfView > np.pi or fieldOfView < 0.0:
            print('ERROR: fieldOfView must be a value between 0 and Pi')
            exit(1)
        vizElement.fieldOfView = fieldOfView

    if 'color' in kwargs:
        color = kwargs['color']
        vizElement.color = toRGBA255(color)

    if 'range' in kwargs:
        range = kwargs['range']
        if not isinstance(range, float):
            print('ERROR: range must be a float')
            exit(1)
        vizElement.range = range

    locationList.append(vizElement)
    del viz.locations[:]  # clear settings list to replace it with updated list
    viz.locations = vizInterface.LocationConfig(locationList)

    return


pointLineList = []
def createPointLine(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    global firstSpacecraftName
    vizElement = vizInterface.PointLine()

    unitTestSupport.checkMethodKeyword(
        ['fromBodyName', 'toBodyName', 'lineColor'],
        kwargs)

    if 'fromBodyName' in kwargs:
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print('ERROR: vizSupport: toBodyName must be a string')
            exit(1)
        vizElement.toBodyName = toName
    else:
        print('ERROR: vizSupport: toBodyName must be a specified')
        exit(1)

    if 'lineColor' in kwargs:
        vizElement.lineColor = toRGBA255(kwargs['lineColor'])
    else:
        print('ERROR: vizSupport: lineColor must be a specified')
        exit(1)

    pointLineList.append(vizElement)
    del viz.settings.pointLineList[:]  # clear settings list to replace it with updated list
    viz.settings.pointLineList = vizInterface.PointLineConfig(pointLineList)
    return

targetLineList = []
def createTargetLine(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    global firstSpacecraftName
    vizElement = vizInterface.PointLine()

    unitTestSupport.checkMethodKeyword(
        ['fromBodyName', 'toBodyName', 'lineColor'],
        kwargs)

    if 'fromBodyName' in kwargs:
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print('ERROR: vizSupport: toBodyName must be a string')
            exit(1)
        vizElement.toBodyName = toName
    else:
        print('ERROR: vizSupport: toBodyName must be a specified')
        exit(1)

    if 'lineColor' in kwargs:
        vizElement.lineColor = toRGBA255(kwargs['lineColor'])
    else:
        print('ERROR: vizSupport: lineColor must be a specified')
        exit(1)

    targetLineList.append(vizElement)
    updateTargetLineList(viz)
    return


def updateTargetLineList(viz):
    del viz.liveSettings.targetLineList[:]  # clear settings list to replace it with updated list
    viz.liveSettings.targetLineList = vizInterface.PointLineConfig(targetLineList)
    return

customModelList = []
def createCustomModel(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    global firstSpacecraftName
    vizElement = vizInterface.CustomModel()

    unitTestSupport.checkMethodKeyword(
        ['modelPath', 'simBodiesToModify', 'offset', 'rotation', 'scale', 'customTexturePath',
         'normalMapPath', 'shader'],
        kwargs)

    if 'modelPath' in kwargs:
        modelPathName = kwargs['modelPath']
        if not isinstance(modelPathName, basestring):
            print('ERROR: vizSupport: modelPath must be a string')
            exit(1)
        if len(modelPathName) == 0:
            print('ERROR: vizSupport: modelPath is required and must be specified.')
            exit(1)
        vizElement.modelPath = modelPathName
    else:
        print('ERROR: vizSupport: modelPath is required and must be specified.')
        exit(1)

    if 'simBodiesToModify' in kwargs:
        simBodiesList = kwargs['simBodiesToModify']
        if not isinstance(simBodiesList, list):
            print('ERROR: vizSupport: simBodiesToModify must be a list of strings')
            exit(1)
        if len(simBodiesList) == 0:
            print('ERROR: vizSupport: simBodiesToModify must be a non-empty list of strings')
            exit(1)
        for item in simBodiesList:
            if not isinstance(item, basestring):
                print('ERROR: vizSupport: the simBody name must be a string, not ' + str(item))
                exit(1)
        vizElement.simBodiesToModify = vizInterface.StringVector(simBodiesList)
    else:
        vizElement.simBodiesToModify = vizInterface.StringVector([firstSpacecraftName])

    if 'offset' in kwargs:
        offsetVariable = kwargs['offset']
        if not isinstance(offsetVariable, list):
            print('ERROR: vizSupport: offset must be a list of three floats')
            exit(1)
        if len(offsetVariable) != 3:
            print('ERROR: vizSupport: offset must be list of three floats')
            exit(1)
        vizElement.offset = offsetVariable
    else:
        vizElement.offset = [0.0, 0.0, 0.0]

    if 'rotation' in kwargs:
        rotationVariable = kwargs['rotation']
        if not isinstance(rotationVariable, list):
            print('ERROR: vizSupport: rotation must be a list of three floats')
            exit(1)
        if len(rotationVariable) != 3:
            print('ERROR: vizSupport: rotation must be list of three floats')
            exit(1)
        vizElement.rotation = rotationVariable
    else:
        vizElement.rotation = [0.0, 0.0, 0.0]

    if 'scale' in kwargs:
        scaleVariable = kwargs['scale']
        if not isinstance(scaleVariable, list):
            print('ERROR: vizSupport: scale must be a list of three floats')
            exit(1)
        if len(scaleVariable) != 3:
            print('ERROR: vizSupport: scale must be list of three floats')
            exit(1)
        vizElement.scale = scaleVariable
    else:
        vizElement.scale = [1.0, 1.0, 1.0]

    if 'customTexturePath' in kwargs:
        customTexturePathName = kwargs['customTexturePath']
        if not isinstance(customTexturePathName, basestring):
            print('ERROR: vizSupport: customTexturePath must be a string')
            exit(1)
        vizElement.customTexturePath = customTexturePathName
    else:
        vizElement.customTexturePath = ""

    if 'normalMapPath' in kwargs:
        normalMapPathName = kwargs['normalMapPath']
        if not isinstance(normalMapPathName, basestring):
            print('ERROR: vizSupport: normalMapPath must be a string')
            exit(1)
        vizElement.normalMapPath = normalMapPathName
    else:
        vizElement.normalMapPath = ""

    if 'shader' in kwargs:
        shaderVariable = kwargs['shader']
        if not isinstance(shaderVariable, int):
            print('ERROR: vizSupport: shader must be a an integer.')
            exit(1)
        if abs(shaderVariable) > 1:
            print('ERROR: vizSupport: shader must have a value of -1, 0 or +1.')
            exit(1)

        vizElement.shader = shaderVariable

    customModelList.append(vizElement)
    del viz.settings.customModelList[:]  # clear settings list to replace it with updated list
    viz.settings.customModelList = vizInterface.CustomModelConfig(customModelList)
    return

actuatorGuiSettingList = []
def setActuatorGuiSetting(viz, **kwargs):
    """
    This method sets the actuator GUI properties for a particular spacecraft.  If no ``spacecraftName`` is
    provided, then the name of the first spacecraft in the simulation is assumed.

    :param viz: copy of the vizInterface module
    :param kwargs: list of keyword arguments that this method supports
    :return: void

    Keyword Args
    ------------
    spacecraftName: str
        The name of the spacecraft for which the actuator GUI options are set.
        Default: If not provided, then the name of the first spacecraft in the simulation is used.
    viewThrusterPanel: bool
        flag if the GUI panel should be shown illustrating the thruster states
        Default: if not provided, then the Vizard default settings are used
    viewRWPanel: bool
        flag if the GUI panel should be shown illustrating the reaction wheel states
        Default: if not provided, then the Vizard default settings are used
    viewThrusterHUD: bool
        flag if the HUD visualization of the thruster states should be shown
        Default: if not provided, then the Vizard default settings are used
    viewRWHUD: bool
        flag if the HUD visualization of the reaction wheel states should be shown
        Default: if not provided, then the Vizard default settings are used
    showThrusterLabels: bool
        flag if the thruster labels should be shown
        Default: if not provided, then the Vizard default settings are used
    showRWLabels: bool
        flag if the reaction wheel labels should be shown
        Default: if not provided, then the Vizard default settings are used

    """
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return

    global firstSpacecraftName
    vizElement = vizInterface.ActuatorGuiSettings()

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'viewThrusterPanel', 'viewThrusterHUD', 'viewRWPanel', 'viewRWHUD',
         'showThrusterLabels', 'showRWLabels'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print('ERROR: vizSupport: spacecraftName must be a string')
            exit(1)
        vizElement.spacecraftName = scName
    else:
        vizElement.spacecraftName = firstSpacecraftName

    if 'viewThrusterPanel' in kwargs:
        setting = kwargs['viewThrusterPanel']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: viewThrusterPanel must be True or False')
            exit(1)
        vizElement.viewThrusterPanel = setting

    if 'viewThrusterHUD' in kwargs:
        setting = kwargs['viewThrusterHUD']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: viewThrusterHUD must be True or False')
            exit(1)
        vizElement.viewThrusterHUD = setting

    if 'viewRWPanel' in kwargs:
        setting = kwargs['viewRWPanel']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: viewRWPanel must be True or False')
            exit(1)
        vizElement.viewRWPanel = setting

    if 'viewRWHUD' in kwargs:
        setting = kwargs['viewRWHUD']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: viewRWHUD must be an integer value')
            exit(1)
        vizElement.viewRWHUD = setting

    if 'showThrusterLabels' in kwargs:
        setting = kwargs['showThrusterLabels']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: showThrusterLabels must be an integer value')
            exit(1)
        vizElement.showThrusterLabels = setting

    if 'showRWLabels' in kwargs:
        setting = kwargs['showRWLabels']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: showRWLabels must be an integer value')
            exit(1)
        vizElement.showRWLabels = setting

    actuatorGuiSettingList.append(vizElement)
    del viz.settings.actuatorGuiSettingsList[:]  # clear settings list to replace it with updated list
    viz.settings.actuatorGuiSettingsList = vizInterface.ActuatorGuiSettingsConfig(actuatorGuiSettingList)
    return

instrumentGuiSettingList = []
def setInstrumentGuiSetting(viz, **kwargs):
    """
    This method sets the instrument GUI properties for a particular spacecraft.  If no ``spacecraftName`` is
    provided, then the name of the first spacecraft in the simulation is assumed.

    :param viz: copy of the vizInterface module
    :param kwargs: list of keyword arguments that this method supports
    :return: void

    Keyword Args
    ------------
    spacecraftName: str
        The name of the spacecraft for which the actuator GUI options are set.
        Default: 0 - If not provided, then the name of the first spacecraft in the simulation is used.
    viewCSSPanel: int
        flag if the GUI panel should be shown (1) or hidden (-1) illustrating the CSS states
        Default: 0 - if not provided, then the Vizard default settings are used
    viewCSSCoverage: int
        flag if the HUD spherical coverage of the CSS states should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    viewCSSBoresight: int
        flag if the HUD boresight axes of the CSS states should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showCSSLabels: int
        flag if the CSS labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showGenericSensorLabels: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used

    """
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return

    global firstSpacecraftName
    vizElement = vizInterface.InstrumentGuiSettings()

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'viewCSSPanel', 'viewCSSCoverage', 'viewCSSBoresight', 'showCSSLabels',
         'showGenericSensorLabels'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print('ERROR: vizSupport: spacecraftName must be a string')
            exit(1)
        vizElement.spacecraftName = scName
    else:
        vizElement.spacecraftName = firstSpacecraftName

    if 'viewCSSPanel' in kwargs:
        setting = kwargs['viewCSSPanel']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        vizElement.viewCSSPanel = setting
        print(vizElement.viewCSSPanel)

    if 'viewCSSCoverage' in kwargs:
        setting = kwargs['viewCSSCoverage']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: viewCSSCoverage must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        vizElement.viewCSSCoverage = setting

    if 'viewCSSBoresight' in kwargs:
        setting = kwargs['viewCSSBoresight']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: viewCSSBoresight must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        vizElement.viewCSSBoresight = setting

    if 'showCSSLabels' in kwargs:
        setting = kwargs['showCSSLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showCSSLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        vizElement.showCSSLabels = setting

    if 'showGenericSensorLabels' in kwargs:
        setting = kwargs['showGenericSensorLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showGenericSensorLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        vizElement.showGenericSensorLabels = setting

    instrumentGuiSettingList.append(vizElement)
    del viz.settings.instrumentGuiSettingsList[:]  # clear settings list to replace it with updated list
    viz.settings.instrumentGuiSettingsList = vizInterface.InstrumentGuiSettingsConfig(instrumentGuiSettingList)
    return

coneInOutList = []
def createConeInOut(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    global firstSpacecraftName
    vizElement = vizInterface.KeepOutInCone()

    unitTestSupport.checkMethodKeyword(
        ['fromBodyName', 'toBodyName', 'coneColor', 'isKeepIn', 'position_B', 'normalVector_B',
         'incidenceAngle', 'coneHeight', 'coneName'],
        kwargs)

    if 'fromBodyName' in kwargs:
        fromName = kwargs['fromBodyName']
        if not isinstance(fromName, basestring):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, basestring):
            print('ERROR: vizSupport: toBodyName must be a string')
            exit(1)
        vizElement.toBodyName = toName
    else:
        print('ERROR: vizSupport: toBodyName must be a specified')
        exit(1)

    if 'coneColor' in kwargs:
        vizElement.coneColor = toRGBA255(kwargs['coneColor'])
    else:
        print('ERROR: vizSupport: coneColor must be a specified')
        exit(1)

    if 'isKeepIn' in kwargs:
        keepInFlag = kwargs['isKeepIn']
        if not isinstance(keepInFlag, bool):
            print('ERROR: vizSupport: isKeepIn must be a BOOL')
            exit(1)
        vizElement.isKeepIn = keepInFlag
    else:
        print('ERROR: vizSupport: isKeepIn must be a specified')
        exit(1)

    if 'position_B' in kwargs:
        pos_B = kwargs['position_B']
        if not isinstance(pos_B, list):
            print('ERROR: vizSupport: position_B must be a 3D array of doubles')
            exit(1)
        vizElement.position_B = pos_B
    else:
        vizElement.position_B = [0.0, 0.0, 0.0]

    if 'normalVector_B' in kwargs:
        n_B = kwargs['normalVector_B']
        if not isinstance(n_B, list):
            print('ERROR: vizSupport: normalVector_B must be a 3D array of doubles')
            exit(1)
        vizElement.normalVector_B = n_B
    else:
        print('ERROR: vizSupport: normalVector_B must be a specified')
        exit(1)

    if 'incidenceAngle' in kwargs:
        angle = kwargs['incidenceAngle']
        if not isinstance(angle, float):
            print('ERROR: vizSupport: incidenceAngle must be a float value in radians')
            exit(1)
        vizElement.incidenceAngle = angle
    else:
        print('ERROR: vizSupport: incidenceAngle must be a specified')
        exit(1)

    if 'coneHeight' in kwargs:
        height = kwargs['coneHeight']
        if not isinstance(height, float):
            print('ERROR: vizSupport: coneHeight must be a float value')
            exit(1)
        vizElement.coneHeight = height
    else:
        print('ERROR: vizSupport: coneHeight must be a specified')
        exit(1)

    if 'coneName' in kwargs:
        coneName = kwargs['coneName']
        if not isinstance(coneName, basestring):
            print('ERROR: vizSupport: coneName must be a string')
            exit(1)
        vizElement.coneName = coneName
    else:
        vizElement.coneName = ""

    coneInOutList.append(vizElement)
    del viz.settings.coneList[:]  # clear settings list to replace it with updated list
    viz.settings.coneList = vizInterface.KeepOutInConeConfig(coneInOutList)
    return

stdCameraList = []
def createStandardCamera(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    cam = vizInterface.StdCameraSettings()

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'setMode', 'setView', 'fieldOfView',
         'bodyTarget', 'pointingVector_B', 'position_B'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, basestring):
            print('ERROR: vizSupport: spacecraftName must be a string, you provided ' + str(scName))
            exit(1)
        cam.spacecraftName = scName
    else:
        cam.spacecraftName = firstSpacecraftName

    if 'setMode' in kwargs:
        setMode = kwargs['setMode']
        if not isinstance(setMode, int):
            print('ERROR: vizSupport: setMode must be an integer')
            exit(1)
        if setMode < 0 or setMode > 2:
            print('ERROR: vizSupport: setMode must be a 0 (body targeting) or 1 (pointing vector)')
            exit(1)
        cam.setMode = setMode

    if 'setView' in kwargs:
        setView = kwargs['setView']
        if cam.setMode == 1:
            print('ERROR: vizSupport: setView does not apply to pointing vector mode.')
            exit(1)
        if not isinstance(setView, int):
            print('ERROR: vizSupport: setView must be an integer')
            exit(1)
        if setView < 0 or setView > 5:
            print('ERROR: vizSupport: setView must be a number of [0,2]')
            print('0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track (default to nadir). '
                  'This is a setting for body targeting mode.')
            exit(1)
        cam.setView = setView

    if 'fieldOfView' in kwargs:
        fieldOfView = kwargs['fieldOfView']
        if not isinstance(fieldOfView, float):
            print('ERROR: vizSupport: spacecraftVisible must be a float in radians')
            exit(1)
        cam.fieldOfView = fieldOfView

    if 'bodyTarget' in kwargs:
        if cam.setMode == 1:
            print('ERROR: vizSupport: bodyTarget does not apply in pointing vector mode')
            exit(1)
        bodyTargetName = kwargs['bodyTarget']
        if not isinstance(bodyTargetName, basestring):
            print('ERROR: vizSupport: targetBodyName must be a string')
            exit(1)
        cam.bodyTarget = bodyTargetName
    else:
        cam.bodyTarget = ""

    if 'pointingVector_B' in kwargs:
        if cam.setMode == 0:
            print('ERROR: vizSupport: pointingVector_B does not apply in body pointing mode')
            exit(1)
        pointingVector_B = kwargs['pointingVector_B']
        if not isinstance(pointingVector_B, list):
            print('ERROR: vizSupport: pointingVector_B must be a 3D array of doubles')
            exit(1)
        if len(pointingVector_B) != 3:
            print('ERROR: vizSupport: pointingVector_B must be 3D list')
            exit(1)
        cam.pointingVector_B = pointingVector_B
    else:
        cam.pointingVector_B = [1.0, 0.0, 0.0]

    if 'position_B' in kwargs:
        position_B = kwargs['position_B']
        if len(position_B) != 3:
            print('ERROR: vizSupport: position_B must be 3D list of float values')
            exit(1)
        cam.position_B = position_B
    else:
        cam.position_B = [0, 0, 0]

    stdCameraList.append(cam)
    del viz.settings.stdCameraList[:]  # clear settings list to replace it with updated list
    viz.settings.stdCameraList = vizInterface.StdCameraConfig(stdCameraList)
    return


def createCameraConfigMsg(viz, **kwargs):
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    global firstSpacecraftName
    unitTestSupport.checkMethodKeyword(
        ['cameraID', 'parentName', 'fieldOfView', 'resolution', 'renderRate', 'cameraPos_B', 'sigma_CB', 'skyBox'],
        kwargs)

    if 'cameraID' in kwargs:
        val = kwargs['cameraID']
        if not isinstance(val, int) or val < 0:
            print('ERROR: vizSupport: cameraID must be non-negative integer value.')
            exit(1)
        viz.cameraConfigBuffer.cameraID = val
    else:
        print('ERROR: vizSupport: cameraID must be defined in createCameraConfigMsg()')
        exit(1)

    if 'parentName' in kwargs:
        val = kwargs['parentName']
        if not isinstance(val, basestring):
            print('ERROR: vizSupport: parentName must be a string')
            exit(1)
        viz.cameraConfigBuffer.parentName = val
    else:
        viz.cameraConfigBuffer.parentName = firstSpacecraftName

    if 'fieldOfView' in kwargs:
        val = kwargs['fieldOfView']
        if not isinstance(val, float):
            print('ERROR: vizSupport: fieldOfView must be a float in radians')
            exit(1)
        viz.cameraConfigBuffer.fieldOfView = val
    else:
        print('ERROR: vizSupport: fieldOfView must be defined in createCameraConfigMsg()')
        exit(1)

    if 'resolution' in kwargs:
        val = kwargs['resolution']
        if not isinstance(val, list):
            print('ERROR: vizSupport: resolution must be a list')
            exit(1)
        if len(val) != 2:
            print('ERROR: vizSupport: resolution list ' + str(val) + 'must be of length 2')
            exit(1)
        if not isinstance(val[0], int) or not isinstance(val[1], int):
            print('ERROR: vizSupport: resolution list ' + str(val) + ' must contain integers')
            exit(1)
        viz.cameraConfigBuffer.resolution = val
    else:
        print('ERROR: vizSupport: resolution must be defined in createCameraConfigMsg()')
        exit(1)

    if 'renderRate' in kwargs:
        val = kwargs['renderRate']
        if not isinstance(val, float) or val < 0:
            print('ERROR: vizSupport: renderRate ' + str(val) + ' must be positive float value in units of seconds.')
            exit(1)
        viz.cameraConfigBuffer.renderRate = int(val * 1e9)     # convert to nano-seconds

    if 'cameraPos_B' in kwargs:
        val = kwargs['cameraPos_B']
        if not isinstance(val, list):
            print('ERROR: vizSupport: cameraPos_B must be a list')
            exit(1)
        if len(val) != 3:
            print('ERROR: vizSupport: cameraPos_B list ' + str(val) + 'must be of length 3')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float) or not isinstance(val[2], float):
            print('ERROR: vizSupport: cameraPos_B list ' + str(val) + ' must contain floats')
            exit(1)
        viz.cameraConfigBuffer.cameraPos_B = val
    else:
        print('ERROR: vizSupport: cameraPos_B must be defined in createCameraConfigMsg()')
        exit(1)

    if 'sigma_CB' in kwargs:
        val = kwargs['sigma_CB']
        if not isinstance(val, list):
            print('ERROR: vizSupport: sigma_CB must be a list')
            exit(1)
        if len(val) != 3:
            print('ERROR: vizSupport: camersigma_CBaPos_B list ' + str(val) + 'must be of length 3')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float) or not isinstance(val[2], float):
            print('ERROR: vizSupport: sigma_CB list ' + str(val) + ' must contain floats')
            exit(1)
        viz.cameraConfigBuffer.sigma_CB = val
    else:
        print('ERROR: vizSupport: sigma_CB must be defined in createCameraConfigMsg()')
        exit(1)

    if 'skyBox' in kwargs:
        val = kwargs['skyBox']
        if not isinstance(val, basestring):
            print('ERROR: vizSupport: skyBox must be a string')
            exit(1)
        viz.cameraConfigBuffer.skyBox = val
    else:
        viz.cameraConfigBuffer.skyBox = ""

    return


def enableUnityVisualization(scSim, simTaskName, scList, **kwargs):
    """
    This methods creates an instance of the vizInterface() modules and setups up associated Vizard
    configuration setting messages.

    Parameters
    ----------
    scSim:
        variable with the simulationBaseClass copy
    simTaskName:
        task to which to add the vizInterface module
    scList:
        :ref:`spacecraft` objects.  Can be a single object or list of objects

    Keyword Args
    ------------
    saveFile: str
        can be a single file name, or a full path + file name. In both cases a local results are stored
        in a local sub-folder.
        Default: empty string resulting in the data not being saved to a file
    rwEffectorList: single or list of ``ReactionWheelStateEffector``
        The list must have the same length ``scList``.  Each entry is the :ref:`ReactionWheelStateEffector` instance
        for the spacecraft, or ``None`` if the spacecraft has no RW devices.
    thrEffectorList: single or double-list of :ref:`ThrusterDynamicEffector`
        The list must have the same length ``scList``.  Each entry is a list of :ref:`ReactionWheelStateEffector`
        instances
        for the spacecraft denoting a thruster cluster, or ``None`` if the spacecraft has no thruster devices.
    thrColors: single or vector of int(4)
        array of RGBA color values for each thruster set.  The list must have the same length as ``scList``.
        Each list entry is a list of RGBA array values for each cluster set.
    cssList:
        list of lists of :ref:`CoarseSunSensor` objects.  The outer list length must match ``scList``.
    genericSensorList:
        list of lists of ``GenericSensor`` structures.  The outer list length must match ``scList``.
    genericSensorCmdInMsgs:
        list of lists of :ref:`DeviceCmdMsgPayload` sensor state messages.  The outer list length must
        match ``scList``.  If the spacecraft has no sensor command msg, then use ``None``.
    opNavMode: bool
        flag if opNaveMode should be used
    liveStream: bool
        flag if live data streaming to Vizard should be used

    Returns
    -------
    :ref:`vizInterface` object
        copy of the vizInterface instance

    """
    if not vizFound:
        print('Could not find vizInterface when import attempted.  Be sure to build BSK with vizInterface support.')
        return

    # clear the list of point line elements
    del pointLineList[:]
    del actuatorGuiSettingList[:]
    del coneInOutList[:]
    global firstSpacecraftName

    unitTestSupport.checkMethodKeyword(
        ['saveFile', 'opNavMode', 'rwEffectorList', 'thrEffectorList', 'thrColors', 'liveStream', 'cssList',
         'genericSensorList'],
        kwargs)

    # setup the Vizard interface module
    vizMessenger = vizInterface.VizInterface()
    vizMessenger.ModelTag = "vizMessenger"
    scSim.AddModelToTask(simTaskName, vizMessenger)

    # ensure the spacecraft object list is a list
    if not isinstance(scList, list):
        scList = [scList]

    firstSpacecraftName = scList[0].ModelTag

    # process the RW effector argument
    rwEffectorScList = False
    if 'rwEffectorList' in kwargs:
        rwEffectorScList = kwargs['rwEffectorList']
        if not isinstance(rwEffectorScList, list):
            rwEffectorScList = [rwEffectorScList]
        if len(rwEffectorScList) != len(scList):
            print('ERROR: vizSupport: rwEffectorList should have the same length as the number of spacecraft')
            exit(1)

    thrEffectorScList = False
    if 'thrEffectorList' in kwargs:
        thrEffectorScList = kwargs['thrEffectorList']
        if not isinstance(thrEffectorScList, list):
            thrEffectorScList = [[thrEffectorScList]]
        if len(thrEffectorScList) != len(scList):
            print('ERROR: vizSupport: thrEffectorList should have the same length as the number of spacecraft')
            exit(1)
    thrColorsScList = False
    if 'thrColors' in kwargs:
        thrColorsScList = kwargs['thrColors']
        if not isinstance(thrColorsScList, list):
            thrColorsScList = [[thrColorsScList]]
        if len(thrColorsScList) != len(scList):
            print('ERROR: vizSupport: thrColors should have the same length as the number of spacecraft')
            exit(1)

    cssScList = False
    if 'cssList' in kwargs:
        cssScList = kwargs['cssList']
        if not isinstance(cssScList, list):
            cssScList = [[cssScList]]
        if len(cssScList) != len(scList):
            print('ERROR: vizSupport: cssList should have the same length as the number '
                  'of spacecraft and contain lists of CSSs')
            exit(1)

    gsScList = False
    if 'genericSensorList' in kwargs:
        gsScList = kwargs['genericSensorList']
        if not isinstance(gsScList, list):
            gsScList = [[gsScList]]
        if len(gsScList) != len(scList):
            print('ERROR: vizSupport: genericSensorList should have the same length as the '
                  'number of spacecraft and contain lists of generic sensors')
            exit(1)

    # loop over all spacecraft to associated states and msg information
    planetNameList = []
    planetInfoList = []
    spiceMsgList = []
    vizMessenger.scData.clear()
    c = 0
    for sc in scList:
        # create spacecraft information container
        scData = vizInterface.VizSpacecraftData()

        # set spacecraft name
        scData.spacecraftName = sc.ModelTag

        # link to spacecraft state message
        scData.scStateInMsg.subscribeTo(sc.scStateOutMsg)

        # link to celestial bodies information
        for gravBody in sc.gravField.gravBodies:
            # check if the celestial object has already been added
            if gravBody.planetName not in planetNameList:
                planetNameList.append(gravBody.planetName)
                planetInfo = vizInterface.GravBodyInfo()
                planetInfo.bodyName = gravBody.planetName
                planetInfo.mu = gravBody.mu
                planetInfo.radEquator = gravBody.radEquator
                planetInfo.radiusRatio = gravBody.radiusRatio
                planetInfoList.append(planetInfo)
                spiceMsgList.append(gravBody.planetBodyInMsg)

        # process RW effectors
        if rwEffectorScList:
            rwList = []
            if rwEffectorScList[c] is not None:
                # RWs have been added to this spacecraft
                for rwLogMsg in rwEffectorScList[c].rwOutMsgs:
                    rwList.append(rwLogMsg.addSubscriber())
            scData.rwInMsgs = messaging.RWConfigLogInMsgsVector(rwList)

        # process THR effectors
        if thrEffectorScList:
            thrList = []
            thrInfo = []
            if thrEffectorScList[c] is not None:  # THR clusters have been added to this spacecraft
                clusterCounter = 0
                for thrEff in thrEffectorScList[c]:  # loop over the THR effectors attached to this spacecraft
                    thSet = vizInterface.ThrClusterMap()
                    thSet.thrTag = thrEff.ModelTag  # set the label for this cluster of THR devices
                    if thrColorsScList:
                        if thrColorsScList[c] is not None:
                            thSet.color = thrColorsScList[c][clusterCounter]
                    for thrLogMsg in thrEff.thrusterOutMsgs:  # loop over the THR cluster log message
                        thrList.append(thrLogMsg.addSubscriber())
                        thrInfo.append(thSet)
                    clusterCounter += 1
            scData.thrInMsgs = messaging.THROutputInMsgsVector(thrList)
            scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)

        # process CSS information
        if cssScList:
            cssDeviceList = []
            if cssScList[c] is not None:  # CSS list has been added to this spacecraft
                for css in cssScList[c]:
                    cssDeviceList.append(css.cssConfigLogOutMsg.addSubscriber())
                scData.cssInMsgs = messaging.CSSConfigLogInMsgsVector(cssDeviceList)

        # process generic sensor information
        if gsScList:
            gsList = []
            if gsScList[c] is not None:  # generic sensor(s) have been added to this spacecraft
                for gs in gsScList[c]:
                    gsList.append(gs)
                scData.genericSensorList = vizInterface.GenericSensorVector(gsList)

        vizMessenger.scData.push_back(scData)
        c += 1

    vizMessenger.gravBodyInformation = vizInterface.GravBodyInfoVector(planetInfoList)
    vizMessenger.spiceInMsgs = messaging.SpicePlanetStateInMsgsVector(spiceMsgList)

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
            print('ERROR: vizSupport: liveStream must True or False')
            exit(1)
        vizMessenger.liveStream = val
        if 'opNavMode' in kwargs:
            if kwargs['opNavMode'] > 0:
                print('ERROR: vizSupport: do not use liveStream and opNavMode flags at the same time.')
                exit(1)

    vizMessenger.opNavMode = 0
    if 'opNavMode' in kwargs:
        val = kwargs['opNavMode']
        if not isinstance(val, int):
            print('ERROR: vizSupport: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        if val < 0 or val > 2:
            print('ERROR: vizSupport: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        vizMessenger.opNavMode = val
        if val > 0:
            vizMessenger.opnavImageOutMsgName = "opnav_circles"

    return vizMessenger
