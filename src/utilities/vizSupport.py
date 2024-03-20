
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

import numpy as np
import os
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import deprecated
from matplotlib import colors
from matplotlib.colors import is_color_like

try:
    from Basilisk.simulation import vizInterface
    vizFound = True
except ImportError:
    vizFound = False

bskPath = __path__[0]

firstSpacecraftName = ''

def toRGBA255(color, alpha=None):
    answer = [0, 0, 0, 0]
    if isinstance(color, str):
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

    if not isinstance(shape, str):
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
        if not isinstance(stationName, str):
            print('ERROR: stationName must be a string')
            exit(1)
        vizElement.stationName = stationName
    else:
        print("ERROR: stationName argument must be provided to addLocation")
        exit(0)

    if 'parentBodyName' in kwargs:
        parentBodyName = kwargs['parentBodyName']
        if not isinstance(parentBodyName, str):
            print('ERROR: parentBodyName must be a string')
            exit(1)
        vizElement.parentBodyName = parentBodyName
    else:
        print("ERROR: parentBodyName argument must be provided to addLocation")
        exit(1)

    r_GP_P = [0, 0, 0]
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
        if not isinstance(fromName, str):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, str):
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
        if not isinstance(fromName, str):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, str):
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
         'normalMapPath', 'shader', 'color'],
        kwargs)

    if 'modelPath' in kwargs:
        modelPathName = kwargs['modelPath']
        if not isinstance(modelPathName, str):
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
            if not isinstance(item, str):
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
        if not isinstance(customTexturePathName, str):
            print('ERROR: vizSupport: customTexturePath must be a string')
            exit(1)
        vizElement.customTexturePath = customTexturePathName
    else:
        vizElement.customTexturePath = ""

    if 'normalMapPath' in kwargs:
        normalMapPathName = kwargs['normalMapPath']
        if not isinstance(normalMapPathName, str):
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

    if 'color' in kwargs:
        colorVariable = kwargs['color']
        if not isinstance(colorVariable, list):
            print('ERROR: vizSupport: color must be a list of 4 integers')
            exit(1)
        if len(colorVariable) != 4:
            print('ERROR: vizSupport: offset must be list of 4 integers')
            exit(1)
        vizElement.color = vizInterface.IntVector(colorVariable)

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
        if not isinstance(scName, str):
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
            print('ERROR: vizSupport: viewRWHUD must be True or False')
            exit(1)
        vizElement.viewRWHUD = setting

    if 'showThrusterLabels' in kwargs:
        setting = kwargs['showThrusterLabels']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: showThrusterLabels must be True or False')
            exit(1)
        vizElement.showThrusterLabels = setting

    if 'showRWLabels' in kwargs:
        setting = kwargs['showRWLabels']
        if not isinstance(setting, bool):
            print('ERROR: vizSupport: showRWLabels must be an True or False')
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
    showTransceiverLabels: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showTransceiverFrustrum: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showGenericStoragePanel: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showMultiSphereLabels: int
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
         'showGenericSensorLabels', 'showTransceiverLabels', 'showTransceiverFrustrum',
         'showGenericStoragePanel', 'showMultiSphereLabels'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, str):
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
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.viewCSSPanel = setting
        print(vizElement.viewCSSPanel)

    if 'viewCSSCoverage' in kwargs:
        setting = kwargs['viewCSSCoverage']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: viewCSSCoverage must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.viewCSSCoverage = setting

    if 'viewCSSBoresight' in kwargs:
        setting = kwargs['viewCSSBoresight']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: viewCSSBoresight must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.viewCSSBoresight = setting

    if 'showCSSLabels' in kwargs:
        setting = kwargs['showCSSLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showCSSLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: viewCSSPanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showCSSLabels = setting

    if 'showGenericSensorLabels' in kwargs:
        setting = kwargs['showGenericSensorLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showGenericSensorLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: showGenericSensorLabels must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showGenericSensorLabels = setting

    if 'showTransceiverLabels' in kwargs:
        setting = kwargs['showTransceiverLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showTransceiverLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: showTransceiverLabels must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showTransceiverLabels = setting

    if 'showTransceiverFrustrum' in kwargs:
        setting = kwargs['showTransceiverFrustrum']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showTransceiverFrustrum must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: showTransceiverFrustrum must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showTransceiverFrustrum = setting

    if 'showGenericStoragePanel' in kwargs:
        setting = kwargs['showGenericStoragePanel']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showGenericStoragePanel must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: showGenericStoragePanel must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showGenericStoragePanel = setting

    if 'showMultiSphereLabels' in kwargs:
        setting = kwargs['showMultiSphereLabels']
        if not isinstance(setting, int):
            print('ERROR: vizSupport: showMultiSphereLabels must be  -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        if setting is False:
            setting = -1
        if setting*setting > 1:
            print('ERROR: vizSupport: showMultiSphereLabels must be -1 (Off), 0 (default) or 1 (On)')
            exit(1)
        vizElement.showMultiSphereLabels = setting

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
        if not isinstance(fromName, str):
            print('ERROR: vizSupport: fromBodyName must be a string')
            exit(1)
        vizElement.fromBodyName = fromName
    else:
        vizElement.fromBodyName = firstSpacecraftName

    if 'toBodyName' in kwargs:
        toName = kwargs['toBodyName']
        if not isinstance(toName, str):
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
        if not isinstance(coneName, str):
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
    '''
    add a standard camera window
    '''
    if not vizFound:
        print('vizFound is false. Skipping this method.')
        return
    cam = vizInterface.StdCameraSettings()

    unitTestSupport.checkMethodKeyword(
        ['spacecraftName', 'setMode', 'setView', 'fieldOfView',
         'bodyTarget', 'pointingVector_B', 'position_B', 'displayName'],
        kwargs)

    if 'spacecraftName' in kwargs:
        scName = kwargs['spacecraftName']
        if not isinstance(scName, str):
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
        if setView < 0 or setView > 2:
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
        if not isinstance(bodyTargetName, str):
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

    if 'displayName' in kwargs:
        displayName = kwargs['displayName']
        if not isinstance(displayName, str):
            print('ERROR: vizSupport: createStandardCamera: displayName must be a string')
            exit(1)
        cam.displayName = displayName

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
        ['cameraID', 'parentName', 'fieldOfView', 'resolution', 'renderRate', 'cameraPos_B',
         'sigma_CB', 'skyBox', 'postProcessingOn', 'ppFocusDistance', 'ppAperture', 'ppFocalLength',
         'ppMaxBlurSize', 'updateCameraParameters', 'renderMode', 'depthMapClippingPlanes'],
        kwargs)

    cameraConfigMsgPayload = messaging.CameraConfigMsgPayload()

    if 'cameraID' in kwargs:
        val = kwargs['cameraID']
        if not isinstance(val, int) or val < 0:
            print('ERROR: vizSupport: cameraID must be non-negative integer value.')
            exit(1)
        cameraConfigMsgPayload.cameraID = val
    else:
        print('ERROR: vizSupport: cameraID must be defined in createCameraConfigMsg()')
        exit(1)

    if 'parentName' in kwargs:
        val = kwargs['parentName']
        if not isinstance(val, str):
            print('ERROR: vizSupport: parentName must be a string')
            exit(1)
        cameraConfigMsgPayload.parentName = val
    else:
        cameraConfigMsgPayload.parentName = firstSpacecraftName

    if 'fieldOfView' in kwargs:
        val = kwargs['fieldOfView']
        if not isinstance(val, float):
            print('ERROR: vizSupport: fieldOfView must be a float in radians')
            exit(1)
        cameraConfigMsgPayload.fieldOfView = val
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
        cameraConfigMsgPayload.resolution = val
    else:
        print('ERROR: vizSupport: resolution must be defined in createCameraConfigMsg()')
        exit(1)

    if 'renderRate' in kwargs:
        val = kwargs['renderRate']
        if not isinstance(val, float) or val < 0:
            print('ERROR: vizSupport: renderRate ' + str(val) + ' must be positive float value in units of seconds.')
            exit(1)
        cameraConfigMsgPayload.renderRate = int(val * 1e9)     # convert to nano-seconds

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
        cameraConfigMsgPayload.cameraPos_B = val
    else:
        print('ERROR: vizSupport: cameraPos_B must be defined in createCameraConfigMsg()')
        exit(1)

    if 'sigma_CB' in kwargs:
        val = kwargs['sigma_CB']
        if not isinstance(val, list):
            print('ERROR: vizSupport: sigma_CB must be a list')
            exit(1)
        if len(val) != 3:
            print('ERROR: vizSupport: sigma_CB list ' + str(val) + 'must be of length 3')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float) or not isinstance(val[2], float):
            print('ERROR: vizSupport: sigma_CB list ' + str(val) + ' must contain floats')
            exit(1)
        cameraConfigMsgPayload.sigma_CB = val
    else:
        print('ERROR: vizSupport: sigma_CB must be defined in createCameraConfigMsg()')
        exit(1)

    if 'skyBox' in kwargs:
        val = kwargs['skyBox']
        if not isinstance(val, str):
            print('ERROR: vizSupport: skyBox must be a string')
            exit(1)
        cameraConfigMsgPayload.skyBox = val
    else:
        cameraConfigMsgPayload.skyBox = ""

    if 'postProcessingOn' in kwargs:
        val = kwargs['postProcessingOn']
        if not isinstance(val, int) or val < 0:
            print('ERROR: vizSupport: postProcessingOn must be non-negative integer value.')
            exit(1)
        cameraConfigMsgPayload.postProcessingOn = val

    if 'ppFocusDistance' in kwargs:
        val = kwargs['ppFocusDistance']
        if not isinstance(val, float) or val < 0:
            print('ERROR: vizSupport: ppFocusDistance ' + str(val) + ' must be 0 or greater than 0.1.')
            exit(1)
        cameraConfigMsgPayload.ppFocusDistance = int(val)

    if 'ppAperture' in kwargs:
        val = kwargs['ppAperture']
        if not isinstance(val, float) or val < 0 or val > 32:
            print('ERROR: vizSupport: ppAperture ' + str(val) + ' must be 0 or with [0.05, 32].')
            exit(1)
        cameraConfigMsgPayload.ppAperture = int(val)

    if 'ppFocalLength' in kwargs:
        val = kwargs['ppFocalLength']
        if not isinstance(val, float) or val < 0 or val > 0.3:
            print('ERROR: vizSupport: ppFocalLength ' + str(val) + ' must be 0 or with [0.001, 0.3] meters.')
            exit(1)
        cameraConfigMsgPayload.ppFocalLength = int(val)

    if 'ppMaxBlurSize' in kwargs:
        val = kwargs['ppMaxBlurSize']
        if not isinstance(val, int) or val < 0 or val > 4:
            print('ERROR: vizSupport: ppMaxBlurSize must be non-negative integer value between [0, 4].')
            exit(1)
        cameraConfigMsgPayload.ppMaxBlurSize = val

    if 'updateCameraParameters' in kwargs:
        val = kwargs['updateCameraParameters']
        if not isinstance(val, int) or val < 0 or val > 1:
            print('ERROR: vizSupport: updateCameraParameters must be 0 or 1.')
            exit(1)
        cameraConfigMsgPayload.cameraID = val
    else:
        cameraConfigMsgPayload.cameraID = 0

    if 'renderMode' in kwargs:
        val = kwargs['renderMode']
        if not isinstance(val, int) or val < 0 or val > 1:
            print('ERROR: vizSupport: renderMode must be 0 or 1.')
            exit(1)
        cameraConfigMsgPayload.renderMode = val
    else:
        cameraConfigMsgPayload.renderMode = 0

    if 'depthMapClippingPlanes' in kwargs:
        if cameraConfigMsgPayload.renderMode != 1:
            print('WARNING: vizSupport: depthMapClippingPlanes only works with renderMode set to 1 (depthMap).')
            exit(1)
        val = kwargs['depthMapClippingPlanes']
        if not isinstance(val, list):
            print('ERROR: vizSupport: depthMapClippingPlanes must be a list of two doubles.')
            exit(1)
        if len(val) != 2:
            print('ERROR: vizSupport: depthMapClippingPlanes list ' + str(val) + 'must be of length 2')
            exit(1)
        if not isinstance(val[0], float) or not isinstance(val[1], float):
            print('ERROR: vizSupport: depthMapClippingPlanes list ' + str(val) + ' must contain floats')
            exit(1)
        print(val)
        cameraConfigMsgPayload.depthMapClippingPlanes = val
    else:
        cameraConfigMsgPayload.depthMapClippingPlanes = [-1.0, -1.0]

    cameraConfigMsg = messaging.CameraConfigMsg().write(cameraConfigMsgPayload)
    cameraConfigMsg.this.disown()
    viz.addCamMsgToModule(cameraConfigMsg)

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
    broadcastStream: bool
        flag if messages should be broadcast for listener Vizards to pick up.
    noDisplay: bool
        flag if Vizard should run performance opNav (no Vizard display)
    genericStorageList:
        list of lists of ``GenericStorage`` structures.  The outer list length must match ``scList``.
    lightList:
        list of lists of ``Light`` structures.   The outer list length must match ``scList``.
    spriteList:
        list of sprite information for each spacecraft
    modelDictionaryKeyList:
        list of the spacecraft model dictionary.  Use ``None`` if default values are used
    oscOrbitColorList:
        list of spacecraft osculating orbit colors.  Can be 4 RGBA integer value (0-255), a color string, or
        ``None`` if default values should be used.  The array must be of the length of the spacecraft list
    trueOrbitColorList:
        list of spacecraft true or actual orbit colors.  Can be 4 RGBA integer value (0-255), a color string, or
        ``None`` if default values should be used.  The array must be of the length of the spacecraft list
    trueOrbitColorInMsgList:
        list of color messages to read and provide the true orbit color at each time step.  This overwrites
        the values set with trueOrbitColorList.
    msmInfoList:
        list of MSM configuration messages
    ellipsoidList:
        list of lists of ``Ellipsoid`` structures.  The outer list length must match ``scList``.

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
        ['saveFile', 'opNavMode', 'rwEffectorList', 'thrEffectorList', 'thrColors', 'cssList', 'liveStream', 'broadcastStream',        
         'noDisplay', 'genericSensorList', 'transceiverList', 'genericStorageList', 'lightList', 'spriteList',
         'modelDictionaryKeyList', 'oscOrbitColorList', 'trueOrbitColorList', 'logoTextureList',
         'msmInfoList', 'ellipsoidList', 'trueOrbitColorInMsgList'],
        kwargs)

    # setup the Vizard interface module
    vizMessenger = vizInterface.VizInterface()
    vizMessenger.ModelTag = "vizMessenger"
    scSim.AddModelToTask(simTaskName, vizMessenger)

    # ensure the spacecraft object list is a list
    if not isinstance(scList, list):
        scList = [scList]
    scListLength = len(scList)

    firstSpacecraftName = scList[0].ModelTag

    # process the RW effector argument
    rwEffectorScList = False
    if 'rwEffectorList' in kwargs:
        rwEffectorScList = kwargs['rwEffectorList']
        if not isinstance(rwEffectorScList, list):
            rwEffectorScList = [rwEffectorScList]
        if len(rwEffectorScList) != scListLength:
            print('ERROR: vizSupport: rwEffectorList should have the same length as the number of spacecraft')
            exit(1)

    thrEffectorScList = False
    if 'thrEffectorList' in kwargs:
        thrEffectorScList = kwargs['thrEffectorList']
        if not isinstance(thrEffectorScList, list):
            thrEffectorScList = [[thrEffectorScList]]
        if len(thrEffectorScList) != scListLength:
            print('ERROR: vizSupport: thrEffectorList should have the same length as the number of spacecraft')
            exit(1)
    thrColorsScList = False
    if 'thrColors' in kwargs:
        thrColorsScList = kwargs['thrColors']
        if len(thrColorsScList) == 4:
            colorCheck = True
            for c in thrColorsScList:
                if not isinstance(c, int):
                    colorCheck = False
            if colorCheck:
                thrColorsScList = [[thrColorsScList]]
        if len(thrColorsScList) != scListLength:
            print('ERROR: vizSupport: thrColors should have the same length as the number of spacecraft')
            exit(1)

    cssScList = False
    if 'cssList' in kwargs:
        cssScList = kwargs['cssList']
        if not isinstance(cssScList, list):
            cssScList = [[cssScList]]
        if len(cssScList) != scListLength:
            print('ERROR: vizSupport: cssList should have the same length as the number '
                  'of spacecraft and contain lists of CSSs')
            exit(1)

    gsScList = False
    if 'genericSensorList' in kwargs:
        gsScList = kwargs['genericSensorList']
        if not isinstance(gsScList, list):
            gsScList = [[gsScList]]
        if len(gsScList) != scListLength:
            print('ERROR: vizSupport: genericSensorList should have the same length as the '
                  'number of spacecraft and contain lists of generic sensors')
            exit(1)

    elScList = False
    if 'ellipsoidList' in kwargs:
        elScList = kwargs['ellipsoidList']
        if not isinstance(elScList, list):
            elScList = [[elScList]]
        if len(elScList) != scListLength:
            print('ERROR: vizSupport: ellipsoidList should have the same length as the '
                  'number of spacecraft and contain lists of generic sensors')
            exit(1)

    liScList = False
    if 'lightList' in kwargs:
        liScList = kwargs['lightList']
        if not isinstance(liScList, list):
            liScList = [[liScList]]
        if len(liScList) != scListLength:
            print('ERROR: vizSupport: lightList should have the same length as the '
                  'number of spacecraft and contain lists of light devices')
            exit(1)

    gsdScList = False
    if 'genericStorageList' in kwargs:
        gsdScList = kwargs['genericStorageList']
        if not isinstance(gsdScList, list):
            gsdScList = [[gsdScList]]
        if len(gsdScList) != scListLength:
            print('ERROR: vizSupport: genericStorageList should have the same length as the '
                  'number of spacecraft and contain lists of generic sensors')
            exit(1)

    tcScList = False
    if 'transceiverList' in kwargs:
        tcScList = kwargs['transceiverList']
        if not isinstance(tcScList, list):
            tcScList = [[tcScList]]
        if len(tcScList) != scListLength:
            print('ERROR: vizSupport: tcScList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)

    spriteScList = False
    if 'spriteList' in kwargs:
        spriteScList = kwargs['spriteList']
        if not isinstance(spriteScList, list):
            spriteScList = [spriteScList]
        if len(spriteScList) != scListLength:
            print('ERROR: vizSupport: spriteScList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)

    modelDictionaryKeyList = False
    if 'modelDictionaryKeyList' in kwargs:
        modelDictionaryKeyList = kwargs['modelDictionaryKeyList']
        if not isinstance(modelDictionaryKeyList, list):
            modelDictionaryKeyList = [modelDictionaryKeyList]
        if len(modelDictionaryKeyList) != scListLength:
            print('ERROR: vizSupport: modelDictionaryKeyList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)

    logoTextureList = False
    if 'logoTextureList' in kwargs:
        logoTextureList = kwargs['logoTextureList']
        if not isinstance(logoTextureList, list):
            logoTextureList = [logoTextureList]
        if len(logoTextureList) != scListLength:
            print('ERROR: vizSupport: logoTextureList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)

    oscOrbitColorList = False
    if 'oscOrbitColorList' in kwargs:
        oscOrbitColorList = kwargs['oscOrbitColorList']
        if len(oscOrbitColorList) != scListLength:
            print('ERROR: vizSupport: oscOrbitColorList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)
        for elem in oscOrbitColorList:
            if isinstance(elem, list):
                if len(elem) != 4:
                    print('ERROR: vizSupport: if specifying oscOrbitColorList color via RGBA values, you '
                          'must provide 4 integers values from 0 to 255 ')
                    exit(1)
                for color in elem:
                    if color < 0 or color > 255:
                        print('ERROR: vizSupport:  oscOrbitColorList color contained negative value ')
                        exit(1)

    trueOrbitColorList = False
    if 'trueOrbitColorList' in kwargs:
        trueOrbitColorList = kwargs['trueOrbitColorList']
        if len(trueOrbitColorList) != scListLength:
            print('ERROR: vizSupport: trueOrbitColorList should have the same length as the '
                  'number of spacecraft and contain lists of transceivers')
            exit(1)
        for elem in trueOrbitColorList:
            if isinstance(elem, list):
                if len(elem) != 4:
                    print('ERROR: vizSupport: if specifying trueOrbitColorList color via RGBA values, you '
                          'must provide 4 integers values from 0 to 255 ')
                    exit(1)
                for color in elem:
                    if color < 0 or color > 255:
                        print('ERROR: vizSupport:  trueOrbitColorList color contained negative value ')
                        exit(1)

    msmInfoList = False
    if 'msmInfoList' in kwargs:
        msmInfoList = kwargs['msmInfoList']
        if not isinstance(msmInfoList, list):
            msmInfoList = [msmInfoList]
        if len(msmInfoList) != scListLength:
            print('ERROR: vizSupport: msmInfoList should have the same length as the '
                  'number of spacecraft')
            exit(1)

    trueOrbitColorInMsgList = False
    if 'trueOrbitColorInMsgList' in kwargs:
        trueOrbitColorInMsgList = kwargs['trueOrbitColorInMsgList']
        if not isinstance(trueOrbitColorInMsgList, list):
            trueOrbitColorInMsgList = [trueOrbitColorInMsgList]
        if len(trueOrbitColorInMsgList) != scListLength:
            print('ERROR: vizSupport: trueOrbitColorInMsgList should have the same length as the '
                  'number of spacecraft')
            exit(1)

    # loop over all spacecraft to associated states and msg information
    planetNameList = []
    planetInfoList = []
    spiceMsgList = []
    vizMessenger.scData.clear()
    c = 0
    spacecraftParentName = ""

    for sc in scList:
        # create spacecraft information container
        scData = vizInterface.VizSpacecraftData()

        # link to spacecraft state message
        if isinstance(sc, type(spacecraft.Spacecraft())):
            # set spacecraft name
            scData.spacecraftName = sc.ModelTag
            spacecraftParentName = sc.ModelTag
            scData.scStateInMsg.subscribeTo(sc.scStateOutMsg)

            # link to celestial bodies information
            for gravBody in sc.gravField.gravBodies:
                # check if the celestial object has already been added
                if gravBody.planetName not in planetNameList:
                    planetNameList.append(gravBody.planetName)
                    planetInfo = vizInterface.GravBodyInfo()
                    if gravBody.displayName == "":
                        planetInfo.bodyName = gravBody.planetName
                    else:
                        planetInfo.bodyName = gravBody.displayName
                    planetInfo.mu = gravBody.mu
                    planetInfo.radEquator = gravBody.radEquator
                    planetInfo.radiusRatio = gravBody.radiusRatio
                    planetInfo.modelDictionaryKey = gravBody.modelDictionaryKey
                    planetInfoList.append(planetInfo)
                    spiceMsgList.append(gravBody.planetBodyInMsg)
        else:
            # the scList object is an effector belonging to the parent spacecraft
            scData.parentSpacecraftName = spacecraftParentName
            ModelTag = sc[0]
            effStateOutMsg = sc[1]
            scData.spacecraftName = ModelTag
            scData.scStateInMsg.subscribeTo(effStateOutMsg)

        # process RW effectors
        if rwEffectorScList:
            rwList = []
            if rwEffectorScList[c] is not None:
                # RWs have been added to this spacecraft
                for rwLogMsg in rwEffectorScList[c].rwOutMsgs:
                    rwList.append(rwLogMsg.addSubscriber())
            scData.rwInMsgs = messaging.RWConfigLogMsgInMsgsVector(rwList)

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
            scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(thrList)
            scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)

        # process CSS information
        if cssScList:
            cssDeviceList = []
            if cssScList[c] is not None:  # CSS list has been added to this spacecraft
                for css in cssScList[c]:
                    cssDeviceList.append(css.cssConfigLogOutMsg.addSubscriber())
                scData.cssInMsgs = messaging.CSSConfigLogMsgInMsgsVector(cssDeviceList)

        # process generic sensor HUD information
        if gsScList:
            gsList = []
            if gsScList[c] is not None:  # generic sensor(s) have been added to this spacecraft
                for gs in gsScList[c]:
                    gsList.append(gs)
                scData.genericSensorList = vizInterface.GenericSensorVector(gsList)

        # process spacecraft ellipsoids
        if elScList:
            elList = []
            if elScList[c] is not None:  # generic sensor(s) have been added to this spacecraft
                for el in elScList[c]:
                    elList.append(el)
                scData.ellipsoidList = vizInterface.EllipsoidVector(elList)

        # process spacecraft lights
        if liScList:
            liList = []
            if liScList[c] is not None: # light objects(s) have been added to this spacecraft
                for li in liScList[c]:
                    liList.append(li)
                scData.lightList = vizInterface.LightVector(liList)

        # process generic storage HUD information
        if gsdScList:
            gsdList = []
            if gsdScList[c] is not None:  # generic storage device(s) have been added to this spacecraft
                for gsd in gsdScList[c]:
                    if len(gsd.color) > 1:
                        if len(gsd.color)/4 != len(gsd.thresholds) + 1:
                            print("ERROR: vizSupport: generic storage " + gsd.label +
                                  " threshold list does not have the correct dimension.  "
                                  "It should be 1 smaller than the list of colors.")
                            exit(1)
                    else:
                        if len(gsd.thresholds) > 0:
                            print("ERROR: vizSupport: generic storage " + gsd.label +
                                  " threshold list is set, but no multiple of colors are provided.")
                            exit(1)
                    gsdList.append(gsd)
                scData.genericStorageList = vizInterface.GenericStorageVector(gsdList)

        # process transceiver HUD information
        if tcScList:
            tcList = []
            if tcScList[c] is not None:  # transceiver(s) have been added to this spacecraft
                for tc in tcScList[c]:
                    tcList.append(tc)
                scData.transceiverList = vizInterface.TransceiverVector(tcList)

        # process sprite information
        if spriteScList:
            if spriteScList[c] is not None:
                scData.spacecraftSprite = spriteScList[c]
        # process modelDictionaryKey information
        if modelDictionaryKeyList:
            if modelDictionaryKeyList[c] is not None:
                scData.modelDictionaryKey = modelDictionaryKeyList[c]
        # process logoTexture information
        if logoTextureList:
            if logoTextureList[c] is not None:
                scData.logoTexture = logoTextureList[c]

        if oscOrbitColorList:
            if oscOrbitColorList[c] is not None:
                scData.oscOrbitLineColor = vizInterface.IntVector(oscOrbitColorList[c])

        if trueOrbitColorList:
            if trueOrbitColorList[c] is not None:
                scData.trueTrajectoryLineColor = vizInterface.IntVector(trueOrbitColorList[c])

        if trueOrbitColorInMsgList:
            if trueOrbitColorInMsgList[c] is not None:
                scData.trueTrajectoryLineColorInMsg = trueOrbitColorInMsgList[c]

        # process MSM information
        if msmInfoList:
            if msmInfoList[c] is not None:  # MSM have been added to this spacecraft
                scData.msmInfo = msmInfoList[c]

        vizMessenger.scData.push_back(scData)

        c += 1

    vizMessenger.gravBodyInformation = vizInterface.GravBodyInfoVector(planetInfoList)
    vizMessenger.spiceInMsgs = messaging.SpicePlanetStateMsgInMsgsVector(spiceMsgList)

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
            print('ERROR: vizSupport: liveStream must be True or False')
            exit(1)
        vizMessenger.liveStream = val

    if 'broadcastStream' in kwargs:
        val = kwargs['broadcastStream']
        if not isinstance(val, bool):
            print('ERROR: vizSupport: broadcastStream must be True or False')
            exit(1)
        vizMessenger.broadcastStream = val

    if 'noDisplay' in kwargs:
        val = kwargs['noDisplay']
        if not isinstance(val, bool):
            print('ERROR: vizSupport: noDisplay must be True or False')
            exit(1)
        if val and (vizMessenger.liveStream or vizMessenger.broadcastStream):
            print('ERROR: vizSupport: noDisplay mode cannot be used with liveStream or broadcastStream.')
            exit(1)
        vizMessenger.noDisplay = val

    if 'opNavMode' in kwargs:
        deprecateOpNav()
        val = kwargs['opNavMode']
        if not isinstance(val, int):
            print('ERROR: vizSupport: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        if val < 0 or val > 2:
            print('ERROR: vizSupport: opNavMode must be 0 (off), 1 (regular opNav) or 2 (high performance opNav)')
            exit(1)
        if val == 1:
            vizMessenger.liveStream = True
        if val == 2:
            if (vizMessenger.liveStream or vizMessenger.broadcastStream):
                print("ERROR: vizSupport: noDisplay mode cannot be used with liveStream or broadcastStream.")
            else:
                vizMessenger.noDisplay = True


    return vizMessenger

@deprecated.deprecated("2025/03/05", "opNavMode has been deprecated. Use 'liveStream' or 'noDisplay' flags instead.")
def deprecateOpNav():
    return
