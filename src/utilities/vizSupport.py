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

import os

import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.utilities import deprecated
from Basilisk.utilities import quadMapSupport as qms
from Basilisk.utilities import unitTestSupport
from matplotlib import colors
from matplotlib.colors import is_color_like
from typing import Optional, Sequence
from Basilisk.utilities import deprecated

try:
    from Basilisk.simulation import vizInterface

    vizFound = True
except ImportError:
    vizFound = False

pauseFlag = False
endFlag = False

bskPath = __path__[0]

firstSpacecraftName = ""




def assert_option(value, low, high, default=None):
    """Check if the value is in [low, high] inclusive."""
    if value is None and default is not None:
        value = default
    if value < low or value > high:
        raise ValueError(f"Value must be between {low} and {high}, not {value}")
    return value


def assert_trinary(value, default=0):
    """Check if the value is in [-1, 0, 1] and map False to -1 or 0 depending on default."""
    if value is False:
        # Map false to which isn't the default
        value = {-1: 0, 0: -1}[default]
    if value is None:
        value = default
    if value not in [-1, 0, 1]:
        raise ValueError("Value must be -1, 0, or 1")
    return value


def toRGBA255(color, alpha=None):
    if isinstance(color, str):
        # convert color name to 4D array of values with 0-255
        if is_color_like(color):
            answer = np.array(colors.to_rgba(color, alpha=alpha)) * 255
            answer = [round(a) for a in answer]
        else:
            raise ValueError("toRGBA255() was provided unknown color name " + color)
    else:
        if not isinstance(color, list):
            raise ValueError("ERROR: vizSupport: color must be a 4D array of integers")
        if max(color) > 255 or min(color) < 0:
            raise ValueError("ERROR: vizSupport: color values must be between [0,255]")
        answer = color
    return answer


def setSprite(shape, color=None):
    """
    Helper function to set the sprite shape and optional sprite color.
    :param shape: Sprite shape, must be either "CIRCLE", "SQUARE", "TRIANGLE", "STAR", or "bskSat"
    :param kwargs: RGBA color, can be either color name string or a 4D list of [0,255] values
    :return: string of the protobuffer sprite setting
    """
    shapeList = ["CIRCLE", "SQUARE", "TRIANGLE", "STAR", "bskSat"]

    if shape not in shapeList:
        raise KeyError(
            "The setSprite() method was provided this unknown sprite shape primitive: "
            + shape
        )

    answer = shape

    if color is not None:
        if shape == "bskSat":
            raise ValueError("cannot set a color for the bskSat sprite option")

        colorValues = toRGBA255(color)

        answer += " " + " ".join(map(str, colorValues))

    return answer


def lla2fixedframe(lla_GP, radEquator, radRatio):
    """
    This method receives a latitude/longitude/altitude point above a reference
    ellipsoid with equatorial radius and flattening ratio, then converts to
    body-fixed frame coordinates.

    Parameters
    ----------
    lla_GP:
        [rad,rad,m] position vector of the location G relative to the parent body
        in lat/lon/alt components
    radEquator:
        [m] equatorial radius of the parent body
    radRatio:
        ratio of polar radius to equatorial radius

    Returns
    -------
    3-element list
        [m] r_GP_P, position vector of the location G relative to parent body frame P in P frame components

    """
    if len(lla_GP) != 3:
        raise ValueError(f"lla_GP must be a list of three floats, not {lla_GP}")
    if lla_GP[0] > np.pi / 2 or lla_GP[0] < -np.pi / 2:
        raise ValueError(
            f"Latitude must be between -pi/2 and pi/2 radians, not {lla_GP[0]}"
        )

    lat = lla_GP[0]
    lon = lla_GP[1]
    alt = lla_GP[2]
    N = radEquator / np.sqrt(1 - (1 - radRatio**2) * np.sin(lat) ** 2)

    X = (N + alt) * np.cos(lat) * np.cos(lon)
    Y = (N + alt) * np.cos(lat) * np.sin(lon)
    Z = (N * (1 - (1 - radRatio**2)) + alt) * np.sin(lat)
    r_GP_P = [X, Y, Z]

    return r_GP_P


def fixedframe2lla(r_GP_P, radEquator, radRatio):
    """
    This method receives a cartesian point above a reference ellipsoid with
    equatorial radius and flattening ratio, then converts to lat/lon/alt.

    Parameters
    ----------
    r_GP_P:
        [m] position vector of the location G relative to the parent body
    radEquator:
        [m] equatorial radius of the parent body
    radRatio:
        ratio of polar radius to equatorial radius

    Returns
    -------
    3-element list
        [rad,rad,m] lla_GP, position vector of the location G relative to parent body frame P in lat/lon/alt

    """
    X = r_GP_P[0]
    Y = r_GP_P[1]
    Z = r_GP_P[2]

    a = radEquator
    b = radEquator * radRatio

    e2 = 1 - (b / a) ** 2  # First eccentricity squared
    ep2 = (a**2 - b**2) / b**2  # Second eccentricity squared

    r = np.sqrt(X**2 + Y**2)
    lon = np.arctan2(Y, X)

    theta = np.arctan2(Z * a, r * b)
    stheta = np.sin(theta)
    ctheta = np.cos(theta)

    lat = np.arctan2(Z + ep2 * b * stheta**3, r - e2 * a * ctheta**3)

    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    h = r / np.cos(lat) - N

    lla_GP = [lat, lon, h]

    return lla_GP


locationDict = {}


def addLocation(
    viz,
    stationName: str,
    parentBodyName: str,
    r_GP_P=None,
    lla_GP=None,
    gHat_P=None,
    fieldOfView=None,
    color=None,
    range=None,
    markerScale=None,
    isHidden=None,
    label=None
):
    """
    This method creates a Location instance on a parent body.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    stationName: str
        Location text label
    parentBodyName: str
        Name of the parent body P (spacecraft or planet) on which the location G is positioned.
    r_GP_P: 3-element double-list
        Position of G relative to parent body frame P.
        Required, if lla_GP not provided
    lla_GP: 3-element double-list
        Position of G relative to parent body in lat/lon/alt coordinates.
        Required, if r_GP_P not provided
    gHat_P: 3-element double-list
        Location normal relative to parent body frame.
    fieldOfView: double
        [rad] FOV angle measured edge-to-edge.
    color: int-list
        Color of the Location.  Can be 4 RGBA integer value (0-255) or a color string.
    range: double
        [m] Range of the ground Location.
    markerScale: double
        Value will be multiplied by default marker scale, values less than 1.0 will decrease size, greater will increase.
    isHidden: bool
        True to hide Location, false to show (vizDefault)
    label: string
        string to display on location label, if empty, then stationName is used. Send "NOLABEL" to delete label
    """
    vizElement = vizInterface.LocationPbMsg()

    # Set location
    if r_GP_P is None == lla_GP is None:  # xor
        raise ValueError("Either r_GP_P or lla_GP must be provided")
    if r_GP_P is not None:
        try:
            vizElement.r_GP_P = r_GP_P
        except TypeError:
            vizElement.r_GP_P = unitTestSupport.EigenVector3d2np(r_GP_P).tolist()
    if lla_GP is not None:
        # find gravity body
        gravBody = next(
            (s for s in viz.gravBodyInformation if s.bodyName == parentBodyName), None
        )
        if gravBody is None:
            raise ValueError(f"Cannot use LLA to set location for {parentBodyName}")

        # convert lat/lon/altitude to fixed frame
        r_GP_P = lla2fixedframe(lla_GP, gravBody.radEquator, gravBody.radiusRatio)
        vizElement.r_GP_P = r_GP_P

    if gHat_P is None:
        gHat_P = r_GP_P / np.linalg.norm(r_GP_P)

    vizElement.stationName = stationName
    vizElement.parentBodyName = parentBodyName
    vizElement.gHat_P = gHat_P

    if color is not None:
        vizElement.color = toRGBA255(color)
    if range is not None:
        vizElement.range = range
    if markerScale is not None:
        if markerScale < 0.0:
            raise ValueError("markerScale must be a positive float")
        vizElement.markerScale = markerScale
    if isHidden is not None:
        vizElement.isHidden = isHidden
    if fieldOfView is not None:
        if fieldOfView > np.pi or fieldOfView < 0.0:
            raise ValueError(
                f"fieldOfView must be a value between 0 and Pi, not {fieldOfView}"
            )
        vizElement.fieldOfView = fieldOfView
    if label is not None:
        vizElement.label = label

    # Pass to Vizard
    locationDict[vizElement.stationName] = vizElement
    viz.locations.append(vizElement)

    return


def changeLocation(
        viz,
        stationName: str,
        parentBodyName: Optional[str] = None,
        r_GP_P: Optional[Sequence[float]] = None,
        lla_GP: Optional[Sequence[float]] = None,
        gHat_P: Optional[Sequence[float]] = None,
        fieldOfView: Optional[Sequence[float]] = None,
        color: Optional[Sequence[float]] = None,
        range: Optional[float] = None,
        markerScale: Optional[float] = None,
        isHidden: Optional[bool] = None,
        label: Optional[str] = None
):
    """
    This method changes the information of a Location instance.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    stationName: str
        Location text label
        Required
    parentBodyName: str
        Name of the parent body P (spacecraft or planet) on which the location G is positioned.
    r_GP_P: 3-element double-list
        Position of G relative to parent body frame P.
        Required, if lla_GP not provided
    lla_GP: 3-element double-list
        Position of G relative to parent body in lat/lon/alt coordinates.
        Required, if r_GP_P not provided
    gHat_P: 3-element double-list
        Location normal relative to parent body frame.
    fieldOfView: double
        [rad] FOV angle measured edge-to-edge.
    color: int-list
        Color of the Location.  Can be 4 RGBA integer value (0-255) or a color string.
    range: double
        [m] Range of the ground Location.
    markerScale: double
        Value will be multiplied by default marker scale, values less than 1.0 will decrease size, greater will increase.
    isHidden: bool
        True to hide Location, false to show (vizDefault)
    label: string
        string to display on location label, if empty, then stationName is used. Send "NOLABEL" to delete label
    """

    vizElement = locationDict[stationName]

    # Set location
    if r_GP_P is not None:
        try:
            vizElement.r_GP_P = r_GP_P
        except TypeError:
            vizElement.r_GP_P = unitTestSupport.EigenVector3d2np(r_GP_P).tolist()
    if lla_GP is not None:
        # find gravity body
        gravBody = next(
            (s for s in viz.gravBodyInformation if s.bodyName == parentBodyName), None
        )
        if gravBody is None:
            raise ValueError(f"Cannot use LLA to set location for {parentBodyName}")

        # convert lat/lon/altitude to fixed frame
        r_GP_P = lla2fixedframe(lla_GP, gravBody.radEquator, gravBody.radiusRatio)
        vizElement.r_GP_P = r_GP_P

    if parentBodyName is not None:
        vizElement.parentBodyName = parentBodyName
    if gHat_P is not None:
        vizElement.gHat_P = gHat_P

    if color is not None:
        vizElement.color = toRGBA255(color)
    if range is not None:
        vizElement.range = range
    if markerScale is not None:
        if markerScale < 0.0:
            raise ValueError("markerScale must be a positive float")
        vizElement.markerScale = markerScale
    if isHidden is not None:
        vizElement.isHidden = isHidden
    if fieldOfView is not None:
        if fieldOfView > np.pi or fieldOfView < 0.0:
            raise ValueError(
                f"fieldOfView must be a value between 0 and Pi, not {fieldOfView}"
            )
        vizElement.fieldOfView = fieldOfView

    # add this location structure to the vector of locations to be transmitted to Vizard
    locationDict[stationName] = vizElement
    viz.locations.append(vizElement)


quadMapList = []


def addQuadMap(viz, ID, parentBodyName, vertices, color, isHidden=None, label=None):
    """
    This method creates a QuadMap element for displaying shaded regions in Vizard.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    ID: int
        The reference ID of the QuadMap instance. Once instantiated, can be used to change the QuadMap settings.
    parentBodyName: str
        Name of the parent body to draw the QuadMap in reference to.
    vertices: single or double-list
        Specifies the internal mesh coordinates of the QuadMap, in body-fixed frame.
    color: int list
        Color of the QuadMap.  Can be 4 RGBA integer value (0-255) or a color string.
    isHidden: bool
        Flag if the QuadMap should be hidden (1) or shown (-1).
        Optional. Default: 0 - if not provided, then the Vizard default settings are used.
    label: str
        Label to display in the center of QuadMap region.
        Optional. Send "NOLABEL" to delete label.
    """
    vizElement = vizInterface.QuadMap()

    vizElement.ID = ID
    vizElement.parentBodyName = parentBodyName
    vizElement.vertices = vizInterface.DoubleVector(vertices)
    vizElement.color = vizInterface.IntVector(toRGBA255(color))

    if isHidden is not None:
        vizElement.isHidden = isHidden
    if label is not None:
        vizElement.label = label

    quadMapList.append(vizElement)
    del viz.quadMaps[:]  # clear settings list to replace it with updated list
    viz.quadMaps = vizInterface.QuadMapVector(quadMapList)

    return


pointLineList = []


def createPointLine(viz, toBodyName, lineColor, fromBodyName=None):
    """
    This method creates a PointLine between two bodies.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    toBodyName: str
        Body which the PointLine points to.
    lineColor: int list
        Color of the PointLine.  Can be 4 RGBA integer value (0-255) or a color string.
    fromBodyName: str
        Body from which PointLine originates.
        Optional, default selects ``firstSpacecraftName``
    """
    global firstSpacecraftName
    vizElement = vizInterface.PointLine()

    if fromBodyName is None:
        fromBodyName = firstSpacecraftName

    vizElement.fromBodyName = fromBodyName
    vizElement.toBodyName = toBodyName
    vizElement.lineColor = toRGBA255(lineColor)

    pointLineList.append(vizElement)
    # clear settings list to replace it with updated list
    del viz.settings.pointLineList[:]
    viz.settings.pointLineList = vizInterface.PointLineConfig(pointLineList)
    return


targetLineList = []


def createTargetLine(viz, toBodyName, lineColor, fromBodyName=None):
    """
    This method creates a TargetLine between two bodies.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    toBodyName: str
        Body which the PointLine points to.
    lineColor: int list
        Color of the PointLine.  Can be 4 RGBA integer value (0-255) or a color string.
    fromBodyName: str
        Body from which PointLine originates.
        Optional, default selects ``firstSpacecraftName``
    """
    global firstSpacecraftName
    vizElement = vizInterface.PointLine()

    if fromBodyName is None:
        fromBodyName = firstSpacecraftName

    vizElement.fromBodyName = fromBodyName
    vizElement.toBodyName = toBodyName
    vizElement.lineColor = toRGBA255(lineColor)

    targetLineList.append(vizElement)
    updateTargetLineList(viz)
    return


def updateTargetLineList(viz):
    # clear settings list to replace it with updated list
    del viz.liveSettings.targetLineList[:]
    viz.liveSettings.targetLineList = vizInterface.PointLineConfig(targetLineList)
    return


customModelList = []


def createCustomModel(
    viz,
    modelPath,
    simBodiesToModify=None,
    offset=None,
    rotation=None,
    scale=None,
    customTexturePath="",
    normalMapPath="",
    shader=-1,
    color=None,
):
    """
    This method creates a CustomModel.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    modelPath: str
        Path to model obj -OR- ``CUBE``, ``CYLINDER``, or ``SPHERE`` to use a primitive shape
    simBodiesToModify: list
        Which bodies in scene to replace with this model, use ``ALL_SPACECRAFT`` to apply custom model to all spacecraft in simulation
        Optional, default modifies ``firstSpacecraftName``
    offset: 3-element double-list
        [m] Offset to use to draw the model
        Optional, default is [0.0, 0.0, 0.0]
    rotation: 3-element double-list
        [rad] 3-2-1 Euler angles to rotate CAD about z, y, x axes
        Optional, default is [0.0, 0.0, 0.0]
    scale: 3-element double-list
        Desired model scaling factor along the body x, y, z, axes in spacecraft CS
        Optional, default is [1.0, 1.0, 1.0]
    customTexturePath: str
        Path to texture to apply to model (note that a custom model's .mtl will be automatically imported with its textures during custom model import)
        Optional
    normalMapPath: str
        Path to the normal map for the customTexture
    shader: int
        Value of -1 to use viz default, 0 for Unity Specular Standard Shader, 1 for Unity Standard Shader
    color: int list
        Send desired RGBA as values between 0 and 255, default is gray, and will be applied to the albedo color setting
    """
    global firstSpacecraftName
    vizElement = vizInterface.CustomModel()

    if offset is None:
        offset = [0.0, 0.0, 0.0]
    if rotation is None:
        rotation = [0.0, 0.0, 0.0]
    if scale is None:
        scale = [1.0, 1.0, 1.0]
    if simBodiesToModify is None:
        simBodiesToModify = [firstSpacecraftName]

    vizElement.modelPath = modelPath
    vizElement.offset = offset
    vizElement.rotation = rotation
    vizElement.scale = scale
    vizElement.customTexturePath = customTexturePath
    vizElement.normalMapPath = normalMapPath
    vizElement.shader = assert_option(shader, low=-1, high=1)
    if len(simBodiesToModify) == 0:
        raise ValueError("simBodiesToModify must be a non-empty list of strings")
    vizElement.simBodiesToModify = vizInterface.StringVector(simBodiesToModify)

    if color is not None:
        vizElement.color = vizInterface.IntVector(color)

    customModelList.append(vizElement)
    # clear settings list to replace it with updated list
    del viz.settings.customModelList[:]
    viz.settings.customModelList = vizInterface.CustomModelConfig(customModelList)
    return


actuatorGuiSettingList = []


def setActuatorGuiSetting(
    viz,
    spacecraftName=None,
    viewThrusterPanel=None,
    viewThrusterHUD=None,
    viewRWPanel=None,
    viewRWHUD=None,
    showThrusterLabels=None,
    showRWLabels=None,
):
    """
    This method sets the actuator GUI properties for a particular spacecraft.  If no ``spacecraftName`` is
    provided, then the name of the first spacecraft in the simulation is assumed.

    :param viz: copy of the vizInterface module
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
    global firstSpacecraftName
    vizElement = vizInterface.ActuatorGuiSettings()

    if spacecraftName is None:
        spacecraftName = firstSpacecraftName

    vizElement.spacecraftName = spacecraftName

    if viewThrusterPanel is not None:
        vizElement.viewThrusterPanel = viewThrusterPanel
    if viewThrusterHUD is not None:
        vizElement.viewThrusterHUD = viewThrusterHUD
    if viewRWPanel is not None:
        vizElement.viewRWPanel = viewRWPanel
    if viewRWHUD is not None:
        vizElement.viewRWHUD = viewRWHUD
    if showThrusterLabels is not None:
        vizElement.showThrusterLabels = showThrusterLabels
    if showRWLabels is not None:
        vizElement.showRWLabels = showRWLabels

    actuatorGuiSettingList.append(vizElement)
    # clear settings list to replace it with updated list
    del viz.settings.actuatorGuiSettingsList[:]
    viz.settings.actuatorGuiSettingsList = vizInterface.ActuatorGuiSettingsConfig(
        actuatorGuiSettingList
    )
    return


instrumentGuiSettingList = []


def setInstrumentGuiSetting(
    viz,
    spacecraftName=None,
    viewCSSPanel=0,
    viewCSSCoverage=0,
    viewCSSBoresight=0,
    showCSSLabels=0,
    showGenericSensorLabels=0,
    showTransceiverLabels=0,
    showTransceiverFrustum=0,
    showTransceiverFrustrum=0,
    showGenericStoragePanel=0,
    showMultiShapeLabels=0,
):
    """
    This method sets the instrument GUI properties for a particular spacecraft.  If no ``spacecraftName`` is
    provided, then the name of the first spacecraft in the simulation is assumed.

    :param viz: copy of the vizInterface module
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
    showTransceiverFrustum: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showGenericStoragePanel: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    showMultiShapeLabels: int
        flag if the generic sensor labels should be shown (1) or hidden (-1)
        Default: 0 - if not provided, then the Vizard default settings are used
    """
    global firstSpacecraftName
    vizElement = vizInterface.InstrumentGuiSettings()

    if spacecraftName is None:
        spacecraftName = firstSpacecraftName

    vizElement.spacecraftName = spacecraftName
    vizElement.viewCSSPanel = assert_trinary(viewCSSPanel, default=0)
    vizElement.viewCSSCoverage = assert_trinary(viewCSSCoverage, default=0)
    vizElement.viewCSSBoresight = assert_trinary(viewCSSBoresight, default=0)
    vizElement.showCSSLabels = assert_trinary(showCSSLabels, default=0)
    vizElement.showGenericSensorLabels = assert_trinary(
        showGenericSensorLabels, default=0
    )
    vizElement.showTransceiverLabels = assert_trinary(showTransceiverLabels, default=0)
    vizElement.showTransceiverFrustum = assert_trinary(
        showTransceiverFrustum, default=0
    )
    if showTransceiverFrustrum:
        transceiverFrustrum(vizElement, showTransceiverFrustrum)
    vizElement.showGenericStoragePanel = assert_trinary(
        showGenericStoragePanel, default=0
    )
    vizElement.showMultiShapeLabels = assert_trinary(showMultiShapeLabels, default=0)

    instrumentGuiSettingList.append(vizElement)
    # clear settings list to replace it with updated list
    del viz.settings.instrumentGuiSettingsList[:]
    viz.settings.instrumentGuiSettingsList = vizInterface.InstrumentGuiSettingsConfig(
        instrumentGuiSettingList
    )
    return

@deprecated.deprecated("2026/10/11", "Use showTransceiverFrustum instead of showTransceiverFrustrum")
def transceiverFrustrum(vizElement, showTransceiverFrustrum):
    vizElement.showTransceiverFrustum = assert_trinary(
        showTransceiverFrustrum, default=0
    )

coneInOutList = []


def createConeInOut(
    viz,
    toBodyName,
    coneColor,
    isKeepIn,
    normalVector_B,
    incidenceAngle,
    coneHeight,
    fromBodyName=None,
    position_B=None,
    coneName="",
):
    """
    This method creates a ``KeepOutInCone``.

    :param viz: copy of the vizInterface module
    :return: void

    Keyword Args
    ------------
    fromBodyName: str
        Name of body to attach cone onto.
        Optional, default selects ``firstSpacecraftName``
    toBodyName: str
        Detect changes if this body has impingement on cone.
    coneColor: int list
        Color of the KeepOutInCone.  Can be 4 RGBA integer value (0-255) or a color string.
    isKeepIn: bool
        True -> keep-in cone created, False -> keep-out cone created
    position_B: 3-element double-list
        [m] Cone start relative to from-body coordinate frame.
        Optional, default [0.0, 0.0, 0.0]
    normalVector_B: 3-element double-list
        Cone normal direction vector
    incidenceAngle: double
        [rad] Cone incidence angle
    coneHeight: double
        [m] Sets height of visible cone (aesthetic only, does not impact function)
    coneName: str
        Cone name, if unspecified, viz will autogenerate name
    """
    global firstSpacecraftName
    vizElement = vizInterface.KeepOutInCone()

    if fromBodyName is None:
        fromBodyName = firstSpacecraftName
    if position_B is None:
        position_B = [0.0, 0.0, 0.0]

    vizElement.fromBodyName = fromBodyName
    vizElement.toBodyName = toBodyName
    vizElement.coneColor = toRGBA255(coneColor)
    vizElement.isKeepIn = isKeepIn
    vizElement.position_B = position_B
    vizElement.normalVector_B = normalVector_B
    vizElement.incidenceAngle = incidenceAngle
    vizElement.coneHeight = coneHeight
    vizElement.coneName = coneName

    coneInOutList.append(vizElement)
    del viz.settings.coneList[:]  # clear settings list to replace it with updated list
    viz.settings.coneList = vizInterface.KeepOutInConeConfig(coneInOutList)
    return


stdCameraList = []


def createStandardCamera(
    viz,
    spacecraftName=None,
    setMode=None,
    showHUDElementsInImage=None,
    setView=None,
    fieldOfView=-1,
    bodyTarget="",
    pointingVector_B=None,
    position_B=None,
    displayName=None,
):
    """
    This method creates a Standard Camera.

    :param viz: copy of the vizInterface module
    :return: camera instance

    Keyword Args
    ------------
    spacecraftName: str
        Name of spacecraft to attach camera onto.
        Optional, default selects ``firstSpacecraftName``
    setMode: int
        0 -> body targeting, 1 -> pointing vector (default).
    showHUDElementsInImage: int
        Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    setView: int
        0 -> nadir (default), 1 -> orbit normal, 2 -> along track. This is a setting for body targeting mode.
    fieldOfView: double
        [rad] FOV angle measured edge-to-edge, -1 to use viz default.
    bodyTarget: str
        Name of body camera should point to (default to first celestial body in messages). This is a setting for body targeting mode.
    pointingVector_B: 3-element double-list
        Camera pointing vector in the spacecraft body frame.
        Optional, default [1.0, 0.0, 0.0]
    position_B: 3-element double-list
        If a non-zero vector, this determines the location of the camera.  If a zero vector, then the camera is placed outside the spacecraft along the pointing vector direction.
        Optional, default [0.0, 0.0, 0.0]
    displayName: str
        Name of the standard camera panel.
        Optional
    """
    global firstSpacecraftName
    cam = vizInterface.StdCameraSettings()

    if spacecraftName is None:
        spacecraftName = firstSpacecraftName
    if pointingVector_B is None:
        pointingVector_B = [1.0, 0.0, 0.0]
    if position_B is None:
        position_B = [0.0, 0.0, 0.0]

    cam.spacecraftName = spacecraftName
    cam.showHUDElementsInImage = assert_trinary(showHUDElementsInImage, default=0)
    cam.fieldOfView = fieldOfView
    cam.bodyTarget = bodyTarget
    cam.pointingVector_B = pointingVector_B
    cam.position_B = position_B

    if setMode is not None:
        cam.setMode = assert_option(setMode, low=0, high=1)
    if setView is not None:
        cam.setView = assert_option(setView, low=0, high=2)
    if displayName is not None:
        cam.displayName = displayName

    stdCameraList.append(cam)
    # clear settings list to replace it with updated list
    del viz.settings.stdCameraList[:]
    viz.settings.stdCameraList = vizInterface.StdCameraConfig(stdCameraList)
    return cam


def createCameraConfigMsg(
    viz,
    cameraID,
    fieldOfView,
    resolution,
    cameraPos_B,
    sigma_CB,
    parentName=None,
    renderRate=None,
    skyBox="",
    postProcessingOn=0,
    ppFocusDistance=None,
    ppAperature=None,
    ppFocalLength=None,
    ppMaxBlurSize=None,
    updateCameraParameters=False,
    renderMode=False,
    depthMapClippingPlanes=None,
    showHUDElementsInImage=None,
):
    """
    This method configures camera settings.

    :param viz: copy of the vizInterface module
    :return: camera instance

    Keyword Args
    ------------
    cameraID: int
        ID of the camera that took the snapshot.
    parentName: str
        Name of the parent body to which the camera should be attached
        Optional, default is ``firstSpacecraftName``
    fieldOfView: double
        [rad] Camera FOV, edge-to-edge along camera y-axis.
    resolution: 2-element int-list
        Camera resolution, width/height in pixels.
    renderRate: int
        [ns] Frame time interval at which to capture images.
    cameraPos_B: 3-element double-list
        [m] Camera position in body frame.
    sigma_CB: 3-element double-list
        MRP defining the orientation of the camera frame relative to the body frame.
    skyBox: str
        String containing the star field preference.
    postProcessingOn: int
        Enable post-processing of camera image. Value of 0 (protobuffer default) to use viz default which is off, -1 for false, 1 for true.
    ppFocusDistance: double
        Distance to the point of focus, minimum value of 0.1, Value of 0 to turn off this parameter entirely.
    ppAperature: double
        Ratio of the aperture (known as f-stop or f-number). The smaller the value is, the shallower the depth of field is. Valid Setting Range: 0.05 to 32. Value of 0 to turn off this parameter entirely.
    ppFocalLength: double
        Valid setting range: 0.001m to 0.3m. Value of 0 to turn off this parameter entirely.
    ppMaxBlurSize: int
        Convolution kernel size of the bokeh filter, which determines the maximum radius of bokeh. It also affects the performance (the larger the kernel is, the longer the GPU time is required). Depth textures Value of 1 for Small, 2 for Medium, 3 for Large, 4 for Extra Large. Value of 0 to turn off this parameter entirely.
    updateCameraParameters: int
        If true, commands camera to update Instrument Camera to current message's parameters.
    renderMode: int
        Value of 0 to render visual image (default), value of 1 to render depth buffer to image.
    depthMapClippingPlanes: 2-element double-list
        [m] Set the bounds of rendered depth map by setting the near and far clipping planes when in renderMode=1 (depthMap mode). Default values of 0.1 and 100.
    showHUDElementsInImage: int
        Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    """
    global firstSpacecraftName
    cameraConfigMsgPayload = messaging.CameraConfigMsgPayload()

    if parentName is None:
        parentName = firstSpacecraftName

    cameraConfigMsgPayload.cameraID = cameraID
    cameraConfigMsgPayload.parentName = parentName
    cameraConfigMsgPayload.fieldOfView = fieldOfView
    cameraConfigMsgPayload.resolution = resolution
    cameraConfigMsgPayload.cameraPos_B = cameraPos_B
    cameraConfigMsgPayload.sigma_CB = sigma_CB
    cameraConfigMsgPayload.skyBox = skyBox
    cameraConfigMsgPayload.postProcessingOn = assert_trinary(
        postProcessingOn, default=0
    )
    cameraConfigMsgPayload.updateCameraParameters = int(updateCameraParameters)
    cameraConfigMsgPayload.renderMode = int(renderMode)
    cameraConfigMsgPayload.showHUDElementsInImage = assert_trinary(
        showHUDElementsInImage, default=0
    )

    if renderRate is not None:  # convert to nano-seconds
        cameraConfigMsgPayload.renderRate = int(renderRate * 1e9)
    if ppFocusDistance is not None:
        if ppFocusDistance != 0 and ppFocusDistance < 0.1:
            raise ValueError("ppFocusDistance must be 0 or greater than 0.1")
        cameraConfigMsgPayload.ppFocusDistance = ppFocusDistance
    if ppAperature is not None:
        if ppAperature != 0 and (ppAperature < 0.05 or ppAperature > 32):
            raise ValueError("ppAperature must be 0 or within [0.05, 32]")
        cameraConfigMsgPayload.ppAperture = ppAperature
    if ppFocalLength is not None:
        if ppFocalLength != 0 and (ppFocalLength < 0.001 or ppFocalLength > 0.3):
            raise ValueError("ppFocalLength must be 0 or within [0.001, 0.3]")
        cameraConfigMsgPayload.ppFocalLength = ppFocalLength
    if ppMaxBlurSize is not None:
        cameraConfigMsgPayload.ppMaxBlurSize = assert_option(
            ppMaxBlurSize, low=0, high=4
        )
    if depthMapClippingPlanes is not None:
        if not cameraConfigMsgPayload.renderMode:
            raise ValueError(
                "depthMapClippingPlanes can only be set when renderMode is 1 (depthMap)."
            )
        cameraConfigMsgPayload.depthMapClippingPlanes = depthMapClippingPlanes
    else:
        cameraConfigMsgPayload.depthMapClippingPlanes = [-1.0, -1.0]

    cameraConfigMsg = messaging.CameraConfigMsg().write(cameraConfigMsgPayload)
    # need to add code to retain camera config msg in memory.  Below
    # the function makes vizInterface subscribe to the pointer to this Msg object
    viz.addCamMsgToModule(cameraConfigMsg)

    return cameraConfigMsgPayload, cameraConfigMsg


def ensure_correct_len_list(input, length, depth=1):
    # Allow lists of all None to pass through as long as they are the correct length
    if (
        isinstance(input, list)
        and all([i is None for i in input])
        and len(input) == length
    ):
        return input

    current_depth = 0
    level = input
    while isinstance(level, list):
        current_depth += 1
        # Skip over Nones when checking shapes at a given level
        level = next((item for item in level if item is not None), None)

    for _ in range(current_depth, depth):
        input = [input]

    if len(input) != length:
        raise ValueError(f"List length should be {length}, not {len(input)}")

    return input


def enableUnityVisualization(
    scSim,
    simTaskName,
    scList,
    saveFile=None,
    rwEffectorList=None,
    thrEffectorList=None,
    thrColors=None,
    cssList=None,
    genericSensorList=None,
    ellipsoidList=None,
    lightList=None,
    genericStorageList=None,
    transceiverList=None,
    spriteList=None,
    modelDictionaryKeyList=None,
    logoTextureList=None,
    oscOrbitColorList=None,
    trueOrbitColorList=None,
    groundTrackColorList=None,
    msmInfoList=None,
    trueOrbitColorInMsgList=None,
    groundTrackBodyNameList=None,
    liveStream=False,
    broadcastStream=False,
    noDisplay=False,
):
    """
    This method creates an instance of the vizInterface() modules and sets up associated Vizard
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
        can be a single python file name, or a full path + file name. In both cases a local results are stored
        in a local sub-folder called ``_VizFiles``.
        If a data file name is provided directly (i.e. it ends with ``.bin``), then the
        associated file path and name are used explicitly.
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
    ellipsoidList:
        list of lists of ``Ellipsoid`` structures.  The outer list length must match ``scList``.
    lightList:
        list of lists of ``Light`` structures.   The outer list length must match ``scList``.
    genericStorageList:
        list of lists of ``GenericStorage`` structures.  The outer list length must match ``scList``.
    transceiverList:
        list of lists of ``Transceiver`` objects.  The outer list length must match ``scList``.
    spriteList:
        list of sprite information for each spacecraft.  The outer list length must match ``scList``.
    modelDictionaryKeyList:
        list of the spacecraft model dictionary.  The outer list length must match ``scList``.
    logoTextureList:
        list of the spacecraft logo texture file paths.  The outer list length must match ``scList``.
    oscOrbitColorList:
        list of spacecraft osculating orbit colors.  Can be 4 RGBA integer value (0-255) or
        ``None`` if default values should be used.  The array must be of the length of the spacecraft list
    trueOrbitColorList:
        list of spacecraft true or actual orbit colors.  Can be 4 RGBA integer value (0-255) or
        ``None`` if default values should be used.  The array must be of the length of the spacecraft list
    trueOrbitColorInMsgList:
        list of color messages to read and provide the true orbit color at each time step.  This overwrites
        the values set with trueOrbitColorList.
    groundTrackColorList:
        list of spacecraft ground track colors.  Can be 4 RGBA integer value (0-255) or
        ``None`` if default values should be used.  The array must be of the length of the spacecraft list
    groundTrackBodyNameList:
        list of celestial bodies relative to which to draw ground track on.
        If None the ground track will default to spacecraft's dominant grav body
    msmInfoList:
        list of MSM configuration messages

    liveStream: bool
        flag if live data streaming to Vizard should be used
    broadcastStream: bool
        flag if messages should be broadcast for listener Vizards to pick up.
    noDisplay: bool
        flag if Vizard should run performance opNav (no Vizard display)

    Returns
    -------
    :ref:`vizInterface` object
        copy of the vizInterface instance

    """
    if not vizFound:
        print("BSK is built without vizInterface support.")
        return

    # clear the list of point line elements
    del pointLineList[:]
    del actuatorGuiSettingList[:]
    del coneInOutList[:]
    global firstSpacecraftName

    # set up the Vizard interface module
    scSim.vizMessenger = vizInterface.VizInterface()
    scSim.vizMessenger.settings = vizInterface.VizSettings()
    scSim.vizMessenger.ModelTag = "vizMessenger"
    scSim.AddModelToTask(simTaskName, scSim.vizMessenger)

    # ensure the spacecraft object list is a list
    if not isinstance(scList, list):
        scList = [scList]
    scListLength = len(scList)

    firstSpacecraftName = scList[0].ModelTag

    if rwEffectorList is not None:
        rwEffectorList = ensure_correct_len_list(rwEffectorList, scListLength)
    if thrEffectorList is not None:
        thrEffectorList = ensure_correct_len_list(
            thrEffectorList, scListLength, depth=2
        )
    if thrColors is not None:
        thrColors = ensure_correct_len_list(thrColors, scListLength, depth=3)
    if cssList is not None:
        cssList = ensure_correct_len_list(cssList, scListLength, depth=2)
    if genericSensorList is not None:
        genericSensorList = ensure_correct_len_list(
            genericSensorList, scListLength, depth=2
        )
    if ellipsoidList is not None:
        ellipsoidList = ensure_correct_len_list(ellipsoidList, scListLength, depth=2)
    if lightList is not None:
        lightList = ensure_correct_len_list(lightList, scListLength, depth=2)
    if genericStorageList is not None:
        genericStorageList = ensure_correct_len_list(
            genericStorageList, scListLength, depth=2
        )
    if transceiverList is not None:
        transceiverList = ensure_correct_len_list(
            transceiverList, scListLength, depth=2
        )
    if spriteList is not None:
        spriteList = ensure_correct_len_list(spriteList, scListLength)
    if modelDictionaryKeyList is not None:
        modelDictionaryKeyList = ensure_correct_len_list(
            modelDictionaryKeyList, scListLength
        )
    if logoTextureList is not None:
        logoTextureList = ensure_correct_len_list(logoTextureList, scListLength)
    if oscOrbitColorList is not None:
        oscOrbitColorList = ensure_correct_len_list(
            oscOrbitColorList, scListLength, depth=2
        )
    if trueOrbitColorList is not None:
        trueOrbitColorList = ensure_correct_len_list(
            trueOrbitColorList, scListLength, depth=2
        )
    if trueOrbitColorInMsgList is not None:
        trueOrbitColorInMsgList = ensure_correct_len_list(
            trueOrbitColorInMsgList, scListLength
        )
    if groundTrackColorList is not None:
        groundTrackColorList = ensure_correct_len_list(
            groundTrackColorList,scListLength, depth=2
        )
    if groundTrackBodyNameList is not None:
        groundTrackBodyNameList = ensure_correct_len_list(
            groundTrackBodyNameList, scListLength, depth=1
        )
    if msmInfoList is not None:
        msmInfoList = ensure_correct_len_list(msmInfoList, scListLength)

    # loop over all spacecraft to associated states and msg information
    planetNameList = []
    planetInfoList = []
    spiceMsgList = []
    scSim.vizMessenger.scData.clear()
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
            bodies = list(getattr(getattr(sc, "gravField", None), "gravBodies", []))
            if bodies:  # only runs if gravField exists and has bodies
                # get the existing dict of kept wrappers, or start fresh
                kept = getattr(scSim, "_kept_grav_bodies", {})

                for gravBody in bodies:
                    # use planetName as a unique key; you could also key by id(gravBody) if needed
                    if gravBody.planetName not in kept:
                        kept[gravBody.planetName] = gravBody

                        planetNameList.append(gravBody.planetName)
                        planetInfo = vizInterface.GravBodyInfo()
                        planetInfo.bodyName = getattr(gravBody, "displayName", "") or gravBody.planetName
                        planetInfo.mu = gravBody.mu
                        planetInfo.radEquator = gravBody.radEquator
                        planetInfo.radiusRatio = gravBody.radiusRatio
                        planetInfo.modelDictionaryKey = gravBody.modelDictionaryKey
                        planetInfoList.append(planetInfo)
                        spiceMsgList.append(gravBody.planetBodyInMsg)

                # update the dict back onto scSim
                scSim._kept_grav_bodies = kept
        else:
            # the scList object is an effector belonging to the parent spacecraft
            scData.parentSpacecraftName = spacecraftParentName
            ModelTag = sc[0]
            effStateOutMsg = sc[1]
            scData.spacecraftName = ModelTag
            scData.scStateInMsg.subscribeTo(effStateOutMsg)

        # process RW effectors
        if rwEffectorList:
            rwList = []
            if rwEffectorList[c] is not None:
                # RWs have been added to this spacecraft
                for rwLogMsg in rwEffectorList[c].rwOutMsgs:
                    rwList.append(rwLogMsg.addSubscriber())
            scData.rwInMsgs = messaging.RWConfigLogMsgInMsgsVector(rwList)

        # process THR effectors
        if thrEffectorList:
            thrList = []
            thrInfo = []
            if (
                thrEffectorList[c] is not None
            ):  # THR clusters have been added to this spacecraft
                clusterCounter = 0
                for thrEff in thrEffectorList[
                    c
                ]:  # loop over the THR effectors attached to this spacecraft
                    thSet = vizInterface.ThrClusterMap()
                    thSet.thrTag = (
                        thrEff.ModelTag
                    )  # set the label for this cluster of THR devices
                    if thrColors:
                        if thrColors[c] is not None:
                            thSet.color = thrColors[c][clusterCounter]
                    for thrLogMsg in (
                        thrEff.thrusterOutMsgs
                    ):  # loop over the THR cluster log message
                        thrList.append(thrLogMsg.addSubscriber())
                        thrInfo.append(thSet)
                    clusterCounter += 1
            scData.thrInMsgs = messaging.THROutputMsgInMsgsVector(thrList)
            scData.thrInfo = vizInterface.ThrClusterVector(thrInfo)

        # process CSS information
        if cssList:
            cssDeviceList = []
            if cssList[c] is not None:  # CSS list has been added to this spacecraft
                for css in cssList[c]:
                    cssDeviceList.append(css.cssConfigLogOutMsg.addSubscriber())
                scData.cssInMsgs = messaging.CSSConfigLogMsgInMsgsVector(cssDeviceList)

        # process generic sensor HUD information
        if genericSensorList:
            gsList = []
            if (
                genericSensorList[c] is not None
            ):  # generic sensor(s) have been added to this spacecraft
                for gs in genericSensorList[c]:
                    gsList.append(gs)
                scData.genericSensorList = vizInterface.GenericSensorVector(gsList)

        # process spacecraft ellipsoids
        if ellipsoidList:
            elList = []
            if (
                ellipsoidList[c] is not None
            ):  # generic sensor(s) have been added to this spacecraft
                for el in ellipsoidList[c]:
                    elList.append(el)
                scData.ellipsoidList = vizInterface.EllipsoidVector(elList)

        # process spacecraft lights
        if lightList:
            liList = []
            if (
                lightList[c] is not None
            ):  # light objects(s) have been added to this spacecraft
                for li in lightList[c]:
                    liList.append(li)
                scData.lightList = vizInterface.LightVector(liList)

        # process generic storage HUD information
        if genericStorageList:
            gsdList = []
            if (
                genericStorageList[c] is not None
            ):  # generic storage device(s) have been added to this spacecraft
                for gsd in genericStorageList[c]:
                    if len(gsd.color) > 1:
                        if len(gsd.color) / 4 != len(gsd.thresholds) + 1:
                            print(
                                "ERROR: vizSupport: generic storage "
                                + gsd.label
                                + " threshold list does not have the correct dimension.  "
                                "It should be 1 smaller than the list of colors."
                            )
                            exit(1)
                    else:
                        if len(gsd.thresholds) > 0:
                            print(
                                "ERROR: vizSupport: generic storage "
                                + gsd.label
                                + " threshold list is set, but no multiple of colors are provided."
                            )
                            exit(1)
                    gsdList.append(gsd)
                scData.genericStorageList = vizInterface.GenericStorageVector(gsdList)

        # process transceiver HUD information
        if transceiverList:
            tcList = []
            if (
                transceiverList[c] is not None
            ):  # transceiver(s) have been added to this spacecraft
                for tc in transceiverList[c]:
                    tcList.append(tc)
                scData.transceiverList = vizInterface.TransceiverVector(tcList)

        # process sprite information
        if spriteList:
            if spriteList[c] is not None:
                scData.spacecraftSprite = spriteList[c]
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
                scData.trueTrajectoryLineColor = vizInterface.IntVector(
                    trueOrbitColorList[c]
                )

        if trueOrbitColorInMsgList:
            if trueOrbitColorInMsgList[c] is not None:
                scData.trueTrajectoryLineColorInMsg = trueOrbitColorInMsgList[c]

        if groundTrackColorList:
            if groundTrackColorList[c] is not None:
                scData.groundTrackLineColor = vizInterface.IntVector(groundTrackColorList[c])

        if groundTrackBodyNameList:
            if groundTrackBodyNameList[c] is not None:
                scData.groundTrackBodyName = groundTrackBodyNameList[c]

        # process MSM information
        if msmInfoList:
            if msmInfoList[c] is not None:  # MSM have been added to this spacecraft
                scData.msmInfo = msmInfoList[c]

        scSim.vizMessenger.scData.push_back(scData)

        c += 1

    scSim.vizMessenger.gravBodyInformation = vizInterface.GravBodyInfoVector(planetInfoList)
    scSim.vizMessenger.spiceInMsgs = messaging.SpicePlanetStateMsgInMsgsVector(spiceMsgList)

    # note that the following logic can receive a single python file name, or a full path + file name.
    # In both cases a local results are stored in a local sub-folder.
    # If a "*.bin" file is provided, then the provided path and name are used to store the data.
    scSim.vizMessenger.saveFile = False
    if saveFile is not None:
        if os.path.splitext(os.path.basename(saveFile))[1].lower() == ".bin":
            # here the provide file path, file name and file extension are used explicitly
            vizFileNamePath = saveFile
        else:
            # here the file path and name string are split into file path and a file name
            # next, the `_VizFiles` folder is created, if needed, and the binary data file
            # has the name of the provided file name with `_UnityViz.bin` appended
            fileName = os.path.splitext(os.path.basename(saveFile))[0]
            filePath = os.path.dirname(saveFile)
            if filePath == "":
                filePath = "."
            if not os.path.isdir(filePath + "/_VizFiles"):
                os.mkdir(filePath + "/_VizFiles")
            vizFileNamePath = filePath + "/_VizFiles/" + fileName + "_UnityViz.bin"
        scSim.vizMessenger.saveFile = True
        scSim.vizMessenger.protoFilename = vizFileNamePath

    if (liveStream or broadcastStream) and noDisplay:
        raise ValueError(
            "noDisplay mode cannot be used with liveStream or broadcastStream."
        )
    scSim.vizMessenger.liveStream = liveStream
    scSim.vizMessenger.broadcastStream = broadcastStream
    scSim.vizMessenger.noDisplay = noDisplay

    return scSim.vizMessenger
