//
//  vizStructures.h
//  basilisk
//
//  Created by Hanspeter Schaub on 6/26/20.
//

#ifndef vizStructures_h
#define vizStructures_h

#include <vector>

#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/DeviceCmdMsgPayload.h"
#include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefC/DataNodeUsageMsgPayload.h"
#include "architecture/msgPayloadDefCpp/DataStorageStatusMsgPayload.h"
#include "architecture/msgPayloadDefC/PowerStorageStatusMsgPayload.h"
#include "architecture/msgPayloadDefC/FuelTankMsgPayload.h"
#include "architecture/msgPayloadDefC/ColorMsgPayload.h"

#include "architecture/msgPayloadDefCpp/CSSConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
#include "architecture/msgPayloadDefCpp/ChargeMsmMsgPayload.h"

#include "architecture/messaging/messaging.h"


/*! Structure to store that status of a Basilisk message being read in by ``vizInterface``. */
typedef struct
//@cond DOXYGEN_IGNORE
MsgCurrStatus
//@endcond
{
    uint64_t lastTimeTag = 0xFFFFFFFFFFFFFFFF;  //!< [ns] The previous read time-tag for msg
    bool dataFresh = false;                     //!< Flag indicating that new data has been read
}MsgCurrStatus;


/*! Structure to store a thruster group information. */
typedef struct
//@cond DOXYGEN_IGNORE
ThrClusterMap
//@endcond
{
    std::string thrTag;   //!< ModelTag associated with the thruster grouping
    int color[4] = {-1};  //!< RGBA thruster plume color for all thrusters in this group
}ThrClusterMap;


/*! Vizard setting structure to define a pointing line feature.  This is used to draw a colored
    line from one space object to another space object.
 */
typedef struct
//@cond DOXYGEN_IGNORE
PointLine
//@endcond
{
    std::string fromBodyName;   //!< name of the body to start the line
    std::string toBodyName;     //!< name of the body to point the line towards
    int lineColor[4];           //!< desired RGBA as values between 0 and 255
}PointLine;


/*! Vizard setting structure to define a keep out/in cone visual feature.  Here cone is drawn
    attached to one body with a body-fixed position and heading.  If a second celestial object
    is within or outside this code, then the opacity of the cone changes.
*/
typedef struct
//@cond DOXYGEN_IGNORE
KeepOutInCone
//@endcond
{
    bool isKeepIn;              //!< True -> keep in cone created, False -> keep out cone created
    double position_B[3];       //!< [m] cone start relative to from body coordinate frame
    double normalVector_B[3];   //!< cone normal direction vector
    double incidenceAngle;      //!< [rad] cone incidence angle
    double coneHeight;          //!< [m] sets height of visible cone (asthetic only, does not impact function)
    std::string fromBodyName;   //!< name of body to attach cone onto
    std::string toBodyName;     //!< detect changes if this body has impingement on cone
    int coneColor[4];           //!< desired RGBA as values between 0 and 255
    std::string coneName;       //!< cone name, if unspecified, viz will autogenerate name
}KeepOutInCone;


/*! Vizard setting structure to define a standard Vizard camera.  These can be assigned to
    any spacecraft and set to point at either another object, or to point in a body-fixed direction.
*/
typedef struct
//@cond DOXYGEN_IGNORE
StdCameraSettings
//@endcond
{
    std::string spacecraftName; //!< name of spacecraft onto which to place a camera
    int setMode=1;              //!< 0 -> body targeting, 1 -> pointing vector (default)
    int showHUDElementsInImage=0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double fieldOfView=-1;      //!< [rad], edge-to-edge field of view setting, -1 -> use default, values between 0.0001 and 179.9999 deg valid
    std::string bodyTarget;     //!< Name of body camera should point to (default to first celestial body in messages). This is a setting for body targeting mode.
    int setView=0;              //!< 0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track (default to nadir). This is a setting for body targeting mode.
    double pointingVector_B[3]; //!< (default to 1, 0, 0). This is a setting for pointing vector mode.
    double position_B[3];       //!< (default to 0, 0, 0). If a non-zero vector, this determines the location of the camera.  If a zero vector, then the camera is placed outside of the spacecraft along the pointing vector direction.
    std::string displayName=""; //!< (optional) name of the standard camera panel
}StdCameraSettings;


/*! Vizard User Interface structure specifying what actuator visualizations to show.
*/
typedef struct
//@cond DOXYGEN_IGNORE
ActuatorGuiSettings
//@endcond
{
    std::string spacecraftName;    /*!< Specify which spacecraft should show actuator information.
                                         If not provided then the ``viz.spacecraftName`` is used. */
    int viewThrusterPanel=0;       //!< [bool] should thruster panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewThrusterHUD=0;         //!< [bool] should thruster Heads-Up-Display be shown, -1 (off), 0 (default), 1 (on)
    int viewRWPanel=0;             //!< [bool] should reaction wheel panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewRWHUD=0;               //!< [bool] should reaction wheel Heads-Up-Display be shown, -1 (off), 0 (default), 1 (on)
    int showThrusterLabels=0;      //!< [bool] should the thruster labels be shown, -1 (off), 0 (default), 1 (on)
    int showRWLabels=0;            //!< [bool] should the reaction wheel labels be shown, -1 (off), 0 (default), 1 (on)
}ActuatorGuiSettings;


/*! Vizard User Interface structure InstrumentGuiSettings InstrumentGuiSettings specify what instrument visualizations to show
 */
typedef struct
//@cond DOXYGEN_IGNORE
InstrumentGuiSettings
//@endcond
{
    std::string spacecraftName;     /*!< Specify which spacecraft should show actuator information.
                                         If not provided then the ``viz.spacecraftName`` is used. */
    int viewCSSPanel=0;             //!< [int] should CSS panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewCSSBoresight=0;         //!< [int] should CSS boresight axes be shown, -1 (off), 0 (default), 1 (on)
    int viewCSSCoverage=0;          //!< [int] should CSS coverage spheres be shown, -1 (off), 0 (default), 1 (on)
    int showCSSLabels=0;            //!< [int] should CSS panel labels be shown, -1 (off), 0 (default), 1 (on)
    int showGenericSensorLabels=0;  //!< [int] Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showTransceiverLabels=0;    //!< [int] Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showTransceiverFrustrum=0;  //!< [int] Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showGenericStoragePanel=0;  //!< [int] Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showMultiShapeLabels=0;    //!< [int] Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
}InstrumentGuiSettings;


/*! Structure defining a custom CAD model to load to represent a simulation object.
*/
typedef struct
//@cond DOXYGEN_IGNORE
CustomModel
//@endcond
{
    std::string modelPath;                      //!< Path to model obj -OR- ``CUBE``, ``CYLINDER``, or ``SPHERE`` to use a primitive shape
    std::vector<std::string> simBodiesToModify; //!< Which bodies in scene to replace with this model, use ``ALL_SPACECRAFT`` to apply custom model to all spacecraft in simulation
    double offset[3];                           //!< [m] offset to use to draw the model
    double rotation[3];                         //!< [rad] 3-2-1 Euler angles to rotate CAD about z, y, x axes
    double scale[3];                            //!< desired model scaling factor along the body x, y, z axes in spacecraft CS
    std::string customTexturePath;              //!< (Optional) Path to texture to apply to model (note that a custom model's .mtl will be automatically imported with its textures during custom model import)
    std::string normalMapPath;                  //!< (Optional) Path to the normal map for the customTexture
    int shader=-1;                              //!< (Optional) Value of -1 to use viz default, 0 for Unity Specular Standard Shader, 1 for Unity Standard Shader
    std::vector<int> color;                     //!< Send desired RGBA as values between 0 and 255, default is gray, and will be applied to the albedo color setting
}CustomModel;


/*! Structure defining ground location information
 */
typedef struct
//@cond DOXYGEN_IGNORE
LocationPbMsg
//@endcond
{
    std::string stationName;            //!< ground location text label
    std::string parentBodyName;         //!< name of the parent planet body P on which the ground location G is positioned
    double r_GP_P[3];                   //!< [m] Position of location G relative to planet frame P
    double gHat_P[3];                   //!< ground location Normal relative to parent body frame.
    double fieldOfView = -1;            //!< [rad] Edge-to-Edge, -1 -> use default, values between 0.0001deg and 179.9999deg valid
    int color[4] = {-1};                //!< Send desired RGBA as values between 0 and 255, -1 -> use default. (Note: alpha is not supported on the lightweight LocationMarkers, which are used by default when number of Locations are >100)
    double range = 0;                   //!< [m] range of the ground location, use 0 (protobuffer default) to use viz default
    double markerScale = 0;             //!< (Optional) Value will be multiplied by default marker scale, value less than 1.0 will decrease size, greater will increase size
    bool isHidden = false;              //!< (Optional) True to hide Location, false to show (vizDefault)
}LocationPbMsg;


/*! Structure defining generic sensor information
 */
typedef struct
//@cond DOXYGEN_IGNORE
GenericSensor
//@endcond
{
    double r_SB_B[3];                   //!< [m] Position of sensor relative to body frame, in body frame components
    std::vector<double> fieldOfView;    //!< [rad] edge-to-edge field of view, single positive value means a conical sensor, 2 positive values are for a rectangular sensor
    double normalVector[3];             //!< normal vector of the sensor bore sight axis
    int isHidden = 0;                   //!< (optional) true to hide sensor HUD, false to show sensor HUD (default)
    double size = 0;                    //!< [m] (optional) size of the sensor visualization, use 0 (protobuffer default) to use viz default size
    std::vector<int> color;             //!< (optional) RGBA as values between 0 and 255, multiple colors can be populated in this field and will be assigned to the additional mode (Modes 0 and 1 will use the 0th color, Mode 2 will use the color indexed to 1, etc.  If 2 colors are provided, then the vector should have size 8 (2x4 color channels)
    std::string label = "";             //!< (optional) string to display on sensor label
    ReadFunctor<DeviceCmdMsgPayload> genericSensorCmdInMsg;   //!< [-] (Optional)  incoming sensor cmd state msg
    uint64_t genericSensorCmd = 0;      //!< [int] (optional) sensor cmd value, if cmd input msg is connected, then this is set from the message

}GenericSensor;


/*! Structure defining spcecraft Ellipsoid information
 */
typedef struct
//@cond DOXYGEN_IGNORE
Ellipsoid
//@endcond
{
    int isOn;                   //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int useBodyFrame;           //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true, default is to use the Hill frame
    double position[3];         //!< [m] Position of ellipsoid center in xyz (if using body frame) or radial-along track-orbit normal (if Hill frame)
    double semiMajorAxes[3];    //!< [m] Semi-major axes in xyz (if using body frame) or radial-along track-orbit normal (if Hill frame)
    std::vector<int> color;     //!< (optional) RGBA as values between 0 and 255, default is translucent gold
    int showGridLines;          //!< Show Gridlines on ellipsoid, Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
}Ellipsoid;


/*! Structure defining spcecraft QuadMap information
 */
typedef struct
//@cond DOXYGEN_IGNORE
QuadMap
//@endcond
{
    int ID;                        //!< ID of QuadMap to be used for updates
    std::string parentBodyName;    //!< Name of the parent body P (spacecraft or planet) on which the QuadMap is positioned
    std::vector<double> vertices;  //!< [m] Four vertices (x,y,z) required per quad, order them clockwise about perimeter of quad in parent body frame P
    std::vector<int> color;        //!< (optional) Send desired RGBA as values between 0 and 255
    bool isHidden;                 //!< (optional) Send string to display in center of QuadMap region, send "NOLABEL" to delete label
    std::string label;             //!< (optional)
}QuadMap;


/*! Structure defining spacecraft light information
 */
typedef struct
//@cond DOXYGEN_IGNORE
Light
//@endcond
{
    std::string label;                              //!< [Optional] Name to use to identify light
    double position[3];                             //!< [m] position of the light in body frame
    double fieldOfView;                             //!< [rad] angle is measured edge-to-edge
    double normalVector[3];                         //!< normal vector of the light in the body frame
    int lightOn;                                    //!< (Optional) Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double range;                                   //!< [m] Distance light will act over
    double intensity=-1;                            //!< (Optional) Intensity of light at light origin, default is 1.0
    int showLightMarker;                            //!< (Optional) Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double markerDiameter=-1;                       //!< [m] (Optional) Size to draw the visible lens of the light, default is 0.01 m
    std::vector<int> color;                         //!< (Optional) Send desired RGBA as values between 0 and 255, default is pure white
    double gammaSaturation=-1;                      //!< (Optional) Desired gamma saturation of the light lens, 0 to match light color, 1.0 for pure white, default is 0.8
    int showLensFlare;                              //!< (Optional) Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double lensFlareBrightness=-1;                  //!< (Optional) Simulates refraction of light in camera lens, this value controls the size and brightness of the lens flare, default is 0.3
    double lensFlareFadeSpeed=-1;                   //!< (Optional) Speed with which the lens flare fades, default is 4.0
    ReadFunctor<DeviceCmdMsgPayload> onOffCmdInMsg; //!< (Optional)  incoming light on/off cmd state msg
}Light;


/*! Structure defining Multi-Shape-Method (MSM) configurations
 */
typedef struct
//@cond DOXYGEN_IGNORE
MultiShape
//@endcond
{
    int isOn=0;                         //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double position[3];                 //!< [m] MSM sphere position in the body frame
    double radius;                      //!< [m] radius of the sphere
    double currentValue;                //!< [Coulomb] current sphere charge value
    double maxValue;                    //!< [Coulomb] maximum sphere charge value
    std::vector<int> positiveColor;     //!< (optional) Send desired RGBA as values between 0 and 255, default is green
    std::vector<int> negativeColor;     //!< (optional) Send desired RGBA as values between 0 and 255, default is red
    int neutralOpacity=-1;              //!< (optional) Send desired opacity value between 0 and 255 for when charge is neutral
    std::string shape = "";             //!< (optional) Set shape to use "CUBE", "CYLINDER", or "SPHERE" (default)
    double dimensions[3];               //!< [m] Desired dimensions of selected shape in x, y, and z (For cylinder, z is height)
    double rotation[3];                 //!< [MRP] Desired orientation of the Multi Shape in the spacecraft body frame
}MultiShape;


/*! Structure defining Multi-Shape-Method (MSM) information
 */
typedef struct
//@cond DOXYGEN_IGNORE
MultiShapeInfo
//@endcond
{
    std::vector<MultiShape *> msmList;                      //!< list of MSM configuration information
    ReadFunctor<ChargeMsmMsgPayload> msmChargeInMsg;        //!< input message to read current MSM charges.  If not connected, currentValue can be set directly from python
}MultiShapeInfo;


/*! Structure defining generic storage information
 */
typedef struct
//@cond DOXYGEN_IGNORE
GenericStorage
//@endcond
{
    std::string label = "";             //!< Name of storage device
    std::string type = "";              //!< Type of storage device, i.e. "Battery", 'Hard Drive", "Propellant Tank", etc.
    double currentValue = 0;            //!< current absolute value of the storage device, set from the input message if connected
    double maxValue = 0;                //!< maximum absolute value of the storage device, set from the input message if connected
    std::string units = "";             //!< (Optional) Units of stored quantity, i.e. "bytes", "TB", "kg", etc.
    std::vector<int> color;             //!< (Optional) Send desired RGBA as values between 0 and 255, multiple colors can be populated in this field and will be used to color the bar graph between thresholds (i.e. the first color will be used between values of 0 and threshold 1, the second color will be used between threshold 1 and 2,..., the last color will be used between threshold n and the maxValue
    std::vector<int> thresholds;        //!< (Optional) Percentages of maxValue at which to change color, note that there should be one fewer threshold values than colors
    ReadFunctor<PowerStorageStatusMsgPayload> batteryStateInMsg;    //!< (Optional)  incoming battery state msg, only connect one input message
    ReadFunctor<DataStorageStatusMsgPayload> dataStorageStateInMsg; //!< (Optional)  incoming data storage state msg, only connect one input message
    ReadFunctor<FuelTankMsgPayload> fuelTankStateInMsg;             //!< (Optional)  incoming fuel tank state msg, only connect one input message
}GenericStorage;


/*! Structure defining antenna transceiver information
 */
typedef struct
//@cond DOXYGEN_IGNORE
Transceiver
//@endcond
{
    double r_SB_B[3];                   //!< [m] Position of sensor relative to body frame, in body frame components
    double fieldOfView;                 //!< [rad] edgle-to-edge transceiver access cone
    double normalVector[3];             //!< normal vector of the transceiver bore sight axis
    int isHidden = 0;                   //!< (optional) true to hide sensor HUD, false to show transceiver HUD (default)
    std::vector<int> color;             //!< (optional) RGBA as values between 0 and 255
    std::string label = "";             //!< (optional) string to display on sensor label
    int animationSpeed = 0;             //!< (optional) Set transmission animation speed to a value between 1(slowest) to 10 (fastest), or 0 to use viz default
    std::vector<ReadFunctor<DataNodeUsageMsgPayload>> transceiverStateInMsgs;   //!< (Optional)  vector of incoming transceiver state msgs
    int transceiverState = 0;           //!< [int] (optional) transceiver state value, if communication input msg is connected, then this is set from the message
}Transceiver;


/*! Defines a data structure for the spacecraft state messages and ID's.
 */
typedef struct
//@cond DOXYGEN_IGNORE
VizSpacecraftData
//@endcond
{
    std::string spacecraftName = "bsk-Sat";                     //!< Name of the spacecraft.
    std::string parentSpacecraftName = "";                      //!< Name of parent spacecraft object for multi-body spacecraft
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;               //!< msg of incoming spacecraft data
    MsgCurrStatus scStateInMsgStatus;                           //!< (Private) status of the incoming spacecraft  data message
    SCStatesMsgPayload scStateMsgBuffer;                        //!< (Private) s/c state message data

    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwInMsgs;   //!< (Optional) Vector of incoming RW state messages.
    std::vector<MsgCurrStatus> rwInMsgStatus;                   //!< (Private) RW msg status vector
    std::vector<RWConfigLogMsgPayload> rwInMessage;             //!< (Private) RW message data vector

    std::vector<ReadFunctor<CSSConfigLogMsgPayload>> cssInMsgs; //!< (Optional) Vector of CSS config log messages
    std::vector<MsgCurrStatus> cssConfLogInMsgStatus;           //!< (Private) status of the incoming array of css configuration log messages
    std::vector<CSSConfigLogMsgPayload> cssInMessage;           //!< (Private) CSS message data vector

    std::vector<ReadFunctor<THROutputMsgPayload>> thrInMsgs;    //!< (Optional) vector of thruster input messages
    std::vector<MsgCurrStatus> thrMsgStatus;                    //!< (Private) THR msg status vector
    std::vector<THROutputMsgPayload> thrOutputMessage;          //!< (Private) Thr message data vector

    std::vector<ThrClusterMap> thrInfo;                         //!< thruster tagging info

    std::vector<GenericSensor *> genericSensorList;             //!< (Optional) Vector of generic sensor configuration info

    std::vector<Transceiver *> transceiverList;                 //!< (Optional) Vector of transceiver configuration info

    std::vector<GenericStorage *> genericStorageList;           //!< (Optional) Vector of generic storage configuration info

    std::vector<Light *> lightList;                             //!< (Optional) Vector of spacecraft light devices

    std::string spacecraftSprite = "";                          //!< Set sprite for this spacecraft only through shape name and optional int RGB color values [0,255] Possible settings: "CIRCLE","SQUARE", "STAR", "TRIANGLE" or "bskSat" for a 2D spacecraft sprite of the bskSat shape
    std::string modelDictionaryKey = "";                        //!< (Optional) string specifiying which Vizard cad model to use. If set, it over-rides the model selected by `spacecraftName`
    std::string logoTexture = "";                               //!< (Optional) Path to image texture to be used to customize built-in spacecraft models
    std::vector<int> oscOrbitLineColor;                         //!< (Optional) Send desired RGBA as values between 0 and 255, color can be changed at any time step
    std::vector<int>  trueTrajectoryLineColor;                  //!< (Optional) Send desired RGBA as values between 0 and 255, color can be changed at any time step
    ReadFunctor<ColorMsgPayload> trueTrajectoryLineColorInMsg;  //!< (Optional) Messages specifying true trajectory orbit line RGBA colors.  If connected, this replaces the values set in trueTrajectoryLineColor
    MultiShapeInfo msmInfo;                                     //!< (Optional) MSM configuration information
    std::vector<Ellipsoid *> ellipsoidList;                     //!< (Optional) ellipsoid about the spacecraft location
}VizSpacecraftData;


/*! Structure defining various Vizard options
*/
typedef struct
//@cond DOXYGEN_IGNORE
VizSettings
//@endcond
{
    double      ambient = -1.0;                    /*!< Ambient background lighting. Should be a value between 0 and 8.
                                                             A value of -1 means it is not set. */
    int32_t     orbitLinesOn = 0;                  //!< Toggle to show osculating orbit lines, Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for relative to parent body, 2 for relative to chief spacecraft body
    int32_t     trueTrajectoryLinesOn = 0;         //!< Toggle to show true orbit lines, Value of 0 (protobuffer default) to use viz default, -1 for false, 1 to use inertial positions, 2 for relative to chief spacecraft body
    int32_t     spacecraftCSon = 0;                //!< Toggle for showing spacecraft CS with values -1 (off), 0 (default), 1 (on)
    int32_t     planetCSon = 0;                    //!< Toggle for showing planet CS with values -1 (off), 0 (default), 1 (on)
    std::vector<PointLine> pointLineList;          //!< vector of lines pointing from the spacecraft towards another scene object
    std::vector<KeepOutInCone> coneList;           //!< vector of keep in/out cones
    std::vector<StdCameraSettings> stdCameraList;  //!< vector of spacecraft cameras
    std::vector<CustomModel> customModelList;      //!< vector of custom object models
    std::vector<ActuatorGuiSettings> actuatorGuiSettingsList;     //!< msg containing the flags on displaying the actuator GUI elements
    std::vector<InstrumentGuiSettings> instrumentGuiSettingsList; //!< msg containing the flags on displaying instruments
    std::string skyBox = "";                       /*!< string containing the star field options, an empty string'' provides default NASA SVS Starmap,
                                                      ``ESO`` use ESO Milky Way skybox, ``black`` provides a black background,
                                                      or provide a filepath to custom background */
    bool        dataFresh;                         //!< flag indicating if the settings have been transmitted,
    int32_t viewCameraBoresightHUD = 0;            //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t viewCameraConeHUD = 0;                 //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCSLabels = 0;                      //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCelestialBodyLabels = 0;           //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showSpacecraftLabels = 0;              //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCameraLabels = 0;                  //!< Value of 0 to use viz default, -1 for false, 1 for true
    double customGUIReferenceHeight = -1.0;        //!< [px] GUI scaling parameter, Value of 0 or -1 to use viz default, minimum value 300
    std::string defaultSpacecraftSprite = "";      //!< Set sprite for ALL spacecraft through shape name and optional int RGB color values [0,255] Possible settings: "CIRCLE","SQUARE", "STAR", "TRIANGLE" or "bskSat" for a 2D spacecraft sprite of the bskSat shape
    int32_t showSpacecraftAsSprites = 0;           //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCelestialBodiesAsSprites = 0;      //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t show24hrClock = 0;                     //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showDataRateDisplay = 0;               //!< flag to show data frame rate, 0 (protobuffer default), -1 for false, 1 for true
    double  keyboardAngularRate = -1.0;            //!< [rad/sec] Rotation rate of camera keyboard rates per click.  Value of less than/equal to zero to use viz default
    double  keyboardZoomRate = -1.0;               //!< Value for speed at which the camera zooms in or out.  Value of less than/equal to zero to use viz default
    int defaultThrusterColor[4] = {-1};            //!< Default thruster plume color RGBA values
    double  defaultThrusterPlumeLifeScalar = 1.0;  //!< Value of 1.0 or 0.0 to use viz default, values between 0 and 1 will decrease the length of all plumes, >1 will increase lengths of all plumes
    int orbitLineSegments = 0;                     //!< Value of 0 (protobuffer default) to use viz default or any value greater than or equal to 4
    int relativeOrbitRange = 0;                    //!< [deg] Value of 0 (protobuffer default) to use viz default or any value greater than or equal to 1
    std::string mainCameraTarget = "";             //!< If valid spacecraft or celestial body name is provided, the main camera will be targeted at that body at start
    int showHillFrame = 0;                         //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showVelocityFrame = 0;                     //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int relativeOrbitFrame = 0;                    //!< Value of 0 (protobuffer default) or 1 to use Hill Frame, 2 to use Velocity Frame
    double spacecraftShadowBrightness = -1.0;      //!< Control the ambient light specific to spacecraft objects, value between 0 and 1, use negative value to use viz default
    double spacecraftSizeMultiplier = -1;          //!< Control the display size of spacecraft in the Planet and Solar System Views, values greater than 0, use negative value to use viz default
    double spacecraftHelioViewSizeMultiplier = -1; //!< Control the display size of spacecraft in the Solar System View, values greater than 0, use negative value to use viz default
    int forceStartAtSpacecraftLocalView = -1;      //!< Require Vizard to start up in spacecraft-view on start up
    int useSimpleLocationMarkers = 0;              //!< Value of 0 (protobuffer default) to use simplified Location markers when number of locations is greater than 100, -1 to force use of full-featured Location, 1 to force use of simplified Location (no cones, range, or communication lines)

    int32_t showLocationCommLines = 0;                    //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showLocationCones = 0;                        //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showLocationLabels = 0;                       //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t atmospheresOff = 0;                           //!< Toggle to disable the atmosphere effect on celestial bodies, Value of 0 (protobuffer default to use viz default, -1 for false, 1 for true.
    int32_t scViewToPlanetViewBoundaryMultiplier = 0;     //!< Multiplier x 1000m to set the boundary at which the spacecraft local view transitions to planet view. Valid range from 1 to 10 or 0 to use viz default.
    int32_t planetViewToHelioViewBoundaryMultiplier = 0;  //!< Multiplier x (10000 * current planet local scale) at which the planet view transitions to the solar system view. Valid range from 1 to 10 or 0 to use viz default.
    double sunIntensity = 0;                              //!< Multiplier for the intensity of the light being used as the main light source or sun, value of 0 to use viz default
    int32_t attenuateSunLightWithDistance = 0;            //!< Toggle to reduce brightness of sun lighting with the square of the distance from the sun. Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true.
    int showLightLabels=0;                                //!< Toggle to label spacecraft light elements, Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double celestialBodyHelioViewSizeMultiplier = -1;     //!< Control the display size of celestial bodies in the Solar System View, values greater than 0, use negative value to use viz default
    int showMissionTime = 0;                              //!< flag to show the mission time instead of the simulation time. Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    std::string keyboardLiveInput = "";                   //!< string of alphanumeric key inputs to listen for during 2-way communication
    int64_t messageBufferSize = 0;                        //!< [bytes] Maximum size of vizMessages to be loaded into memory at one time, -1 to force loading of entire file into memory, 0 to use viz default
    std::string truePathRelativeBody = "";                //!< String of the celestial body name to plot the true path trajectory line[s] against, empty string to use the spacecraft's primary body
    std::string truePathRotatingFrame = "";               //!< String must contain the names of two distinct celestial bodies, separated by a space, to define the desired rotating frame for plotting true path trajectories
    std::string truePathFixedFrame = "";                  //!< String of the spacecraft or celestial body name whose rotation matrix will provide the fixed frame to plot the true path trajectory against
    int32_t showQuadMapLabels = 0;                        //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    double spacecraftOrbitLineWidth = 0;                  //!< Value of 0 (protobuffer default) to use viz default, values greater than 0 to scale spacecraft orbit line width
    double celestialBodyOrbitLineWidth = 0;               //!< Value of 0 (protobuffer default) to use viz default, values greater than 0 to scale celestial body orbit line width
}VizSettings;


/*! Structure defining various Vizard options
*/
typedef struct
//@cond DOXYGEN_IGNORE
LiveVizSettings
//@endcond
{
    std::vector<PointLine> targetLineList;       //!< vector of lines between 2 scenario targets.  This list is redrawn on each update step, thus the line properties can change with time.
    std::string relativeOrbitChief = "";         //!< If valid spacecraft name provided, the relative orbit chief spacecraft will be set to that spacecraft object. Setting the string to "AUTO" or leaving this field empty will select the camera target spacecraft as the chief.
    bool terminateVizard=false;                  //!< If true, Vizard application will immediately shut down and exit
}LiveVizSettings;


/*! Structure defining Vizard dialog boxes
*/
typedef struct
//@cond DOXYGEN_IGNORE
VizEventDialog
//@endcond
{
    std::string eventHandlerID="";              //!< Name of Vizard event handler to be returned with EventReply responses
    std::string displayString="";               //!< Contains the information or choice that should be posed to the user
    std::vector<std::string> userOptions={};    //!< Determines how many user choices will be shown and what the displayed string will say. If this is empty, the dialog is assumed to be informational only
    double durationOfDisplay=-1;                //!< How long to display the dialog box for, use -1 to leave panel on display until closed by user
    bool useSimElapsedTimeForDuration=false;    //!< [seconds] If true and duration of display is set, use the sim elapsed time to calculate when to hide window. If false, use real time (system clock).
    int useConfirmationPanel=0;                 //!< Should event handler pop up a confirmation window before sending back response, -1 to not show confirmation panel, 0 to use viz default, and 1 to require a user confirmation of their selection
    int hideOnSelection=0;                      //!< Should the panel disappear after the user has selected an option?, -1 to continue to show panel, 0 to use viz default, and 1 to hide panel after user makes a selection, 2 to destroy panel after user makes a selection
    std::string dialogFormat="none";            //!< Select format for dialog box: WARNING, CAUTION, or none to use viz default format
}VizEventDialog;


/*! Structure defining vizard gravity body values */
typedef struct
//@cond DOXYGEN_IGNORE
GravBodyInfo
//@endcond
{
    std::string bodyName;            //!< celestial body name
    double mu;                       //!< [m^3/s^2] celestial body gravity constant
    double radEquator;               //!< [m] celestial body radius at equator
    double radiusRatio;              //!< radiusPolar/radiusEq
    std::string modelDictionaryKey;  //!< (optional) Vizard model key to use.  If set, it over-rides the model selected by the name
}GravBodyInfo;

#endif /* vizStructures_h */
