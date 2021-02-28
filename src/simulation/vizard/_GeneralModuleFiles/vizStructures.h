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
#include "architecture/msgPayloadDefC/STSensorMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"

#include "architecture/msgPayloadDefCpp/CSSConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"

#include "architecture/messaging/messaging.h"


/*! Structure to store that status of a Basilisk message being read in by ``vizInterface``. */
typedef struct {
    uint64_t lastTimeTag = 0xFFFFFFFFFFFFFFFF;  //!< [ns] The previous read time-tag for msg
    bool dataFresh = false;                     //!< [-] Flag indicating that new data has been read
}MsgCurrStatus;

/*! Structure to store a thruster group information. */
typedef struct {
    std::string thrTag;   //!< [-] ModelTag associated with the thruster grouping
    int color[4] = {-1};  //!< [-] RGBA thruster plume color for all thrusters in this group
}ThrClusterMap;

/*! Vizard setting structure to define a pointing line feature.  This is used to draw a colored
    line from one space object to another space object.
 */
typedef struct {
    std::string fromBodyName;   //!< [-] name of the body to start the line
    std::string toBodyName;     //!< [-] name of the body to point the line towards
    int lineColor[4];           //!< [-] desired RGBA as values between 0 and 255
}PointLine;

/*! Vizard setting structure to define a keep out/in cone visual feature.  Here cone is drawn
    attached to one body with a body-fixed position and heading.  If a second celestial object
    is within or outside this code, then the opacity of the cone changes.
*/
typedef struct {
    bool isKeepIn;              //!< True -> keep in cone created, False -> keep out cone created
    double position_B[3];       //!< [m] cone start relative to from body coordinate frame
    double normalVector_B[3];   //!< [-] cone normal direction vector
    double incidenceAngle;      //!< [rad] cone incidence angle
    double coneHeight;          //!< [m] sets height of visible cone (asthetic only, does not impact function)
    std::string fromBodyName;   //!< name of body to attach cone onto
    std::string toBodyName;     //!< detect changes if this body has impingement on cone
    int coneColor[4];           //!< [-] desired RGBA as values between 0 and 255
    std::string coneName;       //!< [-] cone name, if unspecified, viz will autogenerate name
}KeepOutInCone;

/*! Vizard setting structure to define a standard Vizard camera.  These can be assigned to
    any spacecraft and set to point at either another object, or to point in a body-fixed direction.
*/
typedef struct {
    std::string spacecraftName; //!< name of spacecraft onto which to place a camera
    int setMode=1;              //!< 0 -> body targeting, 1 -> pointing vector (default)
    double fieldOfView=-1;      //!< [rad], field of view setting, -1 -> use default, values between 0.0001 and 179.9999 deg valid
    std::string bodyTarget;     //!< Name of body camera should point to (default to first celestial body in messages). This is a setting for body targeting mode.
    int setView=0;              //!< 0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track (default to nadir). This is a setting for body targeting mode.
    double pointingVector_B[3]; //!< (default to 1, 0, 0). This is a setting for pointing vector mode.
    double position_B[3];       //!< (default to 0, 0, 0). If a non-zero vector, this determines the location of the camera.  If a zero vector, then the camera is placed outside of the spacecraft along the pointing vector direction.
}StdCameraSettings;

/*! Vizard User Interface structure specifying what actuator visualizations to show.
*/
typedef struct {
    std::string spacecraftName;     /*!< Specify which spacecraft should show actuator information.
                                         If not provided then the ``viz.spacecraftName`` is used. */
    int viewThrusterPanel=0;       //!< [bool] should thruster panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewThrusterHUD=0;         //!< [bool] should thruster Heads-Up-Display be shown, -1 (off), 0 (default), 1 (on)
    int viewRWPanel=0;             //!< [bool] should reaction wheel panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewRWHUD=0;               //!< [bool] should reaction wheel Heads-Up-Display be shown, -1 (off), 0 (default), 1 (on)
    int showThrusterLabels=0;      //!< [bool] should the thruster labels be shown, -1 (off), 0 (default), 1 (on)
    int showRWLabels=0;            //!< [bool] should the reaction wheel labels be shown, -1 (off), 0 (default), 1 (on)
}ActuatorGuiSettings;

/*! Vizard User Interface structure specify what instrument visualizations to show
 */
typedef struct {
    std::string spacecraftName;     /*!< Specify which spacecraft should show actuator information.
                                         If not provided then the ``viz.spacecraftName`` is used. */
    int viewCSSPanel=0;             //!< [bool] should CSS panel illustration be shown, -1 (off), 0 (default), 1 (on)
    int viewCSSBoresight=0;         //!< [bool] should CSS boresight axes be shown, -1 (off), 0 (default), 1 (on)
    int viewCSSCoverage=0;          //!< [bool] should CSS coverage spheres be shown, -1 (off), 0 (default), 1 (on)
    int showCSSLabels=0;            //!< [bool] should CSS panel labels be shown, -1 (off), 0 (default), 1 (on)
}InstrumentGuiSettings;

/*! Structure defining a custom CAD model to load to represent a simulation object.
*/
typedef struct {
    std::string modelPath;                  //!< Path to model obj -OR- ``CUBE``, ``CYLINDER``, or ``SPHERE`` to use a primitive shape
    std::vector<std::string> simBodiesToModify; //!< Which bodies in scene to replace with this model, use ``ALL_SPACECRAFT`` to apply custom model to all spacecraft in simulation
    double offset[3];                       //!< [m] offset to use to draw the model
    double rotation[3];                     //!< [rad] 3-2-1 Euler angles to rotate CAD about z, y, x axes
    double scale[3];                        //!< [-] desired model scaling factor along the body x, y, z axes in spacecraft CS
    std::string customTexturePath;          //!< (Optional) Path to texture to apply to model (note that a custom model's .mtl will be automatically imported with its textures during custom model import)
    std::string normalMapPath;              //!< (Optional) Path to the normal map for the customTexture
    int shader=-1;                          //!< (Optional) Value of -1 to use viz default, 0 for Unity Specular Standard Shader, 1 for Unity Standard Shader
}CustomModel;

/*! Structure defining ground location information
 */
typedef struct {
    std::string stationName;                   //!< ground location text label
    std::string parentBodyName;         //!< name of the parent planet body P on which the ground location G is positioned
    double r_GP_P[3];                   //!< [m] Position of location G relative to planet frame P
    double gHat_P[3];                   //!< ground location Normal relative to parent body frame.
    double fieldOfView = -1;            //!< [rad] Edge-to-Edge, -1 -> use default, values between 0.0001deg and 179.9999deg valid
    int color[4] = {-1};                //!< Send desired RGBA as values between 0 and 255, -1 -> use default
    double range = 0;                   //!< [m] range of the ground location, use 0 (protobuffer default) to use viz default
}LocationPbMsg;

/*! Defines a data structure for the spacecraft state messages and ID's.
 */
typedef struct {
    std::string spacecraftName = "bsk-Sat";                     //!< [-] Name of the spacecraft.
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;               //!< [-] msg of incoming spacecraft data
    MsgCurrStatus scStateInMsgStatus;                           //!< [-] (Private) status of the incoming spacecraft  data message
    SCStatesMsgPayload scStateMsgBuffer;                          //!< [-] (Private) s/c state message data

    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwInMsgs;   //!< [-] (Optional) Vector of incoming RW state messages.
    std::vector<MsgCurrStatus> rwInMsgStatus;                   //!< [-] (Private) RW msg status vector
    std::vector<RWConfigLogMsgPayload> rwInMessage;             //!< [-] (Private) RW message data vector

    std::vector<ReadFunctor<CSSConfigLogMsgPayload>> cssInMsgs; //!< [-] (Optional) Vector of CSS config log messages
    std::vector<MsgCurrStatus> cssConfLogInMsgStatus;           //!< [-] (Private) status of the incoming array of css configuration log messages
    std::vector<CSSConfigLogMsgPayload> cssInMessage;           //!< [-] (Private) CSS message data vector

    ReadFunctor<STSensorMsgPayload> starTrackerInMsg;           //!< [-] (Optional) input message for Star Tracker data
    MsgCurrStatus starTrackerInMsgStatus;                       //!< [-] (Private) status of the incoming Star Tracker data message
    STSensorMsgPayload STMessage;                               //!< [-] (Private) ST message data

    std::vector<ReadFunctor<THROutputMsgPayload>> thrInMsgs;    //!< [-] (Optional) vector of thruster input messages
    std::vector<MsgCurrStatus> thrMsgStatus;                    //!< [-] (Private) THR msg status vector
    std::vector<THROutputMsgPayload> thrOutputMessage;          //!< [-] (Private) Thr message data vector

    std::vector<ThrClusterMap> thrInfo;                         //!< [-] thruster tagging info

    std::string spacecraftSprite = "";                          //!< Set sprite for this spacecraft only through shape name and optional int RGB color values [0,255] Possible settings: "CIRCLE","SQUARE", "STAR", "TRIANGLE" or "bskSat" for a 2D spacecraft sprite of the bskSat shape

}VizSpacecraftData;

/*! Structure defining various Vizard options
*/
typedef struct {
    double      ambient = -1.0;                     /*!< [-] Ambient background lighting. Should be a value between 0 and 8.
                                                             A value of -1 means it is not set. */
    int32_t     orbitLinesOn = 0;                  //!< toogle for showing orbit lines with values -1 (off), 0 (default), 1 (on)
    int32_t     spacecraftCSon = 0;                //!< toogle for showing spacecraft CS with values -1 (off), 0 (default), 1 (on)
    int32_t     planetCSon = 0;                    //!< toogle for showing planet CS with values -1 (off), 0 (default), 1 (on)
    std::vector<PointLine> pointLineList;           //!< vector of powerLine structures
    std::vector<KeepOutInCone> coneList;            //!< vector of keep in/out cones
    std::vector<StdCameraSettings> stdCameraList;   //!< vector of spacecraft cameras
    std::vector<CustomModel> customModelList;       //!< vector of custom object models
    std::vector<ActuatorGuiSettings> actuatorGuiSettingsList; //!< msg containing the flags on displaying the actuator GUI elements
    std::vector<InstrumentGuiSettings> instrumentGuiSettingsList; //!< msg containing the flags on displaying instruments
    std::string skyBox = "";         /*!< string containing the star field options, an empty string'' provides default NASA SVS Starmap,
                                     ``ESO`` use ESO Milky Way skybox, ``black`` provides a black background,
                                     or provide a filepath to custom background */
    bool        dataFresh;      //!< [-] flag indicating if the settings have been transmitted,
    int32_t viewCameraBoresightHUD = 0;            //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t viewCameraConeHUD = 0;                 //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCSLabels = 0;                      //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCelestialBodyLabels = 0;           //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showSpacecraftLabels = 0;              //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCameraLabels = 0;                  //!< Value of 0 to use viz default, -1 for false, 1 for true
    double customGUIScale = -1.0;                   //!< GUI scaling parameter, Value of -1 to use viz default, values in [0.5, 3]
    std::string defaultSpacecraftSprite = "";       //!< Set sprite for ALL spacecraft through shape name and optional int RGB color values [0,255] Possible settings: "CIRCLE","SQUARE", "STAR", "TRIANGLE" or "bskSat" for a 2D spacecraft sprite of the bskSat shape
    int32_t showSpacecraftAsSprites = 0;           //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t showCelestialBodiesAsSprites = 0;      //!< Value of 0 to use viz default, -1 for false, 1 for true
    int32_t show24hrClock = 0;                     //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showDataRateDisplay = 0;               //!< flag to show data frame rate, 0 (protobuffer default), -1 for false, 1 for true
    double  keyboardAngularRate = -1.0;            //!< [rad/sec] Rotation rate of camera keyboard rates per click.  Value of less than/equal to zero to use viz default
    double  keyboardZoomRate = -1.0;               //!< Value for speed at which the camera zooms in or out.  Value of less than/equal to zero to use viz default
    int defaultThrusterColor[4] = {-1};            //!< Default thruster plume color RGBA values
    double  defaultThrusterPlumeLifeScalar = 1.0; //!< Value of 1.0 or 0.0 to use viz default, values between 0 and 1 will decrease the length of all plumes, >1 will increase lengths of all plumes
    int orbitLineSegments = 0; //!< Value of 0 (protobuffer default) to use viz default or any value greater than or equal to 4
    int relativeOrbitRange = 0; //!< [deg] Value of 0 (protobuffer default) to use viz default or any value greater than or equal to 1
    int showHillFrame = 0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int showVelocityFrame = 0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int relativeOrbitFrame = 0; //!< Value of 0 (protobuffer default) or 1 to use Hill Frame, 2 to use Velocity Frame
    std::string relativeOrbitChief = "";            //!< If valid spacecraft name provided, the relative orbit chief spacecraft will be set to that spacecraft object. Setting the string to "AUTO" or leaving this field empty will select the camera target spacecraft as the chief.
    double spacecraftShadowBrightness = -1.0;       //!< Control the ambient light specific to spacecraft objects, value between 0 and 1, use negative value to use viz default
    double spacecraftSizeMultiplier = -1; //!< Control the display size of spacecraft in the Planet and Solar System Views, values greater than 0, use negative value to use viz default
    int32_t showLocationCommLines = 0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showLocationCones = 0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    int32_t showLocationLabels = 0; //!< Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
}VizSettings;


typedef struct{
    std::string bodyName;               //!< [-] celestial body name
    double mu;                          //!< [m^3/s^2] celestial body gravity constant
    double radEquator;                  //!< [m] celestial body radius at equator
    double radiusRatio;                 //!< [] radiusPolar/radiusEq
}GravBodyInfo;

#endif /* vizStructures_h */
