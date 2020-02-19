/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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
 
 */


#ifndef VIZ_INTERFACE_H
#define VIZ_INTERFACE_H

#include "../utilities/vizProtobuffer/vizMessage.pb.h"
#include <vector>
#include <fstream>
#include <map>
#include <zmq.h>

#include "_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/system_messaging.h"
#include "simFswInterfaceMessages/stSensorIntMsg.h"
#include "simFswInterfaceMessages/cameraConfigMsg.h"
#include "simFswInterfaceMessages/cameraImageMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/rwConfigLogSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simFswInterfaceMessages/cssArraySensorIntMsg.h"
#include "simMessages/thrOutputSimMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "../fswAlgorithms/fswMessages/cssConfigFswMsg.h"
#include "../fswAlgorithms/fswMessages/thrArrayConfigFswMsg.h"
#include "utilities/bskLogging.h"

#define VIZ_MAX_SIZE 100000

/*! Structure to store that status of a Basilisk message being read in by ``vizInterface``. */
typedef struct {
    int64_t msgID;        //!< [-] message ID associated with source
    uint64_t lastTimeTag; //!< [ns] The previous read time-tag for msg
    bool dataFresh;       //!< [-] Flag indicating that new data has been read
}MsgCurrStatus;

/*! Structure to store a thruster group information. */
typedef struct {
    std::string thrTag;   //!< [-] ModelTag associated with the thruster grouping
    uint32_t    thrCount; //!< [-] Number of thrusters used in this thruster group
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
    int setMode;                //!< 0 -> body targeting, 1 -> pointing vector (default)
    double fieldOfView;         //!< [rad], field of view setting, -1 -> use default, values between 0.0001 and 179.9999 deg valid
    std::string bodyTarget;     //!< Name of body camera should point to (default to first celestial body in messages). This is a setting for body targeting mode.
    int setView;                //!< 0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track (default to nadir). This is a setting for body targeting mode.
    double pointingVector_B[3]; //!< (default to 1, 0, 0). This is a setting for pointing vector mode.
    double position_B[3];       //!< (default to 0, 0, 0). If a non-zero vector, this determines the location of the camera.  If a zero vector, then the camera is placed outside of the spacecraft along the pointing vector direction.
}StdCameraSettings;

/*! Vizard User Interface structure specifying what actuator visualizations to show.
*/
typedef struct {
    std::string spacecraftName;     /*!< Specify which spacecraft should show actuator information.
                                         If not provided then the ``viz.spacecraftName`` is used. */
    int viewThrusterPanel=-1;       //!< [bool] should thruster panel illustration be shown
    int viewThrusterHUD=-1;         //!< [bool] should thruster Heads-Up-Display be shown
    int viewRWPanel=-1;             //!< [bool] should reaction wheel panel illustration be shown
    int viewRWHUD=-1;               //!< [bool] should reaction wheel Heads-Up-Display be shown
    int showThrusterLabels=-1;      //!< [bool] should the thruster labels be shown
    int showRWLabels=-1;            //!< [bool] should the reaction wheel labels be shown
}ActuatorGuiSettings;

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
    int shader;                             //!< (Optional) Value of -1 to use viz default, 0 for Unity Specular Standard Shader, 1 for Unity Standard Shader
}CustomModel;

/*! Defines a data structure for the spacecraft state messages and ID's.
 */
typedef struct {
    std::string spacecraftName = "bsk-Sat";                     //!< [-] Name of the spacecraft.
    std::string cssDataInMsgName = "css_sensors_data";          //!< [-] (Optional) Name of the incoming css data message
    std::string cssConfInMsgName = "css_config_data";           //!< [-] (Optional) Name of the incoming css constellation data message
    std::string scPlusInMsgName = "inertial_state_output";      //!< [-] Name of the incoming SCPlus data message
    std::vector <std::string> rwInMsgName;                      /*!< [-] Vector of names of the incoming RW state messages.  If this is not
                                                                        set directly, then it is auto-generated using the spacecraft name
                                                                        as the prefix. This is required if ``numRW`` is greater than 0.
                                                                 */
    std::vector <ThrClusterMap> thrMsgData;                     /*!< [-] (Optional) Name of the incoming thruster data.  This is
                                                                         required if ``numThr`` is greater than 0. */
    std::string starTrackerInMsgName = "star_tracker_state";    //!< [-] (Optional) Name of the incoming Star Tracker data
    int numRW = 0;                                              //!< [-] (Optional) Number of RW
    int numThr = 0;                                             //!< [-] (Optional) Number of Thrusters

    std::vector<MsgCurrStatus> rwInMsgID;                       //!< [-] (Private) ID of the incoming rw data
    std::vector<MsgCurrStatus> thrMsgID;                        //!< [-] (Private) ID of the incoming thruster data
    MsgCurrStatus starTrackerInMsgID;                           //!< [-] (Private) ID of the incoming Star Tracker data
    MsgCurrStatus scPlusInMsgID;                                //!< [-] (Private) ID of the incoming SCPlus data
    MsgCurrStatus cssDataInMsgId;                               //!< [-] (Private) ID of the incoming css data
    MsgCurrStatus cssConfInMsgId;                               //!< [-] (Private) ID of the incoming css constellation data
    std::vector <RWConfigLogSimMsg> rwInMessage;                //!< [-] (Private) RW message data
    STSensorIntMsg STMessage;                                   //!< [-] (Private) ST message data
    std::vector <THROutputSimMsg> thrOutputMessage;             //!< [-] (Private) Thr message data
    SCPlusStatesSimMsg scPlusMessage;                           //!< [-] (Private) s/c plus message data
//    CSSArraySensorIntMsg cssDataMessage;                      //!< [-] (Private) CSS message data
    CSSConfigFswMsg cssConfigMessage;                           //!< [-] (Private) CSS config message data
}VizSpacecraftData;

/*! Structure defining various Vizard options
*/
typedef struct {
    double      ambient;                            /*!< [-] Ambient background lighting. Should be a value between 0 and 8.
                                                             A value of -1 means it is not set. */
    int32_t     orbitLinesOn;                       //!< toogle for showing orbit lines (-1, 0, 1)
    int32_t     spacecraftCSon;                     //!< toogle for showing spacecraft CS (-1, 0, 1)
    int32_t     planetCSon;                         //!< toogle for showing planet CS (-1, 0, 1)
    std::vector<PointLine> pointLineList;           //!< vector of powerLine structures
    std::vector<KeepOutInCone> coneList;            //!< vector of keep in/out cones
    std::vector<StdCameraSettings> stdCameraList;   //!< vector of spacecraft cameras
    std::vector<CustomModel> customModelList;       //!< vector of custom object models
    std::vector<ActuatorGuiSettings> actuatorGuiSettingsList; //!< msg containing the flags on displaying the actuator GUI elements
    std::string skyBox;         /*!< string containing the star field options, an empty string'' provides default NASA SVS Starmap,
                                     ``ESO`` use ESO Milky Way skybox, ``black`` provides a black background,
                                     or provide a filepath to custom background */
    bool        dataFresh;      //!< [-] flag indicating if the settings have been transmitted,
    int32_t viewCameraBoresightHUD ;                //!< Value of -1 to use viz default, 0 for false, 1 for true
    int32_t viewCameraConeHUD;                      //!< Value of -1 to use viz default, 0 for false, 1 for true
    int32_t showCSLabels;                           //!< Value of -1 to use viz default, 0 for false, 1 for true
    int32_t showCelestialBodyLabels;                //!< Value of -1 to use viz default, 0 for false, 1 for true
    int32_t showSpacecraftLabels ;                  //!< Value of -1 to use viz default, 0 for false, 1 for true

}VizSettings;


/*! Defines a data structure for the spacecraft state messages and ID's.
*/
class VizInterface : public SysModel {
public:
    VizInterface();
    ~VizInterface();
    void SelfInit();
    void CrossInit();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadBSKMessages();
    void WriteProtobuffer(uint64_t CurrentSimNanos);

public:
    std::vector<VizSpacecraftData> scData;      //!< [-] vector of spacecraft data containers
    std::vector <std::string> spiceInMsgName;   //!< [-] Name of the incoming Spice data
    std::string opnavImageOutMsgName;           //!< The name of the Image output message
    int opNavMode;                              /*!< [int] Set non-zero positive value  if Unity/Viz couple in direct
                                                 communication. (1 - regular opNav, 2 - performance opNav) */
    bool saveFile;                              //!< [Bool] Set True if Vizard should save a file of the data.
    bool liveStream;                            //!< [Bool] Set True if Vizard should receive a live stream of BSK data.
    void* bskImagePtr;                          /*!< [RUN] Permanent pointer for the image to be used in BSK
                                                     without relying on ZMQ because ZMQ will free it (whenever, who knows) */

    std::string cameraConfInMsgName = "camera_config_data";     //!< [-] Name of the incoming camera data
    MsgCurrStatus cameraConfMsgId;                              //!< [-] ID of the incoming camera  data
    CameraConfigMsg cameraConfigMessage;                        //!< [-] Camera config
    
    std::vector <std::string> planetNames;      //!< Vector of planet names we want to track, read in from python

    uint64_t numOutputBuffers;                  //!< [-] Number of buffers to request for the output messages
    
    int64_t FrameNumber;                        //!< Number of frames that have been updated for TimeStamp message
    std::string protoFilename;                  //!< Filename for where to save the protobuff message
    VizSettings settings;                       //!< [-] container for the Viz settings that can be specified from BSK

    BSKLogger bskLogger;                        //!< [-] BSK Logging object


private:
    // ZeroMQ State
    void* context;
    void* requester_socket;
    int firstPass;                                          //!< Flag to intialize the viz at first timestep

    int32_t imageOutMsgID;                                  //!< ID for the outgoing Image message
    std::vector<MsgCurrStatus>spiceInMsgID;                 //!< [-] IDs of the incoming planets' spice data
    std::vector <SpicePlanetStateSimMsg> spiceMessage;      //!< [-] Spice messages
    std::ofstream *outputStream;                            //!< [-] Output file stream opened in reset
    
    std::map<uint32_t, SpicePlanetStateSimMsg> planetData;  //!< -- Internal vector of planets
    
};

#endif /* VIZ_INTERFACE_H */
