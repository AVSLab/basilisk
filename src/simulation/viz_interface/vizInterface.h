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
#include "utilities/bskPrint.h"


typedef struct {
    int64_t msgID;        //!< [-] message ID associated with source
    uint64_t lastTimeTag; //!< [ns] The previous read time-tag for msg
    bool dataFresh;       //!< [-] Flag indicating that new data has been read
}MsgCurrStatus;

typedef struct {
    std::string thrTag;   //!< [-] ModelTag associated with the thruster model
    uint32_t    thrCount; //!< [-] Number of thrusters used in this thruster model
}ThrClusterMap;

// define Viz setting messages
typedef struct {
    std::string fromBodyName;   //!< [-] name of the body to start the line
    std::string toBodyName;     //!< [-] name of the body to point the line towards
    int lineColor[4];           //!< [-] desired RGBA as values between 0 and 255
}PointLine;

typedef struct {
    bool isKeepIn;              //!< True -> keep in cone created, False -> keep out cone created
    double position_B[3];       //!< [m] cone start relative to from body coordinate frame
    double normalVector_B[3];   //!< [-] cone normal direction vector
    double incidenceAngle;      //!< [deg] cone incidence angle
    double coneHeight;          //!< [m] sets height of visible cone (asthetic only, does not impact function)
    std::string fromBodyName;   //!< name of body to attach cone onto
    std::string toBodyName;     //!< [-] detect changes if this body has impingement on cone
    int coneColor[4];              //!< [-] desired RGBA as values between 0 and 255
    std::string coneName;       //!< [-] cone name, if unspecified, viz will autogenerate name
}KeepOutInCone;

typedef struct {
    std::string spacecraftName; //!< Which spacecraft's camera 1
    bool viewPanel;             //!< Flag indicating if the camera panel is visible on the screen, default is falst
    int setView;                //!< Specify the view through 0 -> +X, 1 -> -X, 2 -> +Y, 3 -> -Y, 4 -> +Z, 5 -> -Z
    bool spacecraftVisible;     //!< Flag if the spacecraft itself is visible in this camera view. Default is false
    double fieldOfView;         //!< field of view setting, -1 -> use default, values between 0.0001 and 179.9999 valid
    std::string targetBodyName; //!< for planet centric camera only, name of the planet relative which to point the camera
}CameraSettings;

typedef struct {
    std::string spacecraftName; //!< Which spacecraft's camera 1
    int viewThrusterPanel;
    int viewThrusterHUD;
    int viewRWPanel;
    int viewRWHUD;
}ActuatorGuiSettings;

typedef struct {
    double      ambient;        //!< [-] Ambient background lighting. Should be a value between 0 and 8.  A value of -1 means it is not set.
    int32_t     orbitLinesOn;   //! toogle for showing orbit lines (-1, 0, 1)
    int32_t     spacecraftCSon; //! toogle for showing spacecraft CS (-1, 0, 1)
    int32_t     planetCSon;     //! toogle for showing planet CS (-1, 0, 1)
    std::vector<PointLine> pointLineList;   //! vector of powerLine structures
    std::vector<KeepOutInCone> coneList;    //! vector of keep in/out cones
    CameraSettings cameraOne;   //! msg containing camera one settings
    CameraSettings cameraTwo;   //! msg containing camera one settings
    CameraSettings cameraPlanet;//! msg containing the planet camera settings
    std::vector<ActuatorGuiSettings> actuatorGuiSettingsList; //! msg containing the flags on displaying the actuator GUI elements
    std::string skyBox;         //! string containing the star field options, '' provides default NASA SVS Starmap, "ESO" use ESO Milky Way skybox, "black" provides a black background, or provide a filepath to custom background
    bool        dataFresh;      //!< [-] flag indicating if the settings have been transmitted,
}VizSettings;


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
    std::string cssDataInMsgName;               //! [-] Name of the incoming css data
    std::string cssConfInMsgName;               //! [-] Name of the incoming css constellation data
    std::string cameraConfInMsgName;            //! [-] Name of the incoming camera data
    std::string scPlusInMsgName;                //! [-] Name of the incoming SCPlus data
    std::vector <std::string> spiceInMsgName;   //! [-] Name of the incoming Spice data
    std::vector <std::string> rwInMsgName;      //! [-] Name of the incoming rw data
    std::vector <ThrClusterMap> thrMsgData;     //! [-] Name of the incoming thruster data
    std::string starTrackerInMsgName;           //! [-] Name of the incoming Star Tracker data

    std::string spacecraftName;                 //! [-] Name of the spacecraft being simulated

    std::string opnavImageOutMsgName;           //! The name of the Image output message*/

    uint64_t numSensors;
    int opNavMode;                              //! [int] Set non-zero positive value  if Unity/Viz couple in direct communication. (1 - regular opNav, 2 - performance opNav)
    bool saveFile;                              //! [Bool] Set True if Vizard should save a file of the data.
    bool liveStream;                            //! [Bool] Set True if Vizard should receive a live stream of BSK data.
    void* bskImagePtr;                            //! [RUN] Permanent pointer for the image to be used in BSK without relying on ZMQ because ZMQ will free it (whenever, who knows)

    std::string vizOutMsgName;
    std::vector <std::string> planetNames;      //!< -- Names of planets we want to track, read in from python

    uint64_t numOutputBuffers;                  //! [-] Number of buffers to request for the output messages

    int64_t FrameNumber;                        //! Number of frames that have been updated for TimeStamp message
    std::string protoFilename;                  //! Filename for where to save the protobuff message
    int numRW;                                  //! [-] Number of RW set in python
    int numThr;                                 //! [-] Number of Thrusters set in python
    VizSettings settings;                       //! [-] container for the Viz settings that can be specified from BSK

    CameraConfigMsg cameraConfigMessage;        //! [-] camera config message copy

    BSKPrint bskPrint;                      //!< -- BSK Logging


private:
    // ZeroMQ State
    void* context;
    void* requester_socket;

    std::vector<MsgCurrStatus> rwInMsgID;       //! [-] ID of the incoming rw data
    std::vector<MsgCurrStatus> thrMsgID;        //! [-] ID of the incoming thruster data
    MsgCurrStatus starTrackerInMsgID;           //! [-] ID of the incoming Star Tracker data
    MsgCurrStatus scPlusInMsgID;                //! [-] ID of the incoming SCPlus data
    std::vector<MsgCurrStatus>spiceInMsgID;     //! [-] IDs of the incoming planets' spice data
    MsgCurrStatus cssDataInMsgId;               //! [-] ID of the incoming css data
    MsgCurrStatus cssConfInMsgId;               //! [-] ID of the incoming css constellation data
    MsgCurrStatus cameraConfMsgId;              //! [-] ID of the incoming camera  data
    int32_t imageOutMsgID;                      //! ID for the outgoing Image message */
    int firstPass;                              //! Flag to intialize the viz at first timestep */

    std::vector <RWConfigLogSimMsg> rwInMessage;//! [-] RW data message
    STSensorIntMsg STMessage;                   //! [-] ST data message
    std::vector <THROutputSimMsg> thrOutputMessage;//! [-] Thr data message
    std::vector <SpicePlanetStateSimMsg> spiceMessage;//! [-] Spice messages
    SCPlusStatesSimMsg scPlusMessage;           //! [-] s/c plus message
//    CSSArraySensorIntMsg cssDataMessage;        //! [-] CSS message
    CSSConfigFswMsg cssConfigMessage;           //! [-] CSS config
    std::ofstream *outputStream;                //! [-] Output file stream opened in reset

    std::map<uint32_t, SpicePlanetStateSimMsg> planetData; //!< -- Internal vector of planets

};

#endif /* VIZ_INTERFACE_H */
