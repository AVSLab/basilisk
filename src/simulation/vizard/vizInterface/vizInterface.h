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
#include "simMessages/cssConfigLogSimMsg.h"
#include "simMessages/thrOutputSimMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "../fswAlgorithms/fswMessages/thrArrayConfigFswMsg.h"
#include "utilities/bskLogging.h"
#include "simMessages/epochSimMsg.h"
#include "utilities/simDefinitions.h"
#include "../_GeneralModuleFiles/vizStructures.h"

#define VIZ_MAX_SIZE 100000



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

    std::string comProtocol;                    //!< Communication protocol to use when connecting to Vizard
    std::string comAddress;                     //!< Communication address to use when connecting to Vizard
    std::string comPortNumber;                  //!< Communication port number to use when connecting to Vizard
    
    std::string epochMsgName;                   //!< [-] name of the simulation epoch date/time msg
    MsgCurrStatus epochMsgID;                   //!< [-] ID of the epoch msg
    EpochSimMsg epochMsg;                       //!< [-] epoch msg data

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
