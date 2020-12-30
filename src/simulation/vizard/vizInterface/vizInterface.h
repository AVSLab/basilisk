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

#include "utilities/vizProtobuffer/vizMessage.pb.h"
#include <vector>
#include <fstream>
#include <map>
#include <zmq.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"

#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/messaging2/messaging2.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/simDefinitions.h"

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

    void setNumCSS(VizSpacecraftData *scData, int num);
    void setNumRW(VizSpacecraftData *scData, int num);
    void setNumTHR(VizSpacecraftData *scData, std::vector <ThrClusterMap> thrClusterList);

public:
    std::vector<VizSpacecraftData> scData;      //!< [-] vector of spacecraft data containers
    std::vector <ReadFunctor<SpicePlanetStateMsgPayload>> spiceInMsgs;   //!< [-] vector of input messages of planet Spice data
    std::vector<std::string> planetNames;       //!< [-] planet names
    Message<CameraImageMsgPayload> opnavImageOutMsg;  //!< Image output message
    int opNavMode;                              /*!< [int] Set non-zero positive value  if Unity/Viz couple in direct
                                                 communication. (1 - regular opNav, 2 - performance opNav) */
    bool saveFile;                              //!< [Bool] Set True if Vizard should save a file of the data.
    bool liveStream;                            //!< [Bool] Set True if Vizard should receive a live stream of BSK data.
    void* bskImagePtr;                          /*!< [RUN] Permanent pointer for the image to be used in BSK
                                                     without relying on ZMQ because ZMQ will free it (whenever, who knows) */

    ReadFunctor<CameraConfigMsgPayload> cameraConfInMsg;        //!< [-] msg of incoming camera data
    MsgCurrStatus cameraConfMsgStatus;                          //!< [-] msg status of incoming camera data
    CameraConfigMsgPayload cameraConfigBuffer;                 //!< [-] Camera config buffer
        
    int64_t FrameNumber;                        //!< Number of frames that have been updated for TimeStamp message
    std::string protoFilename;                  //!< Filename for where to save the protobuff message
    VizSettings settings;                       //!< [-] container for the Viz settings that can be specified from BSK

    std::string comProtocol;                    //!< Communication protocol to use when connecting to Vizard
    std::string comAddress;                     //!< Communication address to use when connecting to Vizard
    std::string comPortNumber;                  //!< Communication port number to use when connecting to Vizard
    
    ReadFunctor<EpochMsgPayload> epochInMsg;    //!< [-] simulation epoch date/time input msg
    MsgCurrStatus epochMsgStatus;                   //!< [-] ID of the epoch msg
    EpochMsgPayload epochMsgBuffer;                   //!< [-] epoch msg data

    BSKLogger bskLogger;                        //!< [-] BSK Logging object


private:
    // ZeroMQ State
    void* context;
    void* requester_socket;
    int firstPass;                                          //!< Flag to intialize the viz at first timestep

    std::vector<MsgCurrStatus>spiceInMsgStatus;             //!< [-] status of the incoming planets' spice data messages
    std::vector <SpicePlanetStateMsgPayload> spiceMessage;  //!< [-] Spice message copies
    std::ofstream *outputStream;                            //!< [-] Output file stream opened in reset
};

#endif /* VIZ_INTERFACE_H */
