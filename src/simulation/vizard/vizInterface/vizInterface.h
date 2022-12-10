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
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/simDefinitions.h"

#define VIZ_MAX_SIZE 100000



/*! Defines a data structure for the spacecraft state messages and ID's.
*/
class VizInterface : public SysModel {
public:
    VizInterface();
    ~VizInterface();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void ReadBSKMessages();
    void WriteProtobuffer(uint64_t CurrentSimNanos);
    void addCamMsgToModule(Message<CameraConfigMsgPayload> *tmpMsg);

public:
    std::vector<VizSpacecraftData> scData;      //!< [-] vector of spacecraft data containers
    std::vector <ReadFunctor<SpicePlanetStateMsgPayload>> spiceInMsgs;   //!< [-] vector of input messages of planet Spice data
    std::vector<LocationPbMsg *> locations;       //!< [] vector of ground or spacecraft locations
    std::vector<GravBodyInfo> gravBodyInformation; //!< [-] vector of gravitational body info
    std::vector<Message<CameraImageMsgPayload>*> opnavImageOutMsgs;  //!< vector of vizard instrument camera output messages
    int opNavMode;                              /*!< [int] Set non-zero positive value  if Unity/Viz couple in direct
                                                 communication. (1 - regular opNav, 2 - performance opNav) */
    bool saveFile;                              //!< [Bool] Set True if Vizard should save a file of the data.
    bool liveStream;                            //!< [Bool] Set True if Vizard should receive a live stream of BSK data.
    std::vector<void* >bskImagePtrs;            /*!< [RUN] vector of permanent pointers for the images to be used in BSK
                                                     without relying on ZMQ because ZMQ will free it (whenever, who knows) */

    std::vector<ReadFunctor<CameraConfigMsgPayload>> cameraConfInMsgs;        //!< [-] vector of incoming camera data messages
    std::vector<MsgCurrStatus> cameraConfMsgStatus;                           //!< [-] vector of msg status of incoming camera data
    std::vector<CameraConfigMsgPayload> cameraConfigBuffers;                  //!< [-] vector of Camera config buffers
        
    int64_t FrameNumber;                        //!< Number of frames that have been updated for TimeStamp message
    std::string protoFilename;                  //!< Filename for where to save the protobuff message
    VizSettings settings;                       //!< [-] container for the Viz settings that can be specified from BSK
    LiveVizSettings liveSettings;               //!< [-] container for Viz settings that are updated on each time step

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
    void requestImage(size_t camCounter, uint64_t CurrentSimNanos);  //!<   request image from Vizard and store it in output img msg

};

#endif /* VIZ_INTERFACE_H */
