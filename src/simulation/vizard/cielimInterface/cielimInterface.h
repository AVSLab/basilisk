/*
 Copyright (c) 2023, University of Colorado at Boulder

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

#ifndef CIELIM_INTERFACE_H
#define CIELIM_INTERFACE_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CameraConfigMsgPayload.h"
#include "architecture/msgPayloadDefC/CameraImageMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/simDefinitions.h"
#include "utilities/vizProtobuffer/vizMessage.pb.h"
#include "simulation/vizard/cielimInterface/zmqConnector.h"
#include "simulation/vizard/_GeneralModuleFiles/vizStructures.h"

#include <fstream>
#include <map>
#include <memory>
#include <vector>
#include <zmq.h>

enum class ClosedLoopMode {
    OPEN_LOOP = 0,
    ALL_FRAMES = 1,
    REQUESTED_FRAMES = 2
};

/*! Defines a data structure for the spacecraft state messages and ID's.
*/
class CielimInterface : public SysModel {
public:
    CielimInterface();
    ~CielimInterface();
    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;
    void readBskMessages();
    void writeProtobuffer(uint64_t currentSimNanos);
    void addCamMsgToModule(Message<CameraConfigMsgPayload> *tmpMsg);
    void setOpNavMode(ClosedLoopMode mode);
    ClosedLoopMode getOpNavMode() const;
    int64_t getFrameNumber() const;

    std::vector<VizSpacecraftData> scData;      //!< [-] vector of spacecraft data containers
    std::vector<ReadFunctor<SpicePlanetStateMsgPayload>> spiceInMsgs;   //!< [-] vector of input messages of planet Spice data
    std::vector<LocationPbMsg *> locations;       //!< [] vector of ground or spacecraft locations
    std::vector<GravBodyInfo> gravBodyInformation; //!< [-] vector of gravitational body info
    std::vector<Message<CameraImageMsgPayload>*> opnavImageOutMsgs;  //!< vector of vizard instrument camera output messages

    bool saveFile{false};                              //!< [Bool] Set True if Vizard should save a file of the data.
    bool liveStream{false};                            //!< [Bool] Set True if Vizard should receive a live stream of BSK data.
    std::vector<void* >bskImagePtrs;            /*!< [RUN] vector of permanent pointers for the images to be used in BSK
                                                     without relying on ZMQ because ZMQ will free it (whenever, who knows) */

    std::vector<ReadFunctor<CameraConfigMsgPayload>> cameraConfInMsgs;        //!< [-] vector of incoming camera data messages
    std::vector<MsgCurrStatus> cameraConfMsgStatus;                           //!< [-] vector of msg status of incoming camera data
    std::vector<CameraConfigMsgPayload> cameraConfigBuffers;                  //!< [-] vector of Camera config buffers

    std::string protoFilename;                  //!< Filename for where to save the protobuff message
    VizSettings settings;                       //!< [-] container for the Viz settings that can be specified from BSK
    LiveVizSettings liveSettings;               //!< [-] container for Viz settings that are updated on each time step

    ReadFunctor<EpochMsgPayload> epochInMsg;    //!< [-] simulation epoch date/time input msg
    MsgCurrStatus epochMsgStatus;                   //!< [-] ID of the epoch msg
    EpochMsgPayload epochMsgBuffer{};                   //!< [-] epoch msg data

    BSKLogger bskLogger;                        //!< [-] BSK Logging object

private:
    ZmqConnector connector;
    ClosedLoopMode opNavMode{ClosedLoopMode::ALL_FRAMES}; /*!< [int] Set if Unity/Viz couple in direct communication. */
    int64_t frameNumber{-1}; //!< Number of frames that have been updated for TimeStamp message
    std::vector<MsgCurrStatus> spiceInMsgStatus;             //!< [-] status of the incoming planets' spice data messages
    std::vector <SpicePlanetStateMsgPayload> spiceMessage;  //!< [-] Spice message copies
    std::ofstream *outputStream{};                            //!< [-] Output file stream opened in reset
    void requestImage(size_t camCounter, uint64_t currentSimNanos);  //!<   request image from Vizard and store it in output img msg
    vizProtobufferMessage::VizMessage::VizSettingsPb*  collectVizSettings();
    vizProtobufferMessage::VizMessage::LiveVizSettingsPb* collectVizLiveSettings();

    void readRWConstellationMessages(VizSpacecraftData &scIt) const;
    void readThrusterMessages(VizSpacecraftData &scIt) const;
    void readCSSMessages(VizSpacecraftData &scIt) const;
    bool shouldRequestACameraImage(uint64_t currentSimNanos) const;
};

#endif /* CIELIM_INTERFACE_H */
