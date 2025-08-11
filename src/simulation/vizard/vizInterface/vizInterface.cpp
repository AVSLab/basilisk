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

#include <fstream>
#include <iostream>
#include <cstdio>
#include <string>

#include "vizInterface.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "architecture/utilities/astroConstants.h"

void message_buffer_deallocate(void *data, void *hint);

/*! VizInterface Constructor
 */
VizInterface::VizInterface()
{
    this->saveFile = false;
    this->liveStream = false;
    this->broadcastStream = false;
    this->noDisplay = false;
    this->FrameNumber= -1;

    this->lastSettingsSendTime = time(0); // current system time in seconds
    this->broadcastSettingsSendDelay = 2; // real-time seconds

    this->firstPass = 0;

    this->reqComProtocol = "tcp";
    this->reqComAddress = "localhost";
    this->reqPortNumber = "5556";

    this->pubComProtocol = "tcp";
    this->pubComAddress = "localhost";
    this->pubPortNumber = "5570";

    return;
}

/*! VizInterface Destructor
 */
VizInterface::~VizInterface()
{
    for (size_t c=0; c<this->opnavImageOutMsgs.size(); c++) {
        delete this->opnavImageOutMsgs.at(c);
    }

    return;
}


/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::Reset(uint64_t CurrentSimNanos)
{
    if (this->broadcastStream) {
        // Setup ZMQ for broadcast socket
        this->publisher_context = zmq_ctx_new();
        this->publisher_socket = zmq_socket(this->publisher_context, ZMQ_PUB);
        assert(this->publisher_socket);
        // Build address
        std::string text = this->pubComProtocol + "://" + this->pubComAddress + ":" + this->pubPortNumber;
        int broadcastConnect = zmq_bind(this->publisher_socket, text.c_str());
        // Error if bind failure
        if (broadcastConnect != 0) {
            int error_code = zmq_errno();
            text = "Broadcast socket did not connect correctly. ZMQ error code: " + std::to_string(error_code);
            bskLogger.bskLog(BSK_ERROR, text.c_str());
            return;
        }
        text = "Broadcasting at " + this->pubComProtocol + "://" + this->pubComAddress + ":" + this->pubPortNumber;
        bskLogger.bskLog(BSK_INFORMATION, text.c_str());
    }

    if (this->liveStream || this->noDisplay) {
        // Reset cameras
        for (size_t camCounter =0; camCounter<this->cameraConfInMsgs.size(); camCounter++) {
            this->bskImagePtrs[camCounter] = NULL;
        }
        // Setup ZMQ for 2-way socket
        this->requester_context = zmq_ctx_new();
        this->requester_socket = zmq_socket(this->requester_context, ZMQ_REQ);
        assert(this->requester_socket);
        // Build address
        std::string text = this->reqComProtocol + "://" + this->reqComAddress + ":" + this->reqPortNumber;
        int twoWayConnect = zmq_connect(this->requester_socket, text.c_str());
        // Error if connection failure
        if (twoWayConnect != 0) {
            int error_code = zmq_errno();
            text = "2-way socket did not connect correctly. ZMQ error code: " + std::to_string(error_code);
            bskLogger.bskLog(BSK_ERROR, text.c_str());
            return;
        }

        void* message = malloc(4 * sizeof(char));
        memcpy(message, "PING", 4);
        zmq_msg_t request;

        text = "Waiting for Vizard at " + this->reqComProtocol + "://" + this->reqComAddress + ":" + this->reqPortNumber;
        bskLogger.bskLog(BSK_INFORMATION, text.c_str());

        zmq_msg_init_data(&request, message, 4, message_buffer_deallocate, NULL);
        zmq_msg_send(&request, this->requester_socket, 0);
        char buffer[4];
        zmq_recv (this->requester_socket, buffer, 4, 0);
        zmq_send (this->requester_socket, "PING", 4, 0);
        bskLogger.bskLog(BSK_INFORMATION, "Basilisk-Vizard connection made");
        zmq_msg_close(&request);
    }

    std::vector<VizSpacecraftData>::iterator scIt;
    for (scIt = this->scData.begin(); scIt != this->scData.end(); scIt++)
    {
        /* Check spacecraft input message */
        if (scIt->scStateInMsg.isLinked()) {
            scIt->scStateInMsgStatus.dataFresh = false;
            scIt->scStateInMsgStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            bskLogger.bskLog(BSK_ERROR, "vizInterface: spacecraft msg not linked.");
        }

        /* Check CSS data input messages */
        {
            MsgCurrStatus cssStatus;
            cssStatus.dataFresh = false;
            cssStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            scIt->cssConfLogInMsgStatus.clear();
            scIt->cssInMessage.clear();
            for (size_t idx = 0; idx < (size_t) scIt->cssInMsgs.size(); idx++)
            {
                if (!scIt->cssInMsgs.at(idx).isLinked()) {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: CSS(%zu) msg not linked.", idx);
                }
                scIt->cssConfLogInMsgStatus.push_back(cssStatus);
                CSSConfigLogMsgPayload logMsg = {};
                scIt->cssInMessage.push_back(logMsg);
            }
        }

        /* Check RW input message */
        {
            MsgCurrStatus rwStatus;
            rwStatus.dataFresh = false;
            rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            scIt->rwInMsgStatus.clear();
            scIt->rwInMessage.clear();
            for (size_t idx = 0; idx < (size_t) scIt->rwInMsgs.size(); idx++)
            {
                if (!scIt->rwInMsgs.at(idx).isLinked()) {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: RW(%zu) msg not linked.", idx);
                }
                scIt->rwInMsgStatus.push_back(rwStatus);
                RWConfigLogMsgPayload logMsg = {};
                scIt->rwInMessage.push_back(logMsg);
            }
        }

        /* Check Thr input message */
        {
            MsgCurrStatus thrStatus;
            thrStatus.dataFresh = false;
            thrStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            int thrCounter = 0;

            for (thrCounter = 0; thrCounter < (int) scIt->thrInMsgs.size(); thrCounter++) {
                if (scIt->thrInMsgs.at(thrCounter).isLinked()) {
                    scIt->thrMsgStatus.push_back(thrStatus);
                    THROutputMsgPayload logMsg;
                    scIt->thrOutputMessage.push_back(logMsg);
                } else {
                    bskLogger.bskLog(BSK_ERROR, "vizInterface: TH(%d) msg requested but not found.", thrCounter);
                }
            }
            if (scIt->thrInfo.size() != scIt->thrInMsgs.size()) {
                bskLogger.bskLog(BSK_ERROR, "vizInterface: thrInfo vector (%d) must be the same size as thrInMsgs (%d)"
                                 , (int) scIt->thrInfo.size(), (int) scIt->thrInMsgs.size());
            }
        }
    }

    /* Check Camera input messages */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if (this->cameraConfInMsgs[camCounter].isLinked()) {
            this->cameraConfMsgStatus[camCounter].dataFresh = false;
            this->cameraConfMsgStatus[camCounter].lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        }
    }

    /* Check Spice input message */
    {
        MsgCurrStatus spiceStatus;
        spiceStatus.dataFresh = true;  // this ensures that default planet states are also used
        spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        this->spiceInMsgStatus.clear();
        this->spiceMessage.clear();
        for (long unsigned int c = 0; c<this->gravBodyInformation.size(); c++) {
            /* set default zero translation and rotation states */
            SpicePlanetStateMsgPayload logMsg = {};
            m33SetIdentity(logMsg.J20002Pfix);
            strcpy(logMsg.PlanetName, this->gravBodyInformation.at(c).bodyName.c_str());

            this->spiceInMsgStatus.push_back(spiceStatus);
            this->spiceMessage.push_back(logMsg);
        }
    }

    this->FrameNumber=-1;
    if (this->saveFile) {
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);

        /* check if file could be opened */
        if (!this->outputStream->is_open()) {
            this->saveFile = false; // turn off save file flag
            bskLogger.bskLog(BSK_ERROR, "VizInterface: Unable to open file %s for writing.", this->protoFilename.c_str());
            return;
        } else {
            bskLogger.bskLog(BSK_INFORMATION, "VizInterface: Writing data to %s", this->protoFilename.c_str());
        }
    }

    this->settings.dataFresh = true;        // reset flag to transmit Vizard settings

    this->epochMsgBuffer.year = EPOCH_YEAR;
    this->epochMsgBuffer.month = EPOCH_MONTH;
    this->epochMsgBuffer.day = EPOCH_DAY;
    this->epochMsgBuffer.hours = EPOCH_HOUR;
    this->epochMsgBuffer.minutes = EPOCH_MIN;
    this->epochMsgBuffer.seconds = EPOCH_SEC;
    this->epochMsgStatus.dataFresh = true;

    return;
}

/*! A method in which the module reads the content of all available bsk messages
 */
void VizInterface::ReadBSKMessages()
{
    std::vector<VizSpacecraftData>::iterator scIt;

    for (scIt = this->scData.begin(); scIt != this->scData.end(); scIt++)
    {
        /* Read BSK spacecraft state msg */
        if (scIt->scStateInMsg.isLinked()){
            SCStatesMsgPayload localSCStateArray;
            localSCStateArray = scIt->scStateInMsg();
            if(scIt->scStateInMsg.isWritten() && scIt->scStateInMsg.timeWritten() != scIt->scStateInMsgStatus.lastTimeTag){
                scIt->scStateInMsgStatus.lastTimeTag = scIt->scStateInMsg.timeWritten();
                scIt->scStateInMsgStatus.dataFresh = true;
            }
            scIt->scStateMsgBuffer = localSCStateArray;
        }

        /* Read BSK RW constellation msg */
        {
        for (size_t idx=0;idx< (size_t) scIt->rwInMsgs.size(); idx++) {
            if (scIt->rwInMsgs[idx].isLinked()){
                RWConfigLogMsgPayload localRWArray;
                localRWArray = scIt->rwInMsgs.at(idx)();

                if(scIt->rwInMsgs.at(idx).isWritten() &&
                   scIt->rwInMsgs.at(idx).timeWritten() != scIt->rwInMsgStatus[idx].lastTimeTag) {
                    scIt->rwInMsgStatus[idx].lastTimeTag = scIt->rwInMsgs.at(idx).timeWritten();
                    scIt->rwInMsgStatus[idx].dataFresh = true;
                    scIt->rwInMessage[idx] = localRWArray;
                }
            }
        }
        }

        /* Read incoming Thruster constellation msg */
        {
        for (size_t idx=0;idx< (size_t) scIt->thrInMsgs.size(); idx++){
            if (scIt->thrInMsgs[idx].isLinked()){
                THROutputMsgPayload localThrusterArray;
                localThrusterArray = scIt->thrInMsgs.at(idx)();
                if(scIt->thrInMsgs.at(idx).isWritten() &&
                   scIt->thrInMsgs.at(idx).timeWritten() != scIt->thrMsgStatus[idx].lastTimeTag){
                    scIt->thrMsgStatus[idx].lastTimeTag = scIt->thrInMsgs.at(idx).timeWritten();
                    scIt->thrMsgStatus[idx].dataFresh = true;
                    scIt->thrOutputMessage[idx] = localThrusterArray;
                }
            }
        }
        }

        /* Read CSS constellation log msg */
        {
        for (size_t idx=0;idx< (size_t) scIt->cssInMsgs.size(); idx++) {
            if (scIt->cssInMsgs[idx].isLinked()){
                CSSConfigLogMsgPayload localCSSMsg;
                localCSSMsg = scIt->cssInMsgs.at(idx)();
                if(scIt->cssInMsgs.at(idx).isWritten() &&
                   scIt->cssInMsgs.at(idx).timeWritten() != scIt->cssConfLogInMsgStatus[idx].lastTimeTag){
                    scIt->cssConfLogInMsgStatus[idx].lastTimeTag = scIt->cssInMsgs.at(idx).timeWritten();
                    scIt->cssConfLogInMsgStatus[idx].dataFresh = true;
                    scIt->cssInMessage[idx] = localCSSMsg;
                }
            }
        }
        }

        /* read in true trajectory line color if connected */
        if (scIt->trueTrajectoryLineColorInMsg.isLinked()) {
            if (scIt->trueTrajectoryLineColorInMsg.isWritten()) {
                ColorMsgPayload colorMsg;
                colorMsg = scIt->trueTrajectoryLineColorInMsg();
                scIt->trueTrajectoryLineColor.clear();
                for (int i=0; i<4; i++) {
                    scIt->trueTrajectoryLineColor.push_back(colorMsg.colorRGBA[i]);
                }
            }
        }

        /* read in generic sensor cmd value */
        {
            for (size_t idx=0;idx< (size_t) scIt->genericSensorList.size(); idx++) {
                if (scIt->genericSensorList[idx]->genericSensorCmdInMsg.isLinked()){
                    DeviceCmdMsgPayload deviceCmdMsgBuffer;
                    deviceCmdMsgBuffer = scIt->genericSensorList[idx]->genericSensorCmdInMsg();
                    if(scIt->genericSensorList[idx]->genericSensorCmdInMsg.isWritten()){
                        scIt->genericSensorList[idx]->genericSensorCmd = deviceCmdMsgBuffer.deviceCmd;
                    }
                }
            }
        }

        /* read in light on/off cmd value */
        {
            for (size_t idx=0;idx< (size_t) scIt->lightList.size(); idx++) {
                if (scIt->lightList[idx]->onOffCmdInMsg.isLinked()){
                    DeviceCmdMsgPayload onOffCmdMsgBuffer;
                    onOffCmdMsgBuffer = scIt->lightList[idx]->onOffCmdInMsg();
                    if(scIt->lightList[idx]->onOffCmdInMsg.isWritten()){
                        scIt->lightList[idx]->lightOn = (int) onOffCmdMsgBuffer.deviceCmd;
                    }
                }
            }
        }

        /* read in transceiver state values */
        {
            for (size_t idx=0;idx< (size_t) scIt->transceiverList.size(); idx++) {
                if (scIt->transceiverList[idx]->transceiverStateInMsgs.size() > 0) {
                    scIt->transceiverList[idx]->transceiverState = 0;
                    for (size_t idxTr=0; idxTr < (size_t) scIt->transceiverList[idx]->transceiverStateInMsgs.size(); idxTr++) {
                        if (scIt->transceiverList[idx]->transceiverStateInMsgs[idxTr].isLinked()){
                            DataNodeUsageMsgPayload stateMsgBuffer;
                            stateMsgBuffer = scIt->transceiverList[idx]->transceiverStateInMsgs[idxTr]();
                            if(scIt->transceiverList[idx]->transceiverStateInMsgs[idxTr].isWritten()){
                                /* state 0->off, 1->sending, 2->receiving, 3->sending and receiving */
                                if (stateMsgBuffer.baudRate < 0.0) {
                                    /* sending data */
                                    scIt->transceiverList[idx]->transceiverState = scIt->transceiverList[idx]->transceiverState | 1;
                                } else if (stateMsgBuffer.baudRate > 0.0) {
                                    /* receiving data */
                                    scIt->transceiverList[idx]->transceiverState = scIt->transceiverList[idx]->transceiverState | 2;
                                }
                            }
                        }
                    }
                }
            }
        }

        /* read in generic storage state values */
        {
            for (size_t idx=0;idx< (size_t) scIt->genericStorageList.size(); idx++) {
                /* read in battery device state */
                if (scIt->genericStorageList[idx]->batteryStateInMsg.isLinked()){
                    PowerStorageStatusMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = scIt->genericStorageList[idx]->batteryStateInMsg();
                    if(scIt->genericStorageList[idx]->batteryStateInMsg.isWritten()){
                        scIt->genericStorageList[idx]->currentValue = deviceStateMsgBuffer.storageLevel;
                        scIt->genericStorageList[idx]->maxValue = deviceStateMsgBuffer.storageCapacity;
                    }
                }
                /* read in data storage device state */
                if (scIt->genericStorageList[idx]->dataStorageStateInMsg.isLinked()){
                    DataStorageStatusMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = scIt->genericStorageList[idx]->dataStorageStateInMsg();
                    if(scIt->genericStorageList[idx]->dataStorageStateInMsg.isWritten()){
                        scIt->genericStorageList[idx]->currentValue = deviceStateMsgBuffer.storageLevel;
                        scIt->genericStorageList[idx]->maxValue = deviceStateMsgBuffer.storageCapacity;
                    }
                }
                /* read in fuel tank device state */
                if (scIt->genericStorageList[idx]->fuelTankStateInMsg.isLinked()){
                    FuelTankMsgPayload deviceStateMsgBuffer;
                    deviceStateMsgBuffer = scIt->genericStorageList[idx]->fuelTankStateInMsg();
                    if(scIt->genericStorageList[idx]->fuelTankStateInMsg.isWritten()){
                        scIt->genericStorageList[idx]->currentValue = deviceStateMsgBuffer.fuelMass;
                        scIt->genericStorageList[idx]->maxValue = deviceStateMsgBuffer.maxFuelMass;
                    }
                }
            }
        }

        /* read in MSM charge values */
        {
            /* read in MSM charge states */
            if (scIt->msmInfo.msmChargeInMsg.isLinked()){
                if(scIt->msmInfo.msmChargeInMsg.isWritten()){
                    ChargeMsmMsgPayload msmChargeMsgBuffer;
                    msmChargeMsgBuffer = scIt->msmInfo.msmChargeInMsg();
                    if ((size_t) msmChargeMsgBuffer.q.size() == scIt->msmInfo.msmList.size()) {
                        for (size_t idx=0;idx< (size_t) scIt->msmInfo.msmList.size(); idx++) {
                            scIt->msmInfo.msmList[idx]->currentValue = msmChargeMsgBuffer.q[idx];
                        }
                    } else {
                        bskLogger.bskLog(BSK_ERROR, "vizInterface: the number of charged in MSM message and the number of msm vizInterface spheres must be the same.");

                    }
                }
            }
        }

    } /* end of scIt loop */

    /*! Read incoming camera config msg */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
        if (this->cameraConfInMsgs[camCounter].isLinked()){
            CameraConfigMsgPayload localCameraConfigArray;
            localCameraConfigArray = this->cameraConfInMsgs[camCounter]();
            if(this->cameraConfInMsgs[camCounter].isWritten() &&
               this->cameraConfInMsgs[camCounter].timeWritten() != this->cameraConfMsgStatus[camCounter].lastTimeTag){
                this->cameraConfMsgStatus[camCounter].lastTimeTag = this->cameraConfInMsgs[camCounter].timeWritten();
                this->cameraConfMsgStatus[camCounter].dataFresh = true;
            }
            this->cameraConfigBuffers[camCounter] = localCameraConfigArray;
        }
    }

    /*! Read incoming epoch msg */
    if (this->epochInMsg.isLinked()) {
        EpochMsgPayload epochMsg_Buffer;
        epochMsg_Buffer = this->epochInMsg();
        if(this->epochInMsg.isWritten() &&
           this->epochInMsg.timeWritten() != this->epochMsgStatus.lastTimeTag){
            this->epochMsgStatus.lastTimeTag = this->epochInMsg.timeWritten();
            this->epochMsgStatus.dataFresh = true;
            this->epochMsgBuffer = epochMsg_Buffer;
        }
    }

    /*! Read BSK Spice constellation msg */
    {
    for(size_t i=0; i < this->spiceInMsgs.size(); i++)
    {
        if (this->spiceInMsgs.at(i).isLinked()){
            // If the spice msg is not linked then the default zero planet emphemeris is used
            SpicePlanetStateMsgPayload localSpiceArray;
            localSpiceArray = this->spiceInMsgs.at(i)();
            if(this->spiceInMsgs.at(i).isWritten() &&
               this->spiceInMsgs.at(i).timeWritten() != this->spiceInMsgStatus[i].lastTimeTag){
                this->spiceInMsgStatus[i].lastTimeTag = this->spiceInMsgs.at(i).timeWritten();
                this->spiceInMsgStatus[i].dataFresh = true;
                this->spiceMessage[i] = localSpiceArray;
            }
        }
    }
    }

    return;
}

/*! The method in which the vizInterface writes a protobuffer with the information from the simulation.
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::WriteProtobuffer(uint64_t CurrentSimNanos)
{
    vizProtobufferMessage::VizMessage* message = new vizProtobufferMessage::VizMessage;

    /*! Send the Vizard settings according to interval set by broadcastSettingsSendDelay field */
    this->now = time(0);
    if (this->broadcastStream && (this->now - this->lastSettingsSendTime) >= this->broadcastSettingsSendDelay) {
        this->settings.dataFresh = true;
    }
    if (this->settings.dataFresh) {
        vizProtobufferMessage::VizMessage::VizSettingsPb* vizSettings;
        vizSettings = new vizProtobufferMessage::VizMessage::VizSettingsPb;

        // define the viz ambient light setting
        vizSettings->set_ambient(this->settings.ambient);
        if (this->settings.ambient > 8.0 ||
            (this->settings.ambient < 0.0 && this->settings.ambient != -1.0)) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard ambient light value must be within [0,8].  A value of %f was received.", this->settings.ambient);
        }

        // define if osculating orbit lines should be shown
        vizSettings->set_orbitlineson(this->settings.orbitLinesOn);
        if (abs(this->settings.orbitLinesOn)>2) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard orbitLinesOn flag must be either -1, 0, 1 or 2.  A value of %d was received.", this->settings.orbitLinesOn);
        }

        // define if true orbit lines should be shown
        vizSettings->set_truetrajectorylineson(this->settings.trueTrajectoryLinesOn);
        if (abs(this->settings.trueTrajectoryLinesOn)>5) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard trueTrajectoryLinesOn flag must be either -1, 0, 1 2, 3, 4 or 5.  A value of %d was received.", this->settings.trueTrajectoryLinesOn);
        }

        // define if spacecraft axes should be shown
        vizSettings->set_spacecraftcson(this->settings.spacecraftCSon);
        if (abs(this->settings.spacecraftCSon)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard spacecraftCSon flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.spacecraftCSon);
        }

        // define if planet axes should be shown
        vizSettings->set_planetcson(this->settings.planetCSon);
        if (abs(this->settings.planetCSon)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard planetCSon flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.planetCSon);
        }

        // define the skyBox variable
        if (this->settings.skyBox.length() > 0) {
            vizSettings->set_skybox(this->settings.skyBox);
        }

        // define any pointing lines for Vizard
        for (size_t idx = 0; idx < this->settings.pointLineList.size(); idx++) {
            vizProtobufferMessage::VizMessage::PointLine* pl = vizSettings->add_pointlines();
            pl->set_tobodyname(this->settings.pointLineList[idx].toBodyName);
            pl->set_frombodyname(this->settings.pointLineList[idx].fromBodyName);
            for (int i=0; i<4; i++){
                pl->add_linecolor(this->settings.pointLineList[idx].lineColor[i]);
            }
        }

        // define any keep in/out cones for Vizard
        for (size_t idx = 0; idx < this->settings.coneList.size(); idx++) {
            vizProtobufferMessage::VizMessage::KeepOutInCone* cone = vizSettings->add_keepoutincones();
            cone->set_iskeepin(this->settings.coneList[idx].isKeepIn);
            for (int i=0; i<3; i++) {
                cone->add_position(this->settings.coneList[idx].position_B[i]);
                cone->add_normalvector(this->settings.coneList[idx].normalVector_B[i]);
            }
            cone->set_incidenceangle(this->settings.coneList[idx].incidenceAngle*R2D);  // Unity expects degrees
            cone->set_coneheight(this->settings.coneList[idx].coneHeight);
            cone->set_tobodyname(this->settings.coneList[idx].toBodyName);
            cone->set_frombodyname(this->settings.coneList[idx].fromBodyName);
            for (int i=0; i<4; i++){
                cone->add_conecolor(this->settings.coneList[idx].coneColor[i]);
            }
            cone->set_conename(this->settings.coneList[idx].coneName);
        }

        // define if camera boresight line should be shown
        vizSettings->set_viewcameraboresighthud(this->settings.viewCameraBoresightHUD);
        if (abs(this->settings.viewCameraBoresightHUD)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard viewCameraBoresightHUD flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.viewCameraBoresightHUD);
        }

        // define if camera cone should be shown
        vizSettings->set_viewcameraconehud(this->settings.viewCameraConeHUD);
        if (abs(this->settings.viewCameraConeHUD)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard viewCameraConeHUD flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.viewCameraConeHUD);
        }

        // define if coordinate system labels should be shown
        vizSettings->set_showcslabels(this->settings.showCSLabels);
        if (abs(this->settings.showCSLabels)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showCSLabels flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showCSLabels);
        }

        // define if celestial body labels should be shown
        vizSettings->set_showcelestialbodylabels(this->settings.showCelestialBodyLabels);
        if (abs(this->settings.showCelestialBodyLabels)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showCelestialBodyLabels flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showCelestialBodyLabels);
        }

        // define if spacecraft labels should be shown
        vizSettings->set_showspacecraftlabels(this->settings.showSpacecraftLabels);
        if (abs(this->settings.showSpacecraftLabels)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showSpacecraftLabels flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showSpacecraftLabels);
        }

        // define if camera labels should be shown
        vizSettings->set_showcameralabels(this->settings.showCameraLabels);
        if (abs(this->settings.showCameraLabels)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showCameraLabels flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showCameraLabels);
        }

        // define the GUI scaling factor
        vizSettings->set_customguireferenceheight(this->settings.customGUIReferenceHeight);
        if (this->settings.customGUIReferenceHeight<300.0 && abs(this->settings.customGUIReferenceHeight)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard customGUIReferenceHeight flag must be either -1 or > 300.  A value of %d was received.", this->settings.customGUIReferenceHeight);
        }

        // define default spacecraft sprite behavior
        vizSettings->set_defaultspacecraftsprite(this->settings.defaultSpacecraftSprite);

        // define if spacecraft should be shown as sprites
        vizSettings->set_showspacecraftassprites(this->settings.showSpacecraftAsSprites);
        if (abs(this->settings.showSpacecraftAsSprites)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showSpacecraftAsSprites flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showSpacecraftAsSprites);
        }

        // define if celestial objects should be shown as sprites
        vizSettings->set_showcelestialbodiesassprites(this->settings.showCelestialBodiesAsSprites);
        if (abs(this->settings.showCelestialBodiesAsSprites)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showCelestialBodiesAsSprites flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showCelestialBodiesAsSprites);
        }

        // define if the time should be shown using a 24h clock
        vizSettings->set_show24hrclock(this->settings.show24hrClock);
        if (abs(this->settings.show24hrClock)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard show24hrClock flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.show24hrClock);
        }

        // define if the data frame rate should be shown
        vizSettings->set_showdataratedisplay(this->settings.showDataRateDisplay);
        if (abs(this->settings.showDataRateDisplay)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard showDataRateDisplay flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.showDataRateDisplay);
        }

        // define the keyboard driven camera rates
        vizSettings->set_keyboardangularrate(this->settings.keyboardAngularRate*R2D);
        vizSettings->set_keyboardzoomrate(this->settings.keyboardZoomRate);

        // add default thrust plume color
        if (this->settings.defaultThrusterColor[0] >= 0) {
            for (int i=0; i<4; i++){
                vizSettings->add_defaultthrustercolor(this->settings.defaultThrusterColor[i]);
            }
        }

        vizSettings->set_defaultthrusterplumelifescalar(this->settings.defaultThrusterPlumeLifeScalar);
        vizSettings->set_orbitlinesegments(this->settings.orbitLineSegments);
        vizSettings->set_relativeorbitrange(this->settings.relativeOrbitRange);
        vizSettings->set_showhillframe(this->settings.showHillFrame);
        vizSettings->set_showvelocityframe(this->settings.showVelocityFrame);
        vizSettings->set_relativeorbitframe(this->settings.relativeOrbitFrame);
        vizSettings->set_maincameratarget(this->settings.mainCameraTarget);
        vizSettings->set_spacecraftshadowbrightness(this->settings.spacecraftShadowBrightness);
        vizSettings->set_spacecraftsizemultiplier(this->settings.spacecraftSizeMultiplier);
        vizSettings->set_spacecrafthelioviewsizemultiplier(this->settings.spacecraftHelioViewSizeMultiplier);
        vizSettings->set_forcestartatspacecraftlocalview(this->settings.forceStartAtSpacecraftLocalView);
        vizSettings->set_usesimplelocationmarkers(this->settings.useSimpleLocationMarkers);
        vizSettings->set_showlocationcommlines(this->settings.showLocationCommLines);
        vizSettings->set_showlocationcones(this->settings.showLocationCones);
        vizSettings->set_showlocationlabels(this->settings.showLocationLabels);
        vizSettings->set_atmospheresoff(this->settings.atmospheresOff);
        vizSettings->set_scviewtoplanetviewboundarymultiplier(this->settings.scViewToPlanetViewBoundaryMultiplier);
        vizSettings->set_planetviewtohelioviewboundarymultiplier(this->settings.planetViewToHelioViewBoundaryMultiplier);
        vizSettings->set_sunintensity(this->settings.sunIntensity);
        vizSettings->set_attenuatesunlightwithdistance(this->settings.attenuateSunLightWithDistance);
        vizSettings->set_showlightlabels(this->settings.showLightLabels);
        vizSettings->set_celestialbodyhelioviewsizemultiplier(this->settings.celestialBodyHelioViewSizeMultiplier);
        vizSettings->set_showmissiontime(this->settings.showMissionTime);
        vizSettings->set_keyboardliveinput(this->settings.keyboardLiveInput);
        vizSettings->set_messagebuffersize(this->settings.messageBufferSize);
        vizSettings->set_truepathrelativebody(this->settings.truePathRelativeBody);
        vizSettings->set_truepathrotatingframe(this->settings.truePathRotatingFrame);
        vizSettings->set_truepathfixedframe(this->settings.truePathFixedFrame);
        vizSettings->set_showquadmaplabels(this->settings.showQuadMapLabels);
        vizSettings->set_spacecraftorbitlinewidth(this->settings.spacecraftOrbitLineWidth);
        vizSettings->set_celestialbodyorbitlinewidth(this->settings.celestialBodyOrbitLineWidth);

        // define actuator GUI settings
        for (size_t idx = 0; idx < this->settings.actuatorGuiSettingsList.size(); idx++) {
            vizProtobufferMessage::VizMessage::ActuatorSettings* al = vizSettings->add_actuatorsettings();
            al->set_spacecraftname(this->settings.actuatorGuiSettingsList[idx].spacecraftName);
            al->set_viewthrusterpanel(this->settings.actuatorGuiSettingsList[idx].viewThrusterPanel);
            al->set_viewthrusterhud(this->settings.actuatorGuiSettingsList[idx].viewThrusterHUD);
            al->set_viewrwpanel(this->settings.actuatorGuiSettingsList[idx].viewRWPanel);
            al->set_viewrwhud(this->settings.actuatorGuiSettingsList[idx].viewRWHUD);
            al->set_showthrusterlabels(this->settings.actuatorGuiSettingsList[idx].showThrusterLabels);
            al->set_showrwlabels(this->settings.actuatorGuiSettingsList[idx].showRWLabels);
        }

        // define instrument GUI settings
        for (size_t idx = 0; idx < this->settings.instrumentGuiSettingsList.size(); idx++) {
            vizProtobufferMessage::VizMessage::InstrumentSettings* il = vizSettings->add_instrumentsettings();
            il->set_spacecraftname(this->settings.instrumentGuiSettingsList[idx].spacecraftName);
            il->set_viewcsspanel(this->settings.instrumentGuiSettingsList[idx].viewCSSPanel);
            il->set_viewcsscoverage(this->settings.instrumentGuiSettingsList[idx].viewCSSCoverage);
            il->set_viewcssboresight(this->settings.instrumentGuiSettingsList[idx].viewCSSBoresight);
            il->set_showcsslabels(this->settings.instrumentGuiSettingsList[idx].showCSSLabels);
            il->set_showgenericsensorlabels(this->settings.instrumentGuiSettingsList[idx].showGenericSensorLabels);
            il->set_showtransceiverlabels(this->settings.instrumentGuiSettingsList[idx].showTransceiverLabels);
            il->set_showtransceiverfrustrum(this->settings.instrumentGuiSettingsList[idx].showTransceiverFrustrum);
            il->set_showgenericstoragepanel(this->settings.instrumentGuiSettingsList[idx].showGenericStoragePanel);
            il->set_showmultishapelabels(this->settings.instrumentGuiSettingsList[idx].showMultiShapeLabels);
        }


        // define scene object custom object shapes
        for (size_t idx = 0; idx < this->settings.customModelList.size(); idx++) {
            vizProtobufferMessage::VizMessage::CustomModel* cm = vizSettings->add_custommodels();
            CustomModel *cmp = &(this->settings.customModelList[idx]);
            cm->set_modelpath(cmp->modelPath);
            for (size_t i=0; i<cmp->simBodiesToModify.size(); i++) {
                cm->add_simbodiestomodify(cmp->simBodiesToModify[i]);
            }
            for (size_t i=0; i<3;i++) {
                cm->add_offset(cmp->offset[i]);
                cm->add_rotation(cmp->rotation[i]*R2D);  // Unity expects degrees
                cm->add_scale(cmp->scale[i]);
            }
            cm->set_customtexturepath(cmp->customTexturePath);
            cm->set_normalmappath(cmp->normalMapPath);
            cm->set_shader(cmp->shader);
            for (size_t i=0; i<cmp->color.size(); i++) {
                cm->add_color(cmp->color[i]);
            }

        }

        // define camera settings
        for (size_t idx = 0; idx < this->settings.stdCameraList.size(); idx++){
            vizProtobufferMessage::VizMessage::StandardCameraSettings* sc = vizSettings->add_standardcamerasettings();
            StdCameraSettings *scp = &(this->settings.stdCameraList[idx]);
            sc->set_spacecraftname(scp->spacecraftName);
            sc->set_setmode(scp->setMode);
            sc->set_showhudelementsinimage(scp->showHUDElementsInImage);
            if (scp->fieldOfView < 0)
                sc->set_fieldofview(-1.0);
            else {
                sc->set_fieldofview(scp->fieldOfView*R2D); // Unity expects degrees
            }
            sc->set_bodytarget(scp->bodyTarget);
            sc->set_setview(scp->setView);
            for (size_t i=0; i<3; i++){
                sc->add_pointingvector(scp->pointingVector_B[i]);
            }
            if (v3Norm(scp->position_B) > 0.00001) {
                for (size_t i=0; i<3; i++) {
                    sc->add_position(scp->position_B[i]);
                }
            }
            sc->set_displayname(scp->displayName);
        }

        message->set_allocated_settings(vizSettings);

        this->lastSettingsSendTime = now;
    }

    /*! Send the Vizard live settings */
    vizProtobufferMessage::VizMessage::LiveVizSettingsPb* liveVizSettings;
    liveVizSettings = new vizProtobufferMessage::VizMessage::LiveVizSettingsPb;

    // define any pointing lines for Vizard
    for (size_t idx = 0; idx < this->liveSettings.targetLineList.size(); idx++) {
        vizProtobufferMessage::VizMessage::PointLine* pl = liveVizSettings->add_targetlines();
        pl->set_tobodyname(this->liveSettings.targetLineList[idx].toBodyName);
        pl->set_frombodyname(this->liveSettings.targetLineList[idx].fromBodyName);
        for (int i=0; i<4; i++){
            pl->add_linecolor(this->liveSettings.targetLineList[idx].lineColor[i]);
        }
    }
    liveVizSettings->set_relativeorbitchief(this->liveSettings.relativeOrbitChief);
    liveVizSettings->set_terminatevizard(this->liveSettings.terminateVizard);
    message->set_allocated_livesettings(liveVizSettings);


    // Send dialog panel info to Vizard
    for(size_t k=0; k<this->vizEventDialogs.size(); k++)
    {
        vizProtobufferMessage::VizEventDialog* panel = message->add_vizeventdialogs();
        panel->set_eventhandlerid(this->vizEventDialogs.at(k)->eventHandlerID);
        panel->set_displaystring(this->vizEventDialogs.at(k)->displayString);
        for (size_t idx=0; idx<this->vizEventDialogs.at(k)->userOptions.size(); idx++) {
            panel->add_useroptions(this->vizEventDialogs.at(k)->userOptions[idx]);
        }
        panel->set_durationofdisplay(this->vizEventDialogs.at(k)->durationOfDisplay);
        panel->set_usesimelapsedtimeforduration(this->vizEventDialogs.at(k)->useSimElapsedTimeForDuration);
        panel->set_useconfirmationpanel(this->vizEventDialogs.at(k)->useConfirmationPanel);
        panel->set_hideonselection(this->vizEventDialogs.at(k)->hideOnSelection);
        panel->set_dialogformat(this->vizEventDialogs.at(k)->dialogFormat);

    }
    this->vizEventDialogs.clear(); // Panel requests should only send to Vizard once

    /*! Write timestamp output msg */
    vizProtobufferMessage::VizMessage::TimeStamp* time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->FrameNumber);
    time->set_simtimeelapsed((double) CurrentSimNanos);
    message->set_allocated_currenttime(time);

    /*! write epoch msg */
    if (this->epochMsgStatus.dataFresh) {
        vizProtobufferMessage::VizMessage::EpochDateTime* epoch = new vizProtobufferMessage::VizMessage::EpochDateTime;
        epoch->set_year(this->epochMsgBuffer.year);
        epoch->set_month(this->epochMsgBuffer.month);
        epoch->set_day(this->epochMsgBuffer.day);
        epoch->set_hours(this->epochMsgBuffer.hours);
        epoch->set_minutes(this->epochMsgBuffer.minutes);
        epoch->set_seconds(this->epochMsgBuffer.seconds);
        message->set_allocated_epoch(epoch);
        this->epochMsgStatus.dataFresh = false;
    }

    /*! write the Locations protobuffer messages */
    std::vector<LocationPbMsg *>::iterator glIt;
    for (glIt = locations.begin(); glIt != locations.end(); glIt++) {
        vizProtobufferMessage::VizMessage::Location* glp = message->add_locations();
        glp->set_stationname((*glIt)->stationName);
        glp->set_parentbodyname((*glIt)->parentBodyName);
        glp->set_fieldofview((*glIt)->fieldOfView*R2D);
        glp->set_range((*glIt)->range);
        for (int i=0; i<3; i++) {
            glp->add_r_gp_p((*glIt)->r_GP_P[i]);
            glp->add_ghat_p((*glIt)->gHat_P[i]);
        }
        for (int i=0; i<4; i++) {
            glp->add_color((*glIt)->color[i]);
        }
        glp->set_markerscale((*glIt)->markerScale);
        glp->set_ishidden((*glIt)->isHidden);
    }

    // Write QuadMap messages
    for (size_t k=0; k<this->quadMaps.size(); k++)
    {
        vizProtobufferMessage::VizMessage::QuadMap* qm = message->add_quadmaps();
        qm->set_id(this->quadMaps.at(k)->ID);
        qm->set_parentbodyname(this->quadMaps.at(k)->parentBodyName);
        for (size_t idx=0; idx<this->quadMaps.at(k)->vertices.size(); idx++) {
            qm->add_vertices(this->quadMaps.at(k)->vertices[idx]);
        }
        for (size_t idx=0; idx<this->quadMaps.at(k)->color.size(); idx++) {
            qm->add_color(this->quadMaps.at(k)->color[idx]);
        }
        qm->set_ishidden(this->quadMaps.at(k)->isHidden);
        qm->set_label(this->quadMaps.at(k)->label);
    }
    this->quadMaps.clear(); // QuadMaps should only send to Vizard once

    std::vector<VizSpacecraftData>::iterator scIt;
    for (scIt = scData.begin(); scIt != scData.end(); scIt++)
    {
        /*! Write spacecraft state output msg */
        if (scIt->scStateInMsg.isLinked() && scIt->scStateInMsgStatus.dataFresh){
            vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
            scp->set_spacecraftname(scIt->spacecraftName);
            scp->set_parentspacecraftname(scIt->parentSpacecraftName);
            for (int i=0; i<3; i++){
                scp->add_position(scIt->scStateMsgBuffer.r_BN_N[i]);
                scp->add_velocity(scIt->scStateMsgBuffer.v_BN_N[i]);
                scp->add_rotation(scIt->scStateMsgBuffer.sigma_BN[i]);
            }
//            scIt->scStateInMsgID.dataFresh = false;

            /* Write the SC sprite string */
            scp->set_spacecraftsprite(scIt->spacecraftSprite);

            /*! Write RW output msg */
            for (size_t idx =0; idx < (size_t) scIt->rwInMsgs.size(); idx++)
            {
                if (scIt->rwInMsgs[idx].isLinked() && scIt->rwInMsgStatus[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::ReactionWheel* rwheel = scp->add_reactionwheels();
                    rwheel->set_wheelspeed(scIt->rwInMessage[idx].Omega);
                    rwheel->set_maxspeed(scIt->rwInMessage[idx].Omega_max);
                    rwheel->set_wheeltorque(scIt->rwInMessage[idx].u_current);
                    rwheel->set_maxtorque(scIt->rwInMessage[idx].u_max);
                    for (int i=0; i<3; i++){
                        rwheel->add_position(scIt->rwInMessage[idx].rWB_B[i]);
                        rwheel->add_spinaxisvector(scIt->rwInMessage[idx].gsHat_B[i]);
                    }
                    //rwInMsgID[idx].dataFresh = false;
                }
            }

            /*! Write Thr output msg */
            for (size_t idx =0; idx < (size_t) scIt->thrInMsgs.size(); idx++)
            {
                if (scIt->thrInMsgs[idx].isLinked() && scIt->thrMsgStatus[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::Thruster* thr = scp->add_thrusters();
                    thr->set_maxthrust(scIt->thrOutputMessage[idx].maxThrust);
                    thr->set_currentthrust(scIt->thrOutputMessage[idx].thrustForce);
                    thr->set_thrustertag(scIt->thrInfo[idx].thrTag);
                    if (scIt->thrInfo[idx].color[0] >= 0) {
                        for (int i=0; i<4; i++) {
                            thr->add_color(scIt->thrInfo[idx].color[i]);
                        }
                    }
                    for (int i=0; i<3; i++){
                        thr->add_position(scIt->thrOutputMessage[idx].thrusterLocation[i]);
                        thr->add_thrustvector(scIt->thrOutputMessage[idx].thrusterDirection[i]);
                    }
                    //thrMsgID[idx].dataFresh = false;
                }
            }

            // Write CSS output msg
            for (size_t idx =0; idx < (size_t) scIt->cssInMsgs.size(); idx++)
            {
                if (scIt->cssInMsgs[idx].isLinked() && scIt->cssConfLogInMsgStatus[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::CoarseSunSensor* css = scp->add_css();
                    for (int j=0; j<3; j++){
                        css->add_normalvector(scIt->cssInMessage[idx].nHat_B[j]);
                        css->add_position(scIt->cssInMessage[idx].r_B[j]);
                    }
                    css->set_currentmsmt(scIt->cssInMessage[idx].signal);
                    css->set_maxmsmt(scIt->cssInMessage[idx].maxSignal);
                    css->set_minmsmt(scIt->cssInMessage[idx].minSignal);
                    css->set_cssgroupid(scIt->cssInMessage[idx].CSSGroupID);
                    css->set_fieldofview(scIt->cssInMessage[idx].fov*2*R2D);  /* must be edge to edge fov in degrees */

                    //cssConfLogInMsgId[idx].dataFresh = false;
                }
            }

            // Write generic sensor messages
            for (size_t idx =0; idx < (size_t) scIt->genericSensorList.size(); idx++) {
                vizProtobufferMessage::VizMessage::GenericSensor* gs = scp->add_genericsensors();

                for (int j=0; j<3; j++) {
                    gs->add_position(scIt->genericSensorList[idx]->r_SB_B[j]);
                    gs->add_normalvector(scIt->genericSensorList[idx]->normalVector[j]);
                }
                for (uint64_t j=0; j<scIt->genericSensorList[idx]->fieldOfView.size(); j++) {
                    gs->add_fieldofview(scIt->genericSensorList[idx]->fieldOfView[j]*R2D);
                }
                gs->set_ishidden(scIt->genericSensorList[idx]->isHidden);
                gs->set_size(scIt->genericSensorList[idx]->size);
                gs->set_label(scIt->genericSensorList[idx]->label);
                for (uint64_t j=0; j<scIt->genericSensorList[idx]->color.size(); j++) {
                    gs->add_color(scIt->genericSensorList[idx]->color[j]);
                }
                gs->set_activitystatus((int) scIt->genericSensorList[idx]->genericSensorCmd);
            }

            // Write Ellipsoid messages
            for (size_t idx =0; idx < (size_t) scIt->ellipsoidList.size(); idx++) {
                vizProtobufferMessage::VizMessage::Ellipsoid* el = scp->add_ellipsoids();
                el->set_ison(scIt->ellipsoidList[idx]->isOn);
                el->set_usebodyframe(scIt->ellipsoidList[idx]->useBodyFrame);
                for (int j=0; j<3; j++) {
                    el->add_position(scIt->ellipsoidList[idx]->position[j]);
                    el->add_semimajoraxes(scIt->ellipsoidList[idx]->semiMajorAxes[j]);
                }
                for (uint64_t j=0; j<scIt->ellipsoidList[idx]->color.size(); j++) {
                    el->add_color(scIt->ellipsoidList[idx]->color[j]);
                }
                el->set_showgridlines(scIt->ellipsoidList[idx]->showGridLines);
            }


            // Write transceiver messages
            for (size_t idx =0; idx < (size_t) scIt->transceiverList.size(); idx++) {
                vizProtobufferMessage::VizMessage::Transceiver* tr = scp->add_transceivers();

                for (int j=0; j<3; j++) {
                    tr->add_position(scIt->transceiverList[idx]->r_SB_B[j]);
                    tr->add_normalvector(scIt->transceiverList[idx]->normalVector[j]);
                }
                tr->set_fieldofview(scIt->transceiverList[idx]->fieldOfView*R2D);
                tr->set_ishidden(scIt->transceiverList[idx]->isHidden);
                tr->set_label(scIt->transceiverList[idx]->label);
                for (uint64_t j=0; j<scIt->transceiverList[idx]->color.size(); j++) {
                    tr->add_color(scIt->transceiverList[idx]->color[j]);
                }
                tr->set_transmitstatus(scIt->transceiverList[idx]->transceiverState);
                tr->set_animationspeed(scIt->transceiverList[idx]->animationSpeed);
            }

            // Write generic storage device messages
            for (size_t idx =0; idx < (size_t) scIt->genericStorageList.size(); idx++) {
                vizProtobufferMessage::VizMessage::GenericStorage* gsd = scp->add_storagedevices();

                gsd->set_label(scIt->genericStorageList[idx]->label);
                gsd->set_currentvalue(scIt->genericStorageList[idx]->currentValue);
                gsd->set_maxvalue(scIt->genericStorageList[idx]->maxValue);
                gsd->set_units(scIt->genericStorageList[idx]->units);
                for (uint64_t j=0; j<scIt->genericStorageList[idx]->color.size(); j++) {
                    gsd->add_color(scIt->genericStorageList[idx]->color[j]);
                }
                for (uint64_t j=0; j<scIt->genericStorageList[idx]->thresholds.size(); j++) {
                    gsd->add_thresholds(scIt->genericStorageList[idx]->thresholds[j]);
                }

            }

            // Write light device messages
            for (size_t idx =0; idx < (size_t) scIt->lightList.size(); idx++) {
                vizProtobufferMessage::VizMessage::Light* ld = scp->add_lights();

                ld->set_label(scIt->lightList[idx]->label);
                for (uint64_t j=0; j<3; j++) {
                    ld->add_position(scIt->lightList[idx]->position[j]);
                    ld->add_normalvector(scIt->lightList[idx]->normalVector[j]);
                }
                /* light on integer can only be 1, 0 or -1*/
                if (scIt->lightList[idx]->lightOn > 1) {
                    scIt->lightList[idx]->lightOn = 1;
                } else if (scIt->lightList[idx]->lightOn < 0) {
                    scIt->lightList[idx]->lightOn = -1;
                }
                ld->set_lighton(scIt->lightList[idx]->lightOn);
                ld->set_fieldofview(scIt->lightList[idx]->fieldOfView*R2D);
                ld->set_range(scIt->lightList[idx]->range);
                ld->set_intensity(scIt->lightList[idx]->intensity);
                ld->set_showlightmarker(scIt->lightList[idx]->showLightMarker);
                ld->set_markerdiameter(scIt->lightList[idx]->markerDiameter);
                for (uint64_t j=0; j<scIt->lightList[idx]->color.size(); j++) {
                    ld->add_color(scIt->lightList[idx]->color[j]);
                }
                ld->set_gammasaturation(scIt->lightList[idx]->gammaSaturation);
                ld->set_showlensflare(scIt->lightList[idx]->showLensFlare);
                ld->set_lensflarebrightness(scIt->lightList[idx]->lensFlareBrightness);
                ld->set_lensflarefadespeed(scIt->lightList[idx]->lensFlareFadeSpeed);
            }

            /* Write the SC sprite string */
            scp->set_modeldictionarykey(scIt->modelDictionaryKey);

            /* Write the sc logoTexture string */
            scp->set_logotexture(scIt->logoTexture);

            /* set spacecraft osculating orbit line color */
            for (size_t i=0; i<scIt->oscOrbitLineColor.size(); i++){
                scp->add_oscorbitlinecolor(scIt->oscOrbitLineColor[i]);
            }

            /* set spacecraft true orbit line color */
            for (size_t i=0; i<scIt->trueTrajectoryLineColor.size(); i++){
                scp->add_truetrajectorylinecolor(scIt->trueTrajectoryLineColor[i]);
            }

            // Write Multi-Shape-Model messages
            for (size_t idx =0; idx < (size_t) scIt->msmInfo.msmList.size(); idx++) {
                vizProtobufferMessage::VizMessage::MultiShape* msmp = scp->add_multishapes();

                msmp->set_ison(scIt->msmInfo.msmList[idx]->isOn);
                for (uint64_t j=0; j<3; j++) {
                    msmp->add_position(scIt->msmInfo.msmList[idx]->position[j]);
                }
                msmp->set_radius(scIt->msmInfo.msmList[idx]->radius);
                msmp->set_currentvalue(scIt->msmInfo.msmList[idx]->currentValue);
                msmp->set_maxvalue(scIt->msmInfo.msmList[idx]->maxValue);
                for (uint64_t j=0; j<scIt->msmInfo.msmList[idx]->positiveColor.size(); j++) {
                    msmp->add_positivecolor(scIt->msmInfo.msmList[idx]->positiveColor[j]);
                }
                for (uint64_t j=0; j<scIt->msmInfo.msmList[idx]->negativeColor.size(); j++) {
                    msmp->add_negativecolor(scIt->msmInfo.msmList[idx]->negativeColor[j]);
                }
                msmp->set_neutralopacity(scIt->msmInfo.msmList[idx]->neutralOpacity);
                msmp->set_shape(scIt->msmInfo.msmList[idx]->shape);
                for (int j=0; j<3; j++) {
                    msmp->add_dimensions(scIt->msmInfo.msmList[idx]->dimensions[j]);
                }
                for (int j=0; j<3; j++) {
                    msmp->add_rotation(scIt->msmInfo.msmList[idx]->rotation[j]);
                }
            }

        }
    }

    /*! Write camera output msg */
    for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++){
        if ((this->cameraConfInMsgs[camCounter].isLinked() && this->cameraConfMsgStatus[camCounter].dataFresh)
            || this->cameraConfigBuffers[camCounter].cameraID >= 0){
            /*! This corrective rotation allows unity to place the camera as is expected by the python setting. Unity has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame: z point, x horizontal, y vertical (down) */
            double sigma_CuC[3], unityCameraMRP[3]; /*! Cu is the unity Camera frame */
            v3Set(1./3, 1./3, -1./3, sigma_CuC);
            addMRP(this->cameraConfigBuffers[camCounter].sigma_CB, sigma_CuC, unityCameraMRP);
            vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
            for (int j=0; j<3; j++){
                if (j < 2){
                    camera->add_resolution(this->cameraConfigBuffers[camCounter].resolution[j]);
                }
                camera->add_cameradir_b(unityCameraMRP[j]);
                camera->add_camerapos_b(this->cameraConfigBuffers[camCounter].cameraPos_B[j]);
            }
            camera->set_renderrate(this->cameraConfigBuffers[camCounter].renderRate);        // Unity expects nano-seconds between images
            camera->set_cameraid(this->cameraConfigBuffers[camCounter].cameraID);
            camera->set_fieldofview(this->cameraConfigBuffers[camCounter].fieldOfView*R2D);  // Unity expects degrees
            camera->set_skybox(this->cameraConfigBuffers[camCounter].skyBox);
            camera->set_parentname(this->cameraConfigBuffers[camCounter].parentName);
            camera->set_postprocessingon(this->cameraConfigBuffers[camCounter].postProcessingOn);
            camera->set_ppfocusdistance(this->cameraConfigBuffers[camCounter].ppFocusDistance);
            camera->set_ppaperture(this->cameraConfigBuffers[camCounter].ppAperture);
            camera->set_ppfocallength(this->cameraConfigBuffers[camCounter].ppFocalLength*1000.); // Unity expects mm
            camera->set_ppmaxblursize(this->cameraConfigBuffers[camCounter].ppMaxBlurSize);
            camera->set_updatecameraparameters(this->cameraConfigBuffers[camCounter].updateCameraParameters);
            camera->set_rendermode(this->cameraConfigBuffers[camCounter].renderMode);
            if (this->cameraConfigBuffers[camCounter].depthMapClippingPlanes[0] > 0.0) {
                for (int j=0; j<2; j++) {
                    camera->add_depthmapclippingplanes(this->cameraConfigBuffers[camCounter].depthMapClippingPlanes[j]);
                }
            }
            camera->set_showhudelementsinimage(this->cameraConfigBuffers[camCounter].showHUDElementsInImage);
        }
    }

    /*! Write spice output msgs */
    for(size_t k=0; k<this->gravBodyInformation.size(); k++)
    {
        if (this->spiceInMsgStatus[k].dataFresh){
            vizProtobufferMessage::VizMessage::CelestialBody* spice = message->add_celestialbodies();
            spice->set_bodyname(this->gravBodyInformation.at(k).bodyName);
            spice->set_mu(this->gravBodyInformation.at(k).mu/1e9);  /* must be in km^3/s^2 */
            spice->set_radiuseq(this->gravBodyInformation.at(k).radEquator/1000.);  /* must be in km */
            spice->set_radiusratio(this->gravBodyInformation.at(k).radiusRatio);
            spice->set_modeldictionarykey(this->gravBodyInformation.at(k).modelDictionaryKey);
            for (int i=0; i<3; i++){
                spice->add_position(this->spiceMessage[k].PositionVector[i]);
                spice->add_velocity(this->spiceMessage[k].VelocityVector[i]);
                for (int j=0; j<3; j++){
                    spice->add_rotation(this->spiceMessage[k].J20002Pfix[i][j]);
                }
            }
//                spiceInMsgID[k].dataFresh = false;
        }
    }

    {

        // Serialize message (as is)
        uint32_t byteCount = (uint32_t) message->ByteSizeLong();
        void* serialized_message = malloc(byteCount);
        message->SerializeToArray(serialized_message, (int) byteCount);

        // BROADCAST MODE
        if (this->broadcastStream) {
            // Send serialized message to BROADCAST (PUBLISHER) socket */
            int sendStatus = zmq_send(this->publisher_socket, "SIM_UPDATE", 10, ZMQ_SNDMORE);
            if (sendStatus == -1) {
                bskLogger.bskLog(BSK_ERROR, "Broadcast header did not send to socket.");
            }
            sendStatus = zmq_send(this->publisher_socket, serialized_message, byteCount, 0);
            if (sendStatus == -1) {
                bskLogger.bskLog(BSK_ERROR, "Broadcast protobuffer did not send to socket.");
            }
        }

        /* If settings were broadcast, remove from message before saving & sending to 2-way socket to reduce message
           size. Message must be re-serialized here as its contents have changed. */
        if ((this->liveStream && (this->firstPass != 0) && (this->lastSettingsSendTime == this->now))
             || this->saveFile) {
            // Zero-out settings to reduce message size if not at first timestep
            if (this->settings.dataFresh == false) {
                message->set_allocated_settings(nullptr);
            }
            this->settings.dataFresh = false;

            // Re-serialize
            google::protobuf::uint8 varIntBuffer[4];
            byteCount = (uint32_t) message->ByteSizeLong();
            google::protobuf::uint8 *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount, varIntBuffer);
            unsigned long varIntBytes = (unsigned long) (end - varIntBuffer);
            // Save message to file if saveFile flag is true, and if Vizard is not being terminated
            if (this->saveFile && !this->liveSettings.terminateVizard) {
                this->outputStream->write(reinterpret_cast<char* > (varIntBuffer), (int) varIntBytes);
            }
            serialized_message = malloc(byteCount);
            message->SerializeToArray(serialized_message, (int) byteCount);
        }

        // Check whether noDisplay mode should generate imagery from Vizard at this timestep
        bool returnCamImgStatus = false;
        if (this->noDisplay) {
            for (size_t camCounter = 0; camCounter < this->cameraConfInMsgs.size(); camCounter++) {
                if ((CurrentSimNanos%this->cameraConfigBuffers[camCounter].renderRate == 0 && this->cameraConfigBuffers[camCounter].isOn == 1) ||this->firstPass < 11) {
                    returnCamImgStatus = true;
                }
            }
        }

        if (this->liveStream || (this->noDisplay && returnCamImgStatus)) {
            // Receive pong
            // Viz needs 10 images before placing the planets, wait for 11 protobuffers to have been created before attempting to go into noDisplay mode
            if (this->firstPass < 11){
                this->firstPass++;
            }
            zmq_msg_t receive_buffer;
            zmq_msg_init(&receive_buffer);
            zmq_msg_recv (&receive_buffer, requester_socket, 0);
            zmq_msg_close(&receive_buffer);

            /*! - Normal sim step by sending protobuffers */
            zmq_msg_t request_header;
            zmq_msg_t empty_frame1;
            zmq_msg_t empty_frame2;
            zmq_msg_t request_buffer;

            void* header_message = malloc(10 * sizeof(char));
            memcpy(header_message, "SIM_UPDATE", 10);

            zmq_msg_init_data(&request_header, header_message, 10, message_buffer_deallocate, NULL);
            zmq_msg_init(&empty_frame1);
            zmq_msg_init(&empty_frame2);
            zmq_msg_init_data(&request_buffer, serialized_message, byteCount, message_buffer_deallocate, NULL);

            // Send to 2-WAY (REQUESTER) socket
            zmq_msg_send(&request_header, this->requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&empty_frame1, this->requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&empty_frame2, this->requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&request_buffer, this->requester_socket, 0);

            zmq_msg_close(&request_header);
            zmq_msg_close(&empty_frame1);
            zmq_msg_close(&empty_frame2);
            zmq_msg_close(&request_buffer);

            // Receive status message from Vizard after SIM_UPDATE
            if (this->liveSettings.terminateVizard) {
                this->broadcastStream = false;
                this->liveStream = false;
                this->noDisplay = false;
            }
            else {
                zmq_msg_t receiveOK;
                zmq_msg_init(&receiveOK);
                int receive_status = zmq_msg_recv(&receiveOK, this->requester_socket, 0);
                if (receive_status) {
                    // Make sure "OK" was received from Vizard
                    void* msgData = zmq_msg_data(&receiveOK);
                    size_t msgSize = zmq_msg_size(&receiveOK);
                    std::string receiveOKStr (static_cast<char*>(msgData), msgSize);
                    std::string errStatusStr = "OK";
                    if (receiveOKStr.compare(errStatusStr) != 0) {
                        bskLogger.bskLog(BSK_ERROR, "Vizard 2-way [0]: Error processing SIM_UPDATE.");
                        return;
                    }
                }
                else {
                    bskLogger.bskLog(BSK_ERROR, "Vizard: Did not return a status (OK) message during SIM_UPDATE.");
                }
                zmq_msg_close(&receiveOK);

                // Only handle user input if in liveStream mode (and not in noDisplay mode)
                if (this->liveStream) {
                    this->receiveUserInput(CurrentSimNanos);
                }

                for (size_t camCounter =0; camCounter<this->cameraConfInMsgs.size(); camCounter++) {
                    /*! - If the camera is requesting periodic images, request them */
                    if (CurrentSimNanos%this->cameraConfigBuffers[camCounter].renderRate == 0 &&
                        this->cameraConfigBuffers[camCounter].isOn == 1)
                    {
                        this->requestImage(camCounter, CurrentSimNanos);
                    }
                }
                if (returnCamImgStatus) {
                    /*! -- Ping the Viz back to continue the lock-step */
                    void* keep_alive = malloc(4 * sizeof(char));
                    memcpy(keep_alive, "PING", 4);
                    zmq_msg_t request_life;
                    zmq_msg_init_data(&request_life, keep_alive, 4, message_buffer_deallocate, NULL);
                    zmq_msg_send(&request_life, this->requester_socket, 0);
                    zmq_msg_close(&request_life);
                    return;
                }
            }

        }
        /*!  Write protobuffer to file */
        if (!this->saveFile  || !message->SerializeToOstream(this->outputStream)) {
            return;
        }
        this->outputStream->flush();
    }

    delete message;
    google::protobuf::ShutdownProtobufLibrary();

}

/*! Update this module at the task rate
 @param CurrentSimNanos The current sim time
 */
void VizInterface::UpdateState(uint64_t CurrentSimNanos)
{

    this->FrameNumber+=1;
    ReadBSKMessages();
    if(CurrentSimNanos > 0) {
        WriteProtobuffer(CurrentSimNanos);
    }

}

/*! Method to add a Vizard instrument camera module to vizInterface
 @param tmpMsg Camera configuration msg
 */
void VizInterface::addCamMsgToModule(Message<CameraConfigMsgPayload> *tmpMsg)
{
    /* add the message reader to the vector of input messages */
    this->cameraConfInMsgs.push_back(tmpMsg->addSubscriber());

    /* expand vector of camera status and data buffer information */
    MsgCurrStatus tmpStatusMsg = {};
    this->cameraConfMsgStatus.push_back(tmpStatusMsg);

    CameraConfigMsgPayload tmpCamConfigMsg = {};
    tmpCamConfigMsg.cameraID = -1;
    strcpy(tmpCamConfigMsg.skyBox, "");
    tmpCamConfigMsg.renderRate = 0;
    this->cameraConfigBuffers.push_back(tmpCamConfigMsg);

    /* create output message */
    Message<CameraImageMsgPayload> *msg;
    msg = new Message<CameraImageMsgPayload>;
    this->opnavImageOutMsgs.push_back(msg);

    /* create image pointer */
    void *imgPtr = NULL;
    this->bskImagePtrs.push_back(imgPtr);
}


/*! During liveStream mode, ask Vizard for any user inputs recorded since the last timestep.
    Parse and package into VizUserInputMsgPayload for handling in Python.
 @param CurrentSimNanos The current sim time
 */
void VizInterface::receiveUserInput(uint64_t CurrentSimNanos){

    // Send "REQUEST_INPUT" to elicit a response from Vizard
    void* request_input_str = malloc(13 * sizeof(char));
    memcpy(request_input_str, "REQUEST_INPUT", 13);
    zmq_msg_t request_input_msg;
    zmq_msg_init_data(&request_input_msg, request_input_str, 13, message_buffer_deallocate, NULL);
    zmq_msg_send(&request_input_msg, this->requester_socket, 0);
    zmq_msg_close(&request_input_msg);

    /* Expect Vizard to send a status string on the socket, then the protobuffer
            Status string can be: "VIZARD_INPUT" or "ERROR" */
    zmq_msg_t viz_response;
    zmq_msg_init(&viz_response);
    int receive_status = zmq_msg_recv(&viz_response, this->requester_socket, 0);

    // If socket was empty, throw an error and exit
    if (receive_status == -1) {
        bskLogger.bskLog(BSK_ERROR, "Vizard 2-way [1]: Communication error. No data on socket.");
        return;
    }
    // Else, parse the status string and ensure Vizard did not send "ERROR"
    else {
        void* msgData = zmq_msg_data(&viz_response);
        size_t msgSize = zmq_msg_size(&viz_response);
        std::string receive_status_str (static_cast<char*>(msgData), msgSize);
        std::string err_status_str = "ERROR";
        if (receive_status_str.compare(err_status_str) == 0) {
            bskLogger.bskLog(BSK_ERROR, "Vizard 2-way [2]: Invalid request string.");
            return;
        }
    }

    // Retrieve protobuffer from socket
    receive_status = zmq_msg_recv(&viz_response, this->requester_socket, 0);

    if (receive_status != -1) {
        // Extract message size and data pointer from socket
        int vizPointSize = (int)zmq_msg_size(&viz_response);
        void* vizPoint = zmq_msg_data(&viz_response);

        // Set up and fill VizInput message
        vizProtobufferMessage::VizInput* msgRecv = new vizProtobufferMessage::VizInput;
        msgRecv->ParseFromArray(vizPoint, vizPointSize);

        // Set up output VizUserInputMsgPayload message
        VizUserInputMsgPayload outMsgBuffer;
        outMsgBuffer = this->userInputMsg.zeroMsgPayload;
        outMsgBuffer.frameNumber = static_cast<int>(msgRecv->framenumber());

        // Parse keyboard inputs
        const std::string& keys = msgRecv->keyinputs().keys();
        if (keys.length() > 0) {
            outMsgBuffer.keyboardInput = keys;
        }

        // Receive VizBroadcastSyncSettings
        const vizProtobufferMessage::VizBroadcastSyncSettings* vbss = &(msgRecv->broadcastsyncsettings());
        // Remove "const"ness for compatibility with protobuffer access methods
        vizProtobufferMessage::VizBroadcastSyncSettings* vbss_nc;
        vbss_nc = const_cast<vizProtobufferMessage::VizBroadcastSyncSettings*>(vbss);

        // Iterate through VizEventReply objects
        for (int i=0; i<msgRecv->replies_size(); i++) {
            const vizProtobufferMessage::VizEventReply* ver = &(msgRecv->replies(i));

            // Remove "const"ness for compatibility with protobuffer access methods
            vizProtobufferMessage::VizEventReply* ver_nc;
            ver_nc = const_cast<vizProtobufferMessage::VizEventReply*>(ver);

            // Create EventReply containers and pack into userInputMsg
            VizEventReply* er = new VizEventReply();
            er->eventHandlerID = *(ver_nc->mutable_eventhandlerid());
            er->reply = *(ver_nc->mutable_reply());
            er->eventHandlerDestroyed = ver_nc->eventhandlerdestroyed();
            outMsgBuffer.vizEventReplies.push_back(*er);

            // Populate VizEventReply in VizBroadcastSyncSettings
            vizProtobufferMessage::VizEventReply* ver_nc_bss = vbss_nc->add_dialogevents();
            ver_nc_bss->set_eventhandlerid(er->eventHandlerID);
            ver_nc_bss->set_reply(er->reply);
            ver_nc_bss->set_eventhandlerdestroyed(er->eventHandlerDestroyed);
        }

        // Serialize and send BroadcastSyncSettings*/
        uint32_t syncByteCount = (uint32_t) vbss_nc->ByteSizeLong();

        // Serialize if in broadcastStream mode
        if (syncByteCount > 0 && this->broadcastStream) {
            void* sync_settings = malloc(syncByteCount);
            vbss_nc->SerializeToArray(sync_settings, (int) syncByteCount);

            // Send sync settings message to BROADCAST (PUBLISHER) socket */
            int sendStatus = zmq_send(this->publisher_socket, "SYNC_SETTINGS", 13, ZMQ_SNDMORE);
            if (sendStatus == -1) {
                bskLogger.bskLog(BSK_ERROR, "Broadcast header did not send to socket.");
            }
            sendStatus = zmq_send(this->publisher_socket, sync_settings, syncByteCount, 0);
            if (sendStatus == -1) {
                bskLogger.bskLog(BSK_ERROR, "Broadcast protobuffer did not send to socket.");
            }
        }

        this->userInputMsg.write(&outMsgBuffer, this->moduleID, CurrentSimNanos);

        delete msgRecv;
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Vizard 2-way [2]: Did not return a user input message.");
    }

    zmq_msg_close(&viz_response);

}


/*! Requests an image from Vizard and stores it in the image output message
 */
void VizInterface::requestImage(size_t camCounter, uint64_t CurrentSimNanos)
{
    char buffer[10];
    zmq_recv(this->requester_socket, buffer, 10, 0);
    /*! -- Send request */
    std::string cmdMsg = "REQUEST_IMAGE_";
    cmdMsg += std::to_string(this->cameraConfigBuffers[camCounter].cameraID);
    void* img_message = malloc(cmdMsg.length() * sizeof(char));
    memcpy(img_message, cmdMsg.c_str(), cmdMsg.length());
    zmq_msg_t img_request;
    zmq_msg_init_data(&img_request, img_message, cmdMsg.length(), message_buffer_deallocate, NULL);
    zmq_msg_send(&img_request, this->requester_socket, 0);
    zmq_msg_close(&img_request);

    zmq_msg_t length;
    zmq_msg_t image;
    zmq_msg_init(&length);
    zmq_msg_init(&image);
    if (this->bskImagePtrs[camCounter] != NULL) {
        /*! If the permanent image buffer is not populated, it will be equal to null*/
        free(this->bskImagePtrs[camCounter]);
        this->bskImagePtrs[camCounter] = NULL;
    }
    zmq_msg_recv(&length, this->requester_socket, 0);
    zmq_msg_recv(&image, this->requester_socket, 0);

    int32_t *lengthPoint= (int32_t *)zmq_msg_data(&length);
    void *imagePoint= zmq_msg_data(&image);
    int32_t length_unswapped = *lengthPoint;
    /*! --  Endianness switch for the length of the buffer */
    int32_t imageBufferLength =((length_unswapped>>24)&0xff) | // move byte 3 to byte 0
                                ((length_unswapped<<8)&0xff0000) | // move byte 1 to byte 2
                                ((length_unswapped>>8)&0xff00) | // move byte 2 to byte 1
                                ((length_unswapped<<24)&0xff000000); // byte 0 to byte 3

    /*!-Copy the image buffer pointer, so that it does not get freed by ZMQ*/
    this->bskImagePtrs[camCounter] = malloc(imageBufferLength*sizeof(char));
    memcpy(this->bskImagePtrs[camCounter], imagePoint, imageBufferLength*sizeof(char));

    /*! -- Write out the image information to the Image message */
    CameraImageMsgPayload imageData = {};
    imageData.timeTag = CurrentSimNanos;
    imageData.valid = 0;
    imageData.imagePointer = this->bskImagePtrs[camCounter];
    imageData.imageBufferLength = imageBufferLength;
    imageData.cameraID = this->cameraConfigBuffers[camCounter].cameraID;
    imageData.imageType = 4;
    if (imageBufferLength>0){imageData.valid = 1;}
    this->opnavImageOutMsgs[camCounter]->write(&imageData, this->moduleID, CurrentSimNanos);

    /*! -- Clean the messages to avoid memory leaks */
    zmq_msg_close(&length);
    zmq_msg_close(&image);
}



/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 @param hint
 */
void message_buffer_deallocate(void *data, void *hint)
{
    free (data);
}
