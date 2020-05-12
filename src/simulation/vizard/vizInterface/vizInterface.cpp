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
#include <architecture/messaging/system_messaging.h>
#include <zmq.h>

#include "vizInterface.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "architecture/messaging/system_messaging.h"
#include "sensors/sun_sensor/coarse_sun_sensor.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include "utilities/astroConstants.h"

void message_buffer_deallocate(void *data, void *hint);

/*! VizInterface Constructor
 */
VizInterface::VizInterface()
{
    this->opNavMode = 0;
    this->saveFile = false;
    this->liveStream = false;
    this->FrameNumber= -1;
    this->numOutputBuffers = 2;

    memset(&this->cameraConfigMessage, 0x0, sizeof(CameraConfigMsg));
    this->cameraConfigMessage.cameraID = -1;
    strcpy(this->cameraConfigMessage.skyBox, "");
    this->cameraConfigMessage.renderRate = 0;

    this->planetNames = {};

    this->firstPass = 0;
    
    this->comProtocol = "tcp";
    this->comAddress = "localhost";
    this->comPortNumber = "5556";
    
    return;
}

/*! VizInterface Destructor
 */
VizInterface::~VizInterface()
{
    return;
}

/*! Initialization method for subscription to messages. This module does not output messages, but files containing the protobuffers for Vizard
 */
void VizInterface::SelfInit()
{
    if (this->opNavMode > 0 || this->liveStream){
        /* setup zeroMQ */
        this->bskImagePtr = NULL;
        this->context = zmq_ctx_new();
        this->requester_socket = zmq_socket(this->context, ZMQ_REQ);
        zmq_connect(this->requester_socket, (this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber).c_str());

        void* message = malloc(4 * sizeof(char));
        memcpy(message, "PING", 4);
        zmq_msg_t request;
        
        std::cout << "Waiting for Vizard at " + this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber << std::endl;
        
        zmq_msg_init_data(&request, message, 4, message_buffer_deallocate, NULL);
        zmq_msg_send(&request, this->requester_socket, 0);
        char buffer[4];
        zmq_recv (this->requester_socket, buffer, 4, 0);
        zmq_send (this->requester_socket, "PING", 4, 0);
        std::cout << "Basilisk-Vizard connection made" << std::endl;
        
        /* Create output message for module in opNav mode */
        if (this->opNavMode > 0) {
            uint64_t imageBufferCount = 2;
            this->imageOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->opnavImageOutMsgName,sizeof(CameraImageMsg),imageBufferCount,"CameraImageMsg", moduleID);
        }
    }

    return;
}

/*! Cross initialization. Module subscribes to other messages. In viz interface, many messages are subscribed to in order to extract information from the viz and give it to the visualization tool.
 */
void VizInterface::CrossInit()
{
    MessageIdentData msgInfo;
    std::vector<VizSpacecraftData>::iterator scIt;
    for (scIt = this->scData.begin(); scIt != this->scData.end(); scIt++)
    {
        /* Define CSS data input messages */
        msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(scIt->cssDataInMsgName);
        if (msgInfo.itemFound) {
            scIt->cssDataInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scIt->cssDataInMsgName,
                                                                                      sizeof(CSSArraySensorIntMsg), moduleID);
            scIt->cssDataInMsgId.dataFresh = false;
            scIt->cssDataInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            scIt->cssDataInMsgId.msgID = -1;
        }

        /* Define CSS configuration input messages */
        msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(scIt->cssConfInMsgName);
        if (msgInfo.itemFound) {
            scIt->cssConfInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scIt->cssConfInMsgName,
                                                                                            sizeof(CSSConfigFswMsg), moduleID);
            scIt->cssConfInMsgId.dataFresh = false;
            scIt->cssConfInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            scIt->cssConfInMsgId.msgID = -1;
        }

        /* Define SCPlus input message */
        msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(scIt->scPlusInMsgName);
        if (msgInfo.itemFound) {
            scIt->scPlusInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scIt->scPlusInMsgName,
                    sizeof(SCPlusStatesSimMsg), moduleID);
            scIt->scPlusInMsgID.dataFresh = false;
            scIt->scPlusInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            scIt->scPlusInMsgID.msgID = -1;
        }

        /* Define StarTracker input message */
        msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(scIt->starTrackerInMsgName);
        if (msgInfo.itemFound) {
            scIt->starTrackerInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scIt->starTrackerInMsgName,
                    sizeof(STSensorIntMsg), moduleID);
            scIt->starTrackerInMsgID.dataFresh = false;
            scIt->starTrackerInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            scIt->starTrackerInMsgID.msgID = -1;
        }

        /* Define RW input message */
        {
            MsgCurrStatus rwStatus;
            rwStatus.dataFresh = false;
            rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            bool rwMsgNameSet = scIt->rwInMsgName.size();
            for (size_t idx = 0; idx < scIt->numRW; idx++)
            {
                std::string tmpWheelMsgName;
                if (rwMsgNameSet == false) {
                    tmpWheelMsgName= scIt->spacecraftName + "_rw_config_" + std::to_string(idx) + "_data";
                    scIt->rwInMsgName.push_back(tmpWheelMsgName);
                } else {
                    tmpWheelMsgName = scIt->rwInMsgName[idx];
                }
                msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(tmpWheelMsgName);
                if (msgInfo.itemFound) {
                    rwStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scIt->rwInMsgName[idx],sizeof(RWConfigLogSimMsg), moduleID);
                } else {
                    rwStatus.msgID = -1;
                    bskLogger.bskLog(BSK_WARNING, "vizInterface: RW(%zu) msg %s requested but not found.", idx, scIt->rwInMsgName[idx].c_str());
                }
                scIt->rwInMsgID.push_back(rwStatus);
            }
            scIt->rwInMessage.resize(scIt->rwInMsgID.size());
        }

        /* Define Thr input message */
        {
            MsgCurrStatus thrStatus;
            thrStatus.dataFresh = false;
            thrStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
            std::vector<ThrClusterMap>::iterator thrIt;
            scIt->numThr = 0;
            for (thrIt= scIt->thrMsgData.begin(); thrIt != scIt->thrMsgData.end(); thrIt++)
            {
                for(int idx=0; idx < thrIt->thrCount; idx++) {
                    std::string tmpThrustMsgName = "thruster_" + thrIt->thrTag + "_" + std::to_string(idx) + "_data";
                    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(tmpThrustMsgName);
                    if (msgInfo.itemFound) {
                        thrStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(tmpThrustMsgName, sizeof(THROutputSimMsg), moduleID);
                        scIt->thrMsgID.push_back(thrStatus);
                        scIt->numThr++;
                    } else {
                        thrStatus.msgID = -1;
                        bskLogger.bskLog(BSK_WARNING, "vizInterface: TH(%d) msg %s of tag %s requested but not found.", idx, tmpThrustMsgName.c_str(), thrIt->thrTag.c_str());
                    }
                }
            }
            scIt->thrOutputMessage.resize(scIt->thrMsgID.size());
        }
    }

    /* Define Camera input messages */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->cameraConfInMsgName);
    if (msgInfo.itemFound) {
        this->cameraConfMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cameraConfInMsgName,
                                                                                    sizeof(CameraConfigMsg), moduleID);
        this->cameraConfMsgId.dataFresh = false;
        this->cameraConfMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->cameraConfMsgId.msgID = -1;
    }

    /* Define Spice input message */
    {
        size_t i=0;
        MsgCurrStatus spiceStatus;
        spiceStatus.dataFresh = false;
        spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        std::vector<std::string>::iterator spice_it;
        for(spice_it = this->planetNames.begin(); spice_it != this->planetNames.end(); spice_it++){
            std::string planetMsgName = *spice_it + "_planet_data";
            msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(planetMsgName);
            if (msgInfo.itemFound) {
                this->spiceInMsgName.push_back(planetMsgName);
                //! Subscribe to messages
                spiceStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->spiceInMsgName[i], sizeof(SpicePlanetStateSimMsg), moduleID);
            } else {
                spiceStatus.msgID = -1;
            }
            this->spiceInMsgID.push_back(spiceStatus);
            i++;
        }
        this->spiceMessage.resize(this->spiceInMsgID.size());
    }

    return;
}

/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::Reset(uint64_t CurrentSimNanos)
{
    this->FrameNumber=-1;
    if (this->saveFile) {
//        if(this->outputStream && this->outputStream->is_open())
//        {
//            this->outputStream->close();
//            delete this->outputStream;
//        }
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);
    }

    this->settings.dataFresh = true;        // reset flag to transmit Vizard settings
    return;
}

/*! A method in which the module reads the content of all available bsk messages
 */
void VizInterface::ReadBSKMessages()
{
    std::vector<VizSpacecraftData>::iterator scIt;

    for (scIt = this->scData.begin(); scIt != this->scData.end(); scIt++)
    {
        /* Read BSK SCPlus msg */
        if (scIt->scPlusInMsgID.msgID >= 0){
            SCPlusStatesSimMsg localSCPlusArray;
            SingleMessageHeader localSCPlusHeader;
            SystemMessaging::GetInstance()->ReadMessage(scIt->scPlusInMsgID.msgID, &localSCPlusHeader,
                                                        sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&localSCPlusArray));
            if(localSCPlusHeader.WriteSize > 0 && localSCPlusHeader.WriteClockNanos != scIt->scPlusInMsgID.lastTimeTag){
                scIt->scPlusInMsgID.lastTimeTag = localSCPlusHeader.WriteClockNanos;
                scIt->scPlusInMsgID.dataFresh = true;
            }
            scIt->scPlusMessage = localSCPlusArray;
        }

        /* Read BSK RW constellation msg */
        {
        for (size_t idx=0;idx< scIt->numRW; idx++)
        {
            if (scIt->rwInMsgID[idx].msgID != -1){
            RWConfigLogSimMsg localRWArray;
            SingleMessageHeader localRWHeader;
            SystemMessaging::GetInstance()->ReadMessage(scIt->rwInMsgID[idx].msgID, &localRWHeader, sizeof(RWConfigLogSimMsg), reinterpret_cast<uint8_t*>(&localRWArray));
                if(localRWHeader.WriteSize > 0 && localRWHeader.WriteClockNanos != scIt->rwInMsgID[idx].lastTimeTag){
                    scIt->rwInMsgID[idx].lastTimeTag = localRWHeader.WriteClockNanos;
                    scIt->rwInMsgID[idx].dataFresh = true;
                    scIt->rwInMessage[idx] = localRWArray;
                }
            }
        }
        }

         /* Read incoming Thruster constellation msg */
        {
        for (size_t idx=0;idx< scIt->numThr; idx++){
            if (scIt->thrMsgID[idx].msgID != -1){
                THROutputSimMsg localThrusterArray;
                SingleMessageHeader localThrusterHeader;
                SystemMessaging::GetInstance()->ReadMessage(scIt->thrMsgID[idx].msgID, &localThrusterHeader, sizeof(THROutputSimMsg), reinterpret_cast<uint8_t*>(&localThrusterArray));
                if(localThrusterHeader.WriteSize > 0 && localThrusterHeader.WriteClockNanos != scIt->thrMsgID[idx].lastTimeTag){
                    scIt->thrMsgID[idx].lastTimeTag = localThrusterHeader.WriteClockNanos;
                    scIt->thrMsgID[idx].dataFresh = true;
                    scIt->thrOutputMessage[idx] = localThrusterArray;
                }
            }
        }
        }

        /*! Read CSS data msg */
        if (scIt->cssDataInMsgId.msgID != -1){
            CSSArraySensorIntMsg localCSSDataArray;
            SingleMessageHeader localCSSDataHeader;
            SystemMessaging::GetInstance()->ReadMessage(scIt->cssDataInMsgId.msgID, &localCSSDataHeader, sizeof(CSSArraySensorIntMsg), reinterpret_cast<uint8_t*>(&localCSSDataArray));
            if(localCSSDataHeader.WriteSize > 0 && localCSSDataHeader.WriteClockNanos != scIt->cssDataInMsgId.lastTimeTag){
                scIt->cssDataInMsgId.lastTimeTag = localCSSDataHeader.WriteClockNanos;
                scIt->cssDataInMsgId.dataFresh = true;
            }
        }

        /*! Read incoming CSS config msg */
        if (scIt->cssConfInMsgId.msgID != -1){
            CSSConfigFswMsg localCSSConfigArray;
            SingleMessageHeader localCSSConfigHeader;
            SystemMessaging::GetInstance()->ReadMessage(scIt->cssConfInMsgId.msgID, &localCSSConfigHeader, sizeof(CSSConfigFswMsg), reinterpret_cast<uint8_t*>(&localCSSConfigArray));
            if(localCSSConfigHeader.WriteSize > 0 && localCSSConfigHeader.WriteClockNanos != scIt->cssConfInMsgId.lastTimeTag){
                scIt->cssConfInMsgId.lastTimeTag = localCSSConfigHeader.WriteClockNanos;
                scIt->cssConfInMsgId.dataFresh = true;
            }
            scIt->cssConfigMessage = localCSSConfigArray;
        }

        /*! Read incoming ST constellation msg */
        if (scIt->starTrackerInMsgID.msgID != -1){
            STSensorIntMsg localSTArray;
            SingleMessageHeader localSTHeader;
            SystemMessaging::GetInstance()->ReadMessage(scIt->starTrackerInMsgID.msgID, &localSTHeader, sizeof(STSensorIntMsg), reinterpret_cast<uint8_t*>(&localSTArray));
            if(localSTHeader.WriteSize > 0 && localSTHeader.WriteClockNanos != scIt->starTrackerInMsgID.lastTimeTag){
                scIt->starTrackerInMsgID.lastTimeTag = localSTHeader.WriteClockNanos;
                scIt->starTrackerInMsgID.dataFresh = true;
            }
            scIt->STMessage = localSTArray;
        }
    }

    /*! Read incoming camera config msg */
    if (this->cameraConfMsgId.msgID != -1){
        CameraConfigMsg localCameraConfigArray;
        SingleMessageHeader localCameraConfigHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->cameraConfMsgId.msgID, &localCameraConfigHeader, sizeof(CameraConfigMsg), reinterpret_cast<uint8_t*>(&localCameraConfigArray));
        if(localCameraConfigHeader.WriteSize > 0 && localCameraConfigHeader.WriteClockNanos != this->cameraConfMsgId.lastTimeTag){
            this->cameraConfMsgId.lastTimeTag = localCameraConfigHeader.WriteClockNanos;
            this->cameraConfMsgId.dataFresh = true;
        }
        this->cameraConfigMessage = localCameraConfigArray;
    }


    /*! Read BSK Spice constellation msg */
    {
        size_t i=0;
        std::vector<std::string>::iterator it;
        for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
        {
            if (this->spiceInMsgID[i].msgID != -1){
                SpicePlanetStateSimMsg localSpiceArray;
                SingleMessageHeader localSpiceHeader;
                memset(&localSpiceHeader, 0x0, sizeof(SingleMessageHeader));
                SystemMessaging::GetInstance()->ReadMessage(this->spiceInMsgID[i].msgID, &localSpiceHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&localSpiceArray));
                if(localSpiceHeader.WriteSize > 0 && localSpiceHeader.WriteClockNanos != this->spiceInMsgID[i].lastTimeTag){
                    this->spiceInMsgID[i].lastTimeTag = localSpiceHeader.WriteClockNanos;
                    this->spiceInMsgID[i].dataFresh = true;
                    this->spiceMessage[i] = localSpiceArray;
                }
            }
            i+=1;
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

    /*! Send the Vizard settings once */
    if (this->settings.dataFresh) {
        vizProtobufferMessage::VizMessage::VizSettingsPb* vizSettings;
        vizSettings = new vizProtobufferMessage::VizMessage::VizSettingsPb;

        // define the viz ambient light setting
        vizSettings->set_ambient(this->settings.ambient);
        if (this->settings.ambient > 8.0 ||
            (this->settings.ambient < 0.0 && this->settings.ambient != -1.0)) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard ambient light value must be within [0,8].  A value of %f was received.", this->settings.ambient);
        }

        // define if orbit lines should be shown
        vizSettings->set_orbitlineson(this->settings.orbitLinesOn);
        if (abs(this->settings.orbitLinesOn)>1) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard orbitLinesOn flag must be either -1, 0 or 1.  A value of %d was received.", this->settings.orbitLinesOn);
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
        vizSettings->set_customguiscale(this->settings.customGUIScale);
        if (abs(this->settings.customGUIScale)>3.0) {
            bskLogger.bskLog(BSK_WARNING, "vizInterface: The Vizard customGUIScale flag must be either -1 or [0.5, 3]  A value of %d was received.", this->settings.customGUIScale);
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
        }

        // define camera settings
        for (size_t idx = 0; idx < this->settings.stdCameraList.size(); idx++){
            vizProtobufferMessage::VizMessage::StandardCameraSettings* sc = vizSettings->add_standardcamerasettings();
            StdCameraSettings *scp = &(this->settings.stdCameraList[idx]);
            sc->set_spacecraftname(scp->spacecraftName);
            sc->set_setmode(scp->setMode);
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
        }

        message->set_allocated_settings(vizSettings);

        this->settings.dataFresh = false;
    }

    /*! Write timestamp output msg */
    vizProtobufferMessage::VizMessage::TimeStamp* time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->FrameNumber);
    time->set_simtimeelapsed(CurrentSimNanos);
    message->set_allocated_currenttime(time);


    std::vector<VizSpacecraftData>::iterator scIt;

    for (scIt = scData.begin(); scIt != scData.end(); scIt++)
    {
        /*! Write SCPlus output msg */
        if (scIt->scPlusInMsgID.msgID != -1 && scIt->scPlusInMsgID.dataFresh){
            printf("HPS: writing protobuffer\n");
            vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
            scp->set_spacecraftname(scIt->spacecraftName);
            for (int i=0; i<3; i++){
                scp->add_position(scIt->scPlusMessage.r_BN_N[i]);
                scp->add_velocity(scIt->scPlusMessage.v_BN_N[i]);
                scp->add_rotation(scIt->scPlusMessage.sigma_BN[i]);
            }
            scIt->scPlusInMsgID.dataFresh = false;

            /* Write the SC sprite string */
            scp->set_spacecraftsprite(scIt->spacecraftSprite);

            /*! Write RW output msg */
            for (size_t idx =0; idx < scIt->numRW; idx++)
            {
                if (scIt->rwInMsgID[idx].msgID != -1 && scIt->rwInMsgID[idx].dataFresh){
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
            for (size_t idx =0; idx < scIt->numThr; idx++)
            {
                if (scIt->thrMsgID[idx].msgID != -1 && scIt->thrMsgID[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::Thruster* thr = scp->add_thrusters();
                    thr->set_maxthrust(scIt->thrOutputMessage[idx].maxThrust);
                    thr->set_currentthrust(scIt->thrOutputMessage[idx].thrustForce);
                    for (int i=0; i<3; i++){
                        thr->add_position(scIt->thrOutputMessage[idx].thrusterLocation[i]);
                        thr->add_thrustvector(scIt->thrOutputMessage[idx].thrusterDirection[i]);
                    }
                    //thrMsgID[idx].dataFresh = false;
                }
            }

            // Write CSS output msg
            //if (cssConfInMsgId != -1 and cssConfInMsgId != -1){
            //for (int i=0; i< this->cssConfigMessage.nCSS; i++){
            //    vizProtobufferMessage::VizMessage::CoarseSunSensor* css = scp->add_css();
            //    for (int j=0; j<3; j++){
            //        css->add_normalvector(this->cssConfigMessage.cssVals[i].nHat_B[j]);
            //        css->add_position(0);
            //    }
            //    css->set_currentmsmt(this->cssDataMessage.CosValue[i]);
            //    css->set_cssgroupid(i);
            //    }
            //}


            // Write ST output msg
            //if (starTrackerInMsgID != -1){
            //vizProtobufferMessage::VizMessage::StarTracker* st = scp->add_startrackers();
            //st->set_fieldofviewwidth(90);
            //st->set_fieldofviewheight(90);
            //for (int i=0; i<4; i++){
            //    st->add_position(0);
            //    st->add_rotation(this->STMessage.qInrtl2Case[i]);
            //}
            //}

        }
    }

    /*! Write camera output msg */
    if ((this->cameraConfMsgId.msgID != -1 && this->cameraConfMsgId.dataFresh)
        || this->cameraConfigMessage.cameraID >= 0){
        /*! This corrective rotation allows unity to place the camera as is expected by the python setting. Unity has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame: z point, x horizontal, y vertical (down) */
        double sigma_CuC[3], unityCameraMRP[3]; /*! Cu is the unity Camera frame */
        v3Set(1./3, 1./3, -1./3, sigma_CuC);
        addMRP(this->cameraConfigMessage.sigma_CB, sigma_CuC, unityCameraMRP);
        vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
        for (int j=0; j<3; j++){
            if (j < 2){
            camera->add_resolution(this->cameraConfigMessage.resolution[j]);
            }
            camera->add_cameradir_b(unityCameraMRP[j]);
            camera->add_camerapos_b(this->cameraConfigMessage.cameraPos_B[j]);            }
        camera->set_renderrate(this->cameraConfigMessage.renderRate);        // Unity expects nano-seconds between images 
        camera->set_cameraid(this->cameraConfigMessage.cameraID);
        camera->set_fieldofview(this->cameraConfigMessage.fieldOfView*R2D);  // Unity expects degrees
        camera->set_skybox(this->cameraConfigMessage.skyBox);
        camera->set_parentname(this->cameraConfigMessage.parentName);
    }

    /*! Write spice output msgs */
    size_t k=0;
    std::vector<std::string>::iterator plIt;
    for(plIt = this->planetNames.begin(); plIt != this->planetNames.end(); plIt++)
    {
        if (spiceInMsgID[k].msgID != -1 && spiceInMsgID[k].dataFresh){
            vizProtobufferMessage::VizMessage::CelestialBody* spice = message->add_celestialbodies();
            spice->set_bodyname(*plIt);
            for (int i=0; i<3; i++){
                spice->add_position(this->spiceMessage[k].PositionVector[i]);
                spice->add_velocity(this->spiceMessage[k].VelocityVector[i]);
                for (int j=0; j<3; j++){
                    spice->add_rotation(this->spiceMessage[k].J20002Pfix[i][j]);
                }
            }
//                spiceInMsgID[k].dataFresh = false;
        }
        k++;
    }

    {
        google::protobuf::uint8 varIntBuffer[4];
        uint32_t byteCount = message->ByteSizeLong();
        google::protobuf::uint8 *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount, varIntBuffer);
        unsigned long varIntBytes = (unsigned long) (end - varIntBuffer);
        if (this->saveFile) {
            this->outputStream->write(reinterpret_cast<char* > (varIntBuffer), (int) varIntBytes);
        }

        /*! Enter in lock-step with the vizard to simulate a camera */
        /*!--OpNavMode set to 1 is to stay in lock-step with the viz at all time steps. It is a slower run, but provides visual capabilities during OpNav */
        /*!--OpNavMode set to 2 is a faster mode in which the viz only steps forward to the BSK time step if an image is requested. This is a faster run but nothing can be visualized post-run */
        if (this->opNavMode == 1
            ||(this->opNavMode == 2 && ((CurrentSimNanos%this->cameraConfigMessage.renderRate == 0 && this->cameraConfigMessage.isOn == 1) ||this->firstPass < 11))
            || this->liveStream
            ){
            // Receive pong
            /*! - The viz needs 10 images before placing the planets, wait for 11 protobuffers to have been created before attempting to go into opNavMode 2 */
            if (this->firstPass < 11){
                this->firstPass++;
            }
            zmq_msg_t receive_buffer;
            zmq_msg_init(&receive_buffer);
            zmq_msg_recv (&receive_buffer, requester_socket, 0);
            
            /*! - send protobuffer raw over zmq_socket */
            void* serialized_message = malloc(byteCount);
            message->SerializeToArray(serialized_message, (int) byteCount);

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
            zmq_msg_init_data(&request_buffer, serialized_message,byteCount, message_buffer_deallocate, NULL);

            zmq_msg_send(&request_header, requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&empty_frame1, requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&empty_frame2, requester_socket, ZMQ_SNDMORE);
            zmq_msg_send(&request_buffer, requester_socket, 0);
            
            /*! - If the camera is requesting periodic images, request them */
            if (this->opNavMode > 0 &&
                CurrentSimNanos%this->cameraConfigMessage.renderRate == 0 &&
                this->cameraConfigMessage.isOn == 1)
            {
                char buffer[10];
                zmq_recv(requester_socket, buffer, 10, 0);
                /*! -- Send request */
                void* img_message = malloc(13 * sizeof(char));
                memcpy(img_message, "REQUEST_IMAGE", 13);
                zmq_msg_t img_request;
                zmq_msg_init_data(&img_request, img_message, 13, message_buffer_deallocate, NULL);
                zmq_msg_send(&img_request, requester_socket, 0);
                
                zmq_msg_t length;
                zmq_msg_t image;
                zmq_msg_init(&length);
                zmq_msg_init(&image);
                if (this->bskImagePtr != NULL) {
                    /*! If the permanent image buffer is not populated, it will be equal to null*/
                    free(this->bskImagePtr);
                    this->bskImagePtr = NULL;
                }
                zmq_msg_recv(&length, requester_socket, 0);
                zmq_msg_recv(&image, requester_socket, 0);
                
                int32_t *lengthPoint= (int32_t *)zmq_msg_data(&length);
                void *imagePoint= zmq_msg_data(&image);
                int32_t length_unswapped = *lengthPoint;
                /*! --  Endianness switch for the length of the buffer */
                int32_t imageBufferLength =((length_unswapped>>24)&0xff) | // move byte 3 to byte 0
                ((length_unswapped<<8)&0xff0000) | // move byte 1 to byte 2
                ((length_unswapped>>8)&0xff00) | // move byte 2 to byte 1
                ((length_unswapped<<24)&0xff000000); // byte 0 to byte 3
                
                /*!-Copy the image buffer pointer, so that it does not get freed by ZMQ*/
                this->bskImagePtr = malloc(imageBufferLength*sizeof(char));
                memcpy(this->bskImagePtr, imagePoint, imageBufferLength*sizeof(char));
                
                /*! -- Write out the image information to the Image message */
                CameraImageMsg imageData;
                imageData.timeTag = CurrentSimNanos;
                imageData.valid = 0;
                imageData.imagePointer = this->bskImagePtr;
                imageData.imageBufferLength = imageBufferLength;
                imageData.cameraID = this->cameraConfigMessage.cameraID;
                imageData.imageType = 4;
                if (imageBufferLength>0){imageData.valid = 1;}
                SystemMessaging::GetInstance()->WriteMessage(this->imageOutMsgID, CurrentSimNanos, sizeof(CameraImageMsg), reinterpret_cast<uint8_t *>(&imageData), this->moduleID);

                /*! -- Clean the messages to avoid memory leaks */
                zmq_msg_close(&length);
                zmq_msg_close(&image);
                
                /*! -- Ping the Viz back to continue the lock-step */
                void* keep_alive = malloc(4 * sizeof(char));
                memcpy(keep_alive, "PING", 4);
                zmq_msg_t request_life;
                zmq_msg_init_data(&request_life, keep_alive, 4, message_buffer_deallocate, NULL);
                zmq_msg_send(&request_life, this->requester_socket, 0);
                return;
                
            }
            
        }
        /*!  Write protobuffer to file */
        if (!this->saveFile  || !message->SerializeToOstream(this->outputStream)) {
            return;
        }
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

/*! A cleaning method to ensure the message buffers are wiped clean.
 @param data The current sim time in nanoseconds
 */
void message_buffer_deallocate(void *data, void *hint)
{
    free (data);
}
