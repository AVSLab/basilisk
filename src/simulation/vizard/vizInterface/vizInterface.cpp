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

#include "vizInterface.h"
#include "architecture/utilities/macroDefinitions.h"
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
    this->opNavMode = 0;
    this->saveFile = false;
    this->liveStream = false;
    this->FrameNumber= -1;

    this->cameraConfigBuffer = this->cameraConfInMsg.zeroMsgPayload();
    this->cameraConfigBuffer.cameraID = -1;
    strcpy(this->cameraConfigBuffer.skyBox, "");
    this->cameraConfigBuffer.renderRate = 0;

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
    return;
}

/*! Cross initialization. Module subscribes to other messages. In viz interface, many messages are subscribed to in order to extract information from the viz and give it to the visualization tool.
 */
void VizInterface::CrossInit()
{
    return;
}

/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::Reset(uint64_t CurrentSimNanos)
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

        std::string text;
        text = "Waiting for Vizard at " + this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber;
        bskLogger.bskLog(BSK_INFORMATION, text.c_str());

        zmq_msg_init_data(&request, message, 4, message_buffer_deallocate, NULL);
        zmq_msg_send(&request, this->requester_socket, 0);
        char buffer[4];
        zmq_recv (this->requester_socket, buffer, 4, 0);
        zmq_send (this->requester_socket, "PING", 4, 0);
        bskLogger.bskLog(BSK_INFORMATION, "Basilisk-Vizard connection made");
    }

    std::vector<VizSpacecraftData>::iterator scIt;
    for (scIt = this->scData.begin(); scIt != this->scData.end(); scIt++)
    {
        /* Check SCPlus input message */
        if (scIt->scPlusInMsg.isLinked()) {
            scIt->scPlusInMsgStatus.dataFresh = false;
            scIt->scPlusInMsgStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
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

        /* Check StarTracker input message */
        if (scIt->starTrackerInMsg.isLinked()) {
            scIt->starTrackerInMsgStatus.dataFresh = false;
            scIt->starTrackerInMsgStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
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
    if (this->cameraConfInMsg.isLinked()) {
        this->cameraConfMsgStatus.dataFresh = false;
        this->cameraConfMsgStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    }

    /* Check Spice input message */
    {
        MsgCurrStatus spiceStatus;
        spiceStatus.dataFresh = true;  // this ensures that default planet states are also used
        spiceStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        this->spiceInMsgStatus.clear();
        this->spiceMessage.clear();
        for (int c = 0; c< (int) this->planetNames.size(); c++) {
            /* set default zero translation and rotation states */
            SpicePlanetStateMsgPayload logMsg = {};
            m33SetIdentity(logMsg.J20002Pfix);
            strcpy(logMsg.PlanetName, this->planetNames.at(c).c_str());

            this->spiceInMsgStatus.push_back(spiceStatus);
            this->spiceMessage.push_back(logMsg);
        }
    }

    this->FrameNumber=-1;
    if (this->saveFile) {
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);
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
        /* Read BSK SCPlus msg */
        if (scIt->scPlusInMsg.isLinked()){
            SCPlusStatesMsgPayload localSCPlusArray;
            localSCPlusArray = scIt->scPlusInMsg();
            if(scIt->scPlusInMsg.isWritten() && scIt->scPlusInMsg.timeWritten() != scIt->scPlusInMsgStatus.lastTimeTag){
                scIt->scPlusInMsgStatus.lastTimeTag = scIt->scPlusInMsg.timeWritten();
                scIt->scPlusInMsgStatus.dataFresh = true;
            }
            scIt->scPlusMessage = localSCPlusArray;
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

        /* Read incoming ST constellation msg */
        {
        if (scIt->starTrackerInMsg.isLinked()){
            STSensorMsgPayload localSTArray;
            localSTArray = scIt->starTrackerInMsg();
            if(scIt->starTrackerInMsg.isWritten() &&
               scIt->starTrackerInMsg.timeWritten() != scIt->starTrackerInMsgStatus.lastTimeTag){
                scIt->starTrackerInMsgStatus.lastTimeTag = scIt->starTrackerInMsg.timeWritten();
                scIt->starTrackerInMsgStatus.dataFresh = true;
            }
            scIt->STMessage = localSTArray;
        }
        }
    } /* end of scIt loop */

    /*! Read incoming camera config msg */
    if (this->cameraConfInMsg.isLinked()){
        CameraConfigMsgPayload localCameraConfigArray;
        localCameraConfigArray = this->cameraConfInMsg();
        if(this->cameraConfInMsg.isWritten() &&
           this->cameraConfInMsg.timeWritten() != this->cameraConfMsgStatus.lastTimeTag){
            this->cameraConfMsgStatus.lastTimeTag = this->cameraConfInMsg.timeWritten();
            this->cameraConfMsgStatus.dataFresh = true;
        }
        this->cameraConfigBuffer = localCameraConfigArray;
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
        vizSettings->set_relativeorbitchief(this->settings.relativeOrbitChief);
        vizSettings->set_spacecraftshadowbrightness(this->settings.spacecraftShadowBrightness);
        vizSettings->set_spacecraftsizemultiplier(this->settings.spacecraftSizeMultiplier);
        vizSettings->set_showlocationcommlines(this->settings.showLocationCommLines);
        vizSettings->set_showlocationcones(this->settings.showLocationCones);
        vizSettings->set_showlocationlabels(this->settings.showLocationLabels);
        
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
    std::vector<LocationPbMsg>::iterator glIt;
    for (glIt = locations.begin(); glIt != locations.end(); glIt++) {
        vizProtobufferMessage::VizMessage::Location* glp = message->add_locations();
        glp->set_stationname(glIt->stationName);
        glp->set_parentbodyname(glIt->parentBodyName);
        glp->set_fieldofview(glIt->fieldOfView*R2D);
        glp->set_range(glIt->range);
        for (int i=0; i<3; i++) {
            glp->add_r_gp_p(glIt->r_GP_P[i]);
            glp->add_ghat_p(glIt->gHat_P[i]);
        }
        for (int i=0; i<4; i++) {
            glp->add_color(glIt->color[i]);
        }
    }

    std::vector<VizSpacecraftData>::iterator scIt;
    for (scIt = scData.begin(); scIt != scData.end(); scIt++)
    {
        /*! Write SCPlus output msg */
        if (scIt->scPlusInMsg.isLinked() && scIt->scPlusInMsgStatus.dataFresh){
            vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
            scp->set_spacecraftname(scIt->spacecraftName);
            for (int i=0; i<3; i++){
                scp->add_position(scIt->scPlusMessage.r_BN_N[i]);
                scp->add_velocity(scIt->scPlusMessage.v_BN_N[i]);
                scp->add_rotation(scIt->scPlusMessage.sigma_BN[i]);
            }
//            scIt->scPlusInMsgID.dataFresh = false;

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
    if ((this->cameraConfInMsg.isLinked() && this->cameraConfMsgStatus.dataFresh)
        || this->cameraConfigBuffer.cameraID >= 0){
        /*! This corrective rotation allows unity to place the camera as is expected by the python setting. Unity has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame: z point, x horizontal, y vertical (down) */
        double sigma_CuC[3], unityCameraMRP[3]; /*! Cu is the unity Camera frame */
        v3Set(1./3, 1./3, -1./3, sigma_CuC);
        addMRP(this->cameraConfigBuffer.sigma_CB, sigma_CuC, unityCameraMRP);
        vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
        for (int j=0; j<3; j++){
            if (j < 2){
            camera->add_resolution(this->cameraConfigBuffer.resolution[j]);
            }
            camera->add_cameradir_b(unityCameraMRP[j]);
            camera->add_camerapos_b(this->cameraConfigBuffer.cameraPos_B[j]);            }
        camera->set_renderrate(this->cameraConfigBuffer.renderRate);        // Unity expects nano-seconds between images
        camera->set_cameraid(this->cameraConfigBuffer.cameraID);
        camera->set_fieldofview(this->cameraConfigBuffer.fieldOfView*R2D);  // Unity expects degrees
        camera->set_skybox(this->cameraConfigBuffer.skyBox);
        camera->set_parentname(this->cameraConfigBuffer.parentName);
    }

    /*! Write spice output msgs */
    for(size_t k=0; k<this->planetNames.size(); k++)
    {
        if (this->spiceInMsgStatus[k].dataFresh){
            vizProtobufferMessage::VizMessage::CelestialBody* spice = message->add_celestialbodies();
            spice->set_bodyname(this->planetNames.at(k));
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
            ||(this->opNavMode == 2 && ((CurrentSimNanos%this->cameraConfigBuffer.renderRate == 0 && this->cameraConfigBuffer.isOn == 1) ||this->firstPass < 11))
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
                CurrentSimNanos%this->cameraConfigBuffer.renderRate == 0 &&
                this->cameraConfigBuffer.isOn == 1)
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
                CameraImageMsgPayload imageData;
                imageData.timeTag = CurrentSimNanos;
                imageData.valid = 0;
                imageData.imagePointer = this->bskImagePtr;
                imageData.imageBufferLength = imageBufferLength;
                imageData.cameraID = this->cameraConfigBuffer.cameraID;
                imageData.imageType = 4;
                if (imageBufferLength>0){imageData.valid = 1;}
                this->opnavImageOutMsg.write(&imageData, this->moduleID, CurrentSimNanos);

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
 @param hint
 */
void message_buffer_deallocate(void *data, void *hint)
{
    free (data);
}
