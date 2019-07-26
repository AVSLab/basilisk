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
#include "utilities/bsk_Print.h"
#include "utilities/rigidBodyKinematics.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

void message_buffer_deallocate(void *data, void *hint);

/*! VizInterface Constructor
 */
VizInterface::VizInterface()
{
    this->opNavMode = 0;
    this->saveFile = 0;
    this->FrameNumber= -1;
    this->numOutputBuffers = 2;
    this->scPlusInMsgName = "inertial_state_output";
    this->cssDataInMsgName = "css_sensors_data";
    this->cssConfInMsgName = "css_config_data";
    this->starTrackerInMsgName = "star_tracker_state";
    this->cameraConfInMsgName = "camera_config_data";
    this->numRW = 0;
    this->numThr = 0;
    this->planetNames = {};
    this->spacecraftName = "spacecraft";

    // turn off all Viz settings by default
    this->settings.ambient = -1.0;

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
    if (this->opNavMode == 1){
        /*! setup zeroMQ */
        this->context = zmq_ctx_new();
        this->requester_socket = zmq_socket(this->context, ZMQ_REQ);
        zmq_connect(this->requester_socket, "tcp://localhost:5556");

        void* message = malloc(4 * sizeof(char));
        memcpy(message, "PING", 4);
        zmq_msg_t request;
        
        std::cout << "Waiting for Vizard at tcp://localhost:5556" << std::endl;
        
        zmq_msg_init_data(&request, message, 4, message_buffer_deallocate, NULL);
        zmq_msg_send(&request, this->requester_socket, 0);
        char buffer[4];
        zmq_recv (this->requester_socket, buffer, 4, 0);
        zmq_send (this->requester_socket, "PING", 4, 0);
        std::cout << "Basilisk-Vizard connection made" << std::endl;
        
        /*! - Create output message for module in opNav mode */
        int imageBufferCount = 2;
        this->imageOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->opnavImageOutMsgName,sizeof(CameraImageMsg),imageBufferCount,"CirclesOpNavMsg", moduleID);

    }

    return;
}

/*! Cross initialization. Module subscribes to other messages. In viz interface, many messages are subscribed to in order to extract information from the viz and give it to the visualization tool.
 */
void VizInterface::CrossInit()
{
    MessageIdentData msgInfo;

    /*! Define CSS data input messages */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->cssDataInMsgName);
    if (msgInfo.itemFound) {
        this->cssDataInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cssDataInMsgName,
                                                                                  sizeof(CSSArraySensorIntMsg), moduleID);
        this->cssDataInMsgId.dataFresh = false;
        this->cssDataInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->cssDataInMsgId.msgID = -1;
    }

    /*! Define CSS configuration input messages */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->cameraConfInMsgName);
    if (msgInfo.itemFound) {
        this->cssConfInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cssConfInMsgName,
                                                                                        sizeof(CSSConfigFswMsg), moduleID);
        this->cssConfInMsgId.dataFresh = false;
        this->cssConfInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->cssConfInMsgId.msgID = -1;
    }

    /*! Define Camera input messages */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->cameraConfInMsgName);
    if (msgInfo.itemFound) {
        this->cameraConfMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cameraConfInMsgName,
                                                                                    sizeof(CameraConfigMsg), moduleID);
        this->cameraConfMsgId.dataFresh = false;
        this->cameraConfMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->cameraConfMsgId.msgID = -1;
    }

    /*! Define SCPlus input message */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->scPlusInMsgName);
    if (msgInfo.itemFound) {
        this->scPlusInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->scPlusInMsgName,
                sizeof(SCPlusStatesSimMsg), moduleID);
        this->scPlusInMsgID.dataFresh = false;
        this->scPlusInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->scPlusInMsgID.msgID = -1;
    }

    /*! Define Spice input message */
    {
        int i=0;
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
    
    /*! Define StarTracker input message */
    msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(this->starTrackerInMsgName);
    if (msgInfo.itemFound) {
        this->starTrackerInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->starTrackerInMsgName,
                sizeof(STSensorIntMsg), moduleID);
        this->starTrackerInMsgID.dataFresh = false;
        this->starTrackerInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    } else {
        this->starTrackerInMsgID.msgID = -1;
    }

    /*! Define RW input message */
    {
        MsgCurrStatus rwStatus;
        rwStatus.dataFresh = false;
        rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        for (int idx = 0; idx < this->numRW; idx++)
        {
            std::string tmpWheelMsgName = "rw_config_" + std::to_string(idx) + "_data";
            this->rwInMsgName.push_back(tmpWheelMsgName);
            msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(tmpWheelMsgName);
            if (msgInfo.itemFound) {
                rwStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->rwInMsgName[idx],sizeof(RWConfigLogSimMsg), moduleID);
                this->rwInMsgID.push_back(rwStatus);
            } else {
                rwStatus.msgID = -1;
                BSK_PRINT(MSG_WARNING, "RW(%d) msg requested but not found.", idx);
            }
        }
        this->rwInMessage.resize(this->rwInMsgID.size());
    }
    
    /*! Define Thr input message */
    {
        MsgCurrStatus thrStatus;
        thrStatus.dataFresh = false;
        thrStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        std::vector<ThrClusterMap>::iterator it;
        this->numThr = 0;
    for (it= this->thrMsgData.begin(); it != this->thrMsgData.end(); it++)
    {
        for(int idx=0; idx < it->thrCount; idx++) {
            std::string tmpThrustMsgName = "thruster_" + it->thrTag + "_" + std::to_string(idx) + "_data";
            msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(tmpThrustMsgName);
            if (msgInfo.itemFound) {
                thrStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(tmpThrustMsgName, sizeof(THROutputSimMsg), moduleID);
                this->thrMsgID.push_back(thrStatus);
                this->numThr++;
            } else {
                thrStatus.msgID = -1;
                BSK_PRINT(MSG_WARNING, "TH(%d) msg of tag %s requested but not found.", idx, it->thrTag.c_str());
            }
        }
    }
    this->thrOutputMessage.resize(this->thrMsgID.size());
    }

}

/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::Reset(uint64_t CurrentSimNanos)
{
    this->FrameNumber=-1;
    if (this->saveFile == 1) {
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);
    }

    this->settings.dataFresh = true;        // reset flag to transmit Vizard settings
    return;
}

/*! A method in which the module reads the content of all available bsk messages
 */
void VizInterface::ReadBSKMessages()
{
    /* Read BSK SCPlus msg */
    if (scPlusInMsgID.msgID >= 0){
    SCPlusStatesSimMsg localSCPlusArray;
    SingleMessageHeader localSCPlusHeader;
    SystemMessaging::GetInstance()->ReadMessage(scPlusInMsgID.msgID, &localSCPlusHeader,
                                                sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&localSCPlusArray));
    if(localSCPlusHeader.WriteSize > 0 && localSCPlusHeader.WriteClockNanos != scPlusInMsgID.lastTimeTag)
    {
        scPlusInMsgID.lastTimeTag = localSCPlusHeader.WriteClockNanos;
        scPlusInMsgID.dataFresh = true;
    }
    this->scPlusMessage = localSCPlusArray;
    }
    /*! Read BSK Spice constellation msg */
    {
    int i=0;
    std::vector<std::string>::iterator it;
    for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
    {
        if (this->spiceInMsgID[i].msgID != -1){
        SpicePlanetStateSimMsg localSpiceArray;
        SingleMessageHeader localSpiceHeader;
        memset(&localSpiceHeader, 0x0, sizeof(SingleMessageHeader));
        SystemMessaging::GetInstance()->ReadMessage(this->spiceInMsgID[i].msgID, &localSpiceHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&localSpiceArray));
        if(localSpiceHeader.WriteSize > 0 && localSpiceHeader.WriteClockNanos != this->spiceInMsgID[i].lastTimeTag)
        {
            this->spiceInMsgID[i].lastTimeTag = localSpiceHeader.WriteClockNanos;
            this->spiceInMsgID[i].dataFresh = true;
            this->spiceMessage[i] = localSpiceArray;
        }
        }
        i+=1;
    }
    }

    /*! Read BSK RW constellation msg */
    {
    for (int idx=0;idx< this->numRW; idx++)
    {
        if (this->rwInMsgID[idx].msgID != -1){
        RWConfigLogSimMsg localRWArray;
        SingleMessageHeader localRWHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->rwInMsgID[idx].msgID, &localRWHeader, sizeof(RWConfigLogSimMsg), reinterpret_cast<uint8_t*>(&localRWArray));
            if(localRWHeader.WriteSize > 0 && localRWHeader.WriteClockNanos != this->rwInMsgID[idx].lastTimeTag)
            {
                this->rwInMsgID[idx].lastTimeTag = localRWHeader.WriteClockNanos;
                this->rwInMsgID[idx].dataFresh = true;
                this->rwInMessage[idx] = localRWArray;
            }
        }
    }
    }
    
     /*! Read incoming Thruster constellation msg */
    {
    for (int idx=0;idx< this->numThr; idx++){
        if (this->thrMsgID[idx].msgID != -1){
            THROutputSimMsg localThrusterArray;
            SingleMessageHeader localThrusterHeader;
            SystemMessaging::GetInstance()->ReadMessage(this->thrMsgID[idx].msgID, &localThrusterHeader, sizeof(THROutputSimMsg), reinterpret_cast<uint8_t*>(&localThrusterArray));
            if(localThrusterHeader.WriteSize > 0 && localThrusterHeader.WriteClockNanos != this->thrMsgID[idx].lastTimeTag)
            {
                this->thrMsgID[idx].lastTimeTag = localThrusterHeader.WriteClockNanos;
                this->thrMsgID[idx].dataFresh = true;
                this->thrOutputMessage[idx] = localThrusterArray;
            }
        }
    }
    }
    
    
    /*! Read CSS data msg */
    if (this->cssDataInMsgId.msgID != -1){
        CSSArraySensorIntMsg localCSSDataArray;
        SingleMessageHeader localCSSDataHeader;
        SystemMessaging::GetInstance()->ReadMessage(cssDataInMsgId.msgID, &localCSSDataHeader, sizeof(CSSArraySensorIntMsg), reinterpret_cast<uint8_t*>(&localCSSDataArray));
        if(localCSSDataHeader.WriteSize > 0 && localCSSDataHeader.WriteClockNanos != cssDataInMsgId.lastTimeTag)
        {
            cssDataInMsgId.lastTimeTag = localCSSDataHeader.WriteClockNanos;
            cssDataInMsgId.dataFresh = true;
        }
    }
    
    /*! Read incoming CSS config msg */
    if (this->cssConfInMsgId.msgID != -1){
        CSSConfigFswMsg localCSSConfigArray;
        SingleMessageHeader localCSSConfigHeader;
        SystemMessaging::GetInstance()->ReadMessage(cssConfInMsgId.msgID, &localCSSConfigHeader, sizeof(CSSConfigFswMsg), reinterpret_cast<uint8_t*>(&localCSSConfigArray));
        if(localCSSConfigHeader.WriteSize > 0 && localCSSConfigHeader.WriteClockNanos != cssConfInMsgId.lastTimeTag)
        {
            cssConfInMsgId.lastTimeTag = localCSSConfigHeader.WriteClockNanos;
            cssConfInMsgId.dataFresh = true;
        }
        this->cssConfigMessage = localCSSConfigArray;
    }
    
    /*! Read incoming camera config msg */
    if (this->cameraConfMsgId.msgID != -1){
        CameraConfigMsg localCameraConfigArray;
        SingleMessageHeader localCameraConfigHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->cameraConfMsgId.msgID, &localCameraConfigHeader, sizeof(CameraConfigMsg), reinterpret_cast<uint8_t*>(&localCameraConfigArray));
        if(localCameraConfigHeader.WriteSize > 0 && localCameraConfigHeader.WriteClockNanos != this->cameraConfMsgId.lastTimeTag)
        {
            this->cameraConfMsgId.lastTimeTag = localCameraConfigHeader.WriteClockNanos;
            this->cameraConfMsgId.dataFresh = true;
        }
        this->cameraConfigMessage = localCameraConfigArray;
    }
    
    
    /*! Read incoming ST constellation msg */
    if (this->starTrackerInMsgID.msgID != -1){
        STSensorIntMsg localSTArray;
        SingleMessageHeader localSTHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->starTrackerInMsgID.msgID, &localSTHeader, sizeof(STSensorIntMsg), reinterpret_cast<uint8_t*>(&localSTArray));
        if(localSTHeader.WriteSize > 0 && localSTHeader.WriteClockNanos != this->starTrackerInMsgID.lastTimeTag)
        {
            this->starTrackerInMsgID.lastTimeTag = localSTHeader.WriteClockNanos;
            this->starTrackerInMsgID.dataFresh = true;
        }
        this->STMessage = localSTArray;
    }
}

/*! The method in which the viz_interface writes a protobuffer with the
 infomration from the simulation.
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
        if (this->settings.ambient > 8.0 || this->settings.ambient < 0.0) {
            BSK_PRINT(MSG_WARNING, "The Vizard ambient light value must be within [0,8].  A value of %f was received.", this->settings.ambient);
        }
        
        message->set_allocated_settings(vizSettings);

        this->settings.dataFresh = false;
    }

    /*! Write timestamp output msg */
    vizProtobufferMessage::VizMessage::TimeStamp* time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->FrameNumber);
    time->set_simtimeelapsed(CurrentSimNanos);
    message->set_allocated_currenttime(time);


    /*! Write SCPlus output msg */
    if (scPlusInMsgID.msgID != -1 && scPlusInMsgID.dataFresh){
        vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
        scp->set_spacecraftname(this->spacecraftName);
        for (int i=0; i<3; i++){
            scp->add_position(this->scPlusMessage.r_BN_N[i]);
            scp->add_velocity(this->scPlusMessage.v_BN_N[i]);
            scp->add_rotation(this->scPlusMessage.sigma_BN[i]);
        }
        //scPlusInMsgID.dataFresh = false;

        /*! Write RW output msg */
        for (int idx =0; idx < this->numRW; idx++)
        {
            if (rwInMsgID[idx].msgID != -1 && rwInMsgID[idx].dataFresh){
                vizProtobufferMessage::VizMessage::ReactionWheel* rwheel = scp->add_reactionwheels();
                rwheel->set_wheelspeed(this->rwInMessage[idx].Omega);
                rwheel->set_wheeltorque(this->rwInMessage[idx].u_current);
                for (int i=0; i<3; i++){
                    rwheel->add_position(this->rwInMessage[idx].rWB_B[i]);
                    rwheel->add_spinaxisvector(this->rwInMessage[idx].gsHat_B[i]);
                }
                //rwInMsgID[idx].dataFresh = false;
            }
        }

        /*! Write Thr output msg */
        for (int idx =0; idx < this->numThr; idx++)
        {
            if (thrMsgID[idx].msgID != -1 && thrMsgID[idx].dataFresh){
                vizProtobufferMessage::VizMessage::Thruster* thr = scp->add_thrusters();
                thr->set_maxthrust(this->thrOutputMessage[idx].maxThrust);
                thr->set_currentthrust(this->thrOutputMessage[idx].thrustForce);
                for (int i=0; i<3; i++){
                    thr->add_position(this->thrOutputMessage[idx].thrusterLocation[i]);
                    thr->add_thrustvector(this->thrOutputMessage[idx].thrusterDirection[i]);
                }
                //thrMsgID[idx].dataFresh = false;
            }
        }
        
        /*! Write camera output msg */
        if (cameraConfMsgId.msgID != -1 && cameraConfMsgId.dataFresh){
            /*! This corrective rotation allows unity to place the camera as is expected by the python setting. Unity has a -x pointing camera, with z vertical on the sensor, and y horizontal which is not the OpNav frame: z point, x horizontal, y vertical (down) */
            double sigma_CuC[3], sigma_BCu[3], unityCameraMRP[3]; /*! Cu is the unity Camera frame */
            v3Scale(-1, this->cameraConfigMessage.sigma_CB, sigma_BCu);
            v3Set(1./3, 1./3, -1./3, sigma_CuC);
            addMRP(sigma_BCu, sigma_CuC, unityCameraMRP);
            vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
            for (int j=0; j<3; j++){
                if (j < 2){
                camera->add_resolution(this->cameraConfigMessage.resolution[j]);
                camera->add_sensorsize(this->cameraConfigMessage.sensorSize[j]);
                }
                camera->add_cameradir_b(unityCameraMRP[j]);
                camera->add_camerapos_b(this->cameraConfigMessage.cameraPos_B[j]);            }
            camera->set_renderrate(this->cameraConfigMessage.renderRate);
            camera->set_cameraid(this->cameraConfigMessage.cameraID);
            camera->set_fieldofview(this->cameraConfigMessage.fieldOfView);
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



        /*! Write spice output msgs */
        int k=0;
        std::vector<std::string>::iterator it;
        for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
        {
            if (spiceInMsgID[k].msgID != -1 && spiceInMsgID[k].dataFresh){
                vizProtobufferMessage::VizMessage::CelestialBody* spice = message->add_celestialbodies();
                spice->set_bodyname(*it);
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
    }

    {
        google::protobuf::uint8 varIntBuffer[4];
        uint32_t byteCount = message->ByteSizeLong();
        google::protobuf::uint8 *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount, varIntBuffer);
        unsigned long varIntBytes = end - varIntBuffer;
        if (this->saveFile == 1) {
            this->outputStream->write(reinterpret_cast<char* > (varIntBuffer), varIntBytes);
        }

        /*! Enter in lock-step with the vizard to simulate a camera */
        if (this->opNavMode == 1){
            // Receive pong
            zmq_msg_t receive_buffer;
            zmq_msg_init(&receive_buffer);
            zmq_msg_recv (&receive_buffer, requester_socket, 0);
            
            /*! - send protobuffer raw over zmq_socket */
            void* serialized_message = malloc(byteCount);
            message->SerializeToArray(serialized_message, byteCount);

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
            if (CurrentSimNanos%this->cameraConfigMessage.renderRate == 0){
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
                
                /*! -- Write out the image information to the Image message */
                CameraImageMsg imageData;
                imageData.timeTag = CurrentSimNanos;
                imageData.valid = 0;
                imageData.imagePointer = imagePoint;
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
