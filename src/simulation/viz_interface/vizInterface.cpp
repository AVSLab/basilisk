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
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

void message_buffer_deallocate(void *data, void *hint);

/*! VizInterface Constructor
 */
VizInterface::VizInterface()
{
    VizSpacecraftData scData;
    this->opNavMode = 0;
    this->saveFile = 0;
    this->FrameNumber= -1;
    this->numOutputBuffers = 2;
    scData.scPlusInMsgName = "inertial_state_output";
    scData.cssDataInMsgName = "css_sensors_data";
    scData.cssConfInMsgName = "css_config_data";
    scData.starTrackerInMsgName = "star_tracker_state";
    scData.cameraConfInMsgName = "camera_config_data";
    scData.numRW = 0;
    scData.numThr = 0;
    this->scData.push_back(scData);
    this->planetNames = {};
    this->vizOutMsgName = "viz_message";
    this->outputStream = NULL;
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
    this->vizOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(this->vizOutMsgName, sizeof(VizMsg),
                                                                         this->numOutputBuffers,"VizMsg", moduleID);
    std::vector<VizSpacecraftData>::iterator it;
    
    if (this->opNavMode == 1){
        /*! setup zeroMQ */
        context = zmq_ctx_new();
        requester_socket = zmq_socket(context, ZMQ_REQ);
        zmq_connect(requester_socket, "tcp://localhost:5556");

        void* message = malloc(4 * sizeof(char));
        memcpy(message, "PING", 4);
        zmq_msg_t request;
        
        std::cout << "Waiting for Vizard at tcp://localhost:5556" << std::endl;
        
        zmq_msg_init_data(&request, message, 4, message_buffer_deallocate, NULL);
        zmq_msg_send(&request, requester_socket, 0);
        char buffer[4];
        zmq_recv (requester_socket, buffer, 4, 0);
        zmq_send (requester_socket, "PING", 4, 0);
        std::cout << "Basilisk-Vizard connection made" << std::endl;
        
        /*! - Create output message for module in opNav mode */
        int imageBufferCount = 2;
        for(it=this->scData.begin(); it != this->scData.end(); it++)
        {
            it->imageOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(it->opnavImageOutMsgName,sizeof(CameraImageMsg),imageBufferCount,"CirclesOpNavMsg", moduleID);
        }
    }

    return;
}

/*! Cross initialization. Module subscribes to other messages. In viz interface, many messages are subscribed to in order to extract information from the viz and give it to the visualization tool.
 */
void VizInterface::CrossInit()
{

    std::vector<VizSpacecraftData>::iterator it;

    for(it=scData.begin(); it != scData.end(); it++)
    {
        /*! Define CSS input messages */
        it->cssDataInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->cssDataInMsgName,
                                                                                  sizeof(CSSArraySensorIntMsg), moduleID);
        it->cssDataInMsgId.dataFresh = false;
        it->cssDataInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        it->cssConfInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->cssConfInMsgName,
                                                                                  sizeof(CSSConfigFswMsg), moduleID);
        it->cssConfInMsgId.dataFresh = false;
        it->cssConfInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        /*! Define SCPlus input message */
        it->scPlusInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->scPlusInMsgName,
                sizeof(SCPlusStatesSimMsg), moduleID);
        it->scPlusInMsgID.dataFresh = false;
        it->scPlusInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        /*! Define StarTracker input message */
        it->starTrackerInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->starTrackerInMsgName,
                sizeof(STSensorIntMsg), moduleID);
        it->starTrackerInMsgID.dataFresh = false;
        it->starTrackerInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    
        /*! Define Camera input messages */
        MessageIdentData msgInfo;
        msgInfo = SystemMessaging::GetInstance()->messagePublishSearch(it->cameraConfInMsgName);
        if (msgInfo.itemFound) {
            it->cameraConfMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->cameraConfInMsgName,
                                                                                        sizeof(CameraConfigMsg), moduleID);
            it->cameraConfMsgId.dataFresh = false;
            it->cameraConfMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        } else {
            it->cameraConfMsgId.msgID = -1;
        }

        /*! Define RW input message */
        MsgCurrStatus rwStatus;
        rwStatus.dataFresh = false;
        rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        for (int idx = 0; idx < it->numRW; idx++)
        {
            std::string tmpWheelMsgName = "rw_config_" + std::to_string(idx) + "_data";
            it->rwInMsgName.push_back(tmpWheelMsgName);

            rwStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(it->rwInMsgName[idx],sizeof(RWConfigLogSimMsg), moduleID);
            it->rwInMsgID.push_back(rwStatus);
        }
        it->rwInMessage.resize(it->rwInMsgID.size());
        
        /*! Define Thr input message */
        MsgCurrStatus thrStatus;
        thrStatus.dataFresh = false;
        thrStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
        std::vector<ThrClusterMap>::iterator it2;
        it->numThr = 0;
        for (it2= it->thrMsgData.begin(); it2 != it->thrMsgData.end(); it2++)
        {
            for(unsigned int idx=0; idx < it2->thrCount; idx++) {
                std::string tmpThrustMsgName = "thruster_" + it2->thrTag + "_" + std::to_string(idx) + "_data";
                thrStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(tmpThrustMsgName, sizeof(THROutputSimMsg), moduleID);
                it->thrMsgID.push_back(thrStatus);
                it->numThr++;
            }
        }
        it->thrOutputMessage.resize(it->thrMsgID.size());
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
        this->spiceInMsgName.push_back(planetMsgName);
        //! Subscribe to messages
        spiceStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->spiceInMsgName[i], sizeof(SpicePlanetStateSimMsg), moduleID);
        this->spiceInMsgID.push_back(spiceStatus);
        i++;
    }
    this->spiceMessage.resize(this->spiceInMsgID.size());
    }
    
}

/*! A Reset method to put the module back into a clean state
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::Reset(uint64_t CurrentSimNanos)
{
    memset(&(this->viz_msg), 0x0, sizeof(VizMsg));
    this->FrameNumber=-1;
    if (this->saveFile == 1) {
        if(this->outputStream && this->outputStream->is_open())
        {
            this->outputStream->close();
            delete this->outputStream;
        }
        this->outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);
    }
    return;
}

/*! A method in which the module reads the content of all available bsk messages
 */
void VizInterface::ReadBSKMessages()
{
    /* Read BSK SCPlus msg */
    std::vector<VizSpacecraftData>::iterator it;

    for(it=scData.begin(); it != scData.end(); it++)
    {
        if (it->scPlusInMsgID.msgID >= 0){
            SCPlusStatesSimMsg localSCPlusArray;
            SingleMessageHeader localSCPlusHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->scPlusInMsgID.msgID, &localSCPlusHeader,
                                                        sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&localSCPlusArray));
            if(localSCPlusHeader.WriteSize > 0 && localSCPlusHeader.WriteClockNanos != it->scPlusInMsgID.lastTimeTag)
            {
                it->scPlusInMsgID.lastTimeTag = localSCPlusHeader.WriteClockNanos;
                it->scPlusInMsgID.dataFresh = true;
            }
            it->scPlusMessage = localSCPlusArray;
        }
        /*! Read BSK RW constellation msg */
        {
        for (int idx=0;idx< it->numRW; idx++)
        {
            if (it->rwInMsgID[idx].msgID != -1){
            RWConfigLogSimMsg localRWArray;
            SingleMessageHeader localRWHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->rwInMsgID[idx].msgID, &localRWHeader, sizeof(RWConfigLogSimMsg), reinterpret_cast<uint8_t*>(&localRWArray));
                if(localRWHeader.WriteSize > 0 && localRWHeader.WriteClockNanos != it->rwInMsgID[idx].lastTimeTag)
                {
                    it->rwInMsgID[idx].lastTimeTag = localRWHeader.WriteClockNanos;
                    it->rwInMsgID[idx].dataFresh = true;
                    it->rwInMessage[idx] = localRWArray;
                }
            }
        }
        }
    
        /*! Read incoming Thruster constellation msg */
        {
        for (int idx=0;idx< it->numThr; idx++){
            if (it->thrMsgID[idx].msgID != -1){
                THROutputSimMsg localThrusterArray;
                SingleMessageHeader localThrusterHeader;
                SystemMessaging::GetInstance()->ReadMessage(it->thrMsgID[idx].msgID, &localThrusterHeader, sizeof(THROutputSimMsg), reinterpret_cast<uint8_t*>(&localThrusterArray));
                if(localThrusterHeader.WriteSize > 0 && localThrusterHeader.WriteClockNanos != it->thrMsgID[idx].lastTimeTag)
                {
                    it->thrMsgID[idx].lastTimeTag = localThrusterHeader.WriteClockNanos;
                    it->thrMsgID[idx].dataFresh = true;
                    it->thrOutputMessage[idx] = localThrusterArray;
                }

            }
        }
        }
    
    
        /*! Read CSS data msg */
        if (it->cssDataInMsgId.msgID != -1){
            CSSArraySensorIntMsg localCSSDataArray;
            SingleMessageHeader localCSSDataHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->cssDataInMsgId.msgID, &localCSSDataHeader, sizeof(CSSArraySensorIntMsg), reinterpret_cast<uint8_t*>(&localCSSDataArray));
            if(localCSSDataHeader.WriteSize > 0 && localCSSDataHeader.WriteClockNanos != it->cssDataInMsgId.lastTimeTag)
            {
                it->cssDataInMsgId.lastTimeTag = localCSSDataHeader.WriteClockNanos;
                it->cssDataInMsgId.dataFresh = true;
            }
        }
        
        /*! Read incoming CSS config msg */
        if (it->cssConfInMsgId.msgID != -1){
            CSSConfigFswMsg localCSSConfigArray;
            SingleMessageHeader localCSSConfigHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->cssConfInMsgId.msgID, &localCSSConfigHeader, sizeof(CSSConfigFswMsg), reinterpret_cast<uint8_t*>(&localCSSConfigArray));
            if(localCSSConfigHeader.WriteSize > 0 && localCSSConfigHeader.WriteClockNanos != it->cssConfInMsgId.lastTimeTag)
            {
                it->cssConfInMsgId.lastTimeTag = localCSSConfigHeader.WriteClockNanos;
                it->cssConfInMsgId.dataFresh = true;
            }
            it->cssConfigMessage = localCSSConfigArray;
        }
        
        /*! Read incoming camera config msg */
        if (it->cameraConfMsgId.msgID != -1){
            CameraConfigMsg localCameraConfigArray;
            SingleMessageHeader localCameraConfigHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->cameraConfMsgId.msgID, &localCameraConfigHeader, sizeof(CameraConfigMsg), reinterpret_cast<uint8_t*>(&localCameraConfigArray));
            if(localCameraConfigHeader.WriteSize > 0 && localCameraConfigHeader.WriteClockNanos != it->cameraConfMsgId.lastTimeTag)
            {
                it->cameraConfMsgId.lastTimeTag = localCameraConfigHeader.WriteClockNanos;
                it->cameraConfMsgId.dataFresh = true;
            }
            it->cameraConfigMessage = localCameraConfigArray;
        }
        
        
        /*! Read incoming ST constellation msg */
        if (it->starTrackerInMsgID.msgID != -1){
            STSensorIntMsg localSTArray;
            SingleMessageHeader localSTHeader;
            SystemMessaging::GetInstance()->ReadMessage(it->starTrackerInMsgID.msgID, &localSTHeader, sizeof(STSensorIntMsg), reinterpret_cast<uint8_t*>(&localSTArray));
            if(localSTHeader.WriteSize > 0 && localSTHeader.WriteClockNanos != it->starTrackerInMsgID.lastTimeTag)
            {
                it->starTrackerInMsgID.lastTimeTag = localSTHeader.WriteClockNanos;
                it->starTrackerInMsgID.dataFresh = true;
            }
            it->STMessage = localSTArray;
        }
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

}

/*! The method in which the viz_interface writes a protobuffer with the
 infomration from the simulation.
 @param CurrentSimNanos The current sim time in nanoseconds
 */
void VizInterface::WriteProtobuffer(uint64_t CurrentSimNanos)
{
    vizProtobufferMessage::VizMessage* message = new vizProtobufferMessage::VizMessage;


    /*! Write timestamp output msg */
    vizProtobufferMessage::VizMessage::TimeStamp* time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->FrameNumber);
    time->set_simtimeelapsed(CurrentSimNanos);
    message->set_allocated_currenttime(time);


    std::vector<VizSpacecraftData>::iterator it;

    for(it=scData.begin(); it != scData.end(); it++)
    {
        /*! Write SCPlus output msg */
        if (it->scPlusInMsgID.msgID != -1 && it->scPlusInMsgID.dataFresh){
            vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
            scp->set_spacecraftname(it->scName);
            for (int i=0; i<3; i++){
                scp->add_position(it->scPlusMessage.r_BN_N[i]);
                scp->add_velocity(it->scPlusMessage.v_BN_N[i]);
                scp->add_rotation(it->scPlusMessage.sigma_BN[i]);
            }
            //scPlusInMsgID.dataFresh = false;

            /*! Write RW output msg */
            for (int idx =0; idx < it->numRW; idx++)
            {
                if (it->rwInMsgID[idx].msgID != -1 && it->rwInMsgID[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::ReactionWheel* rwheel = scp->add_reactionwheels();
                    rwheel->set_wheelspeed(it->rwInMessage[idx].Omega);
                    rwheel->set_wheeltorque(it->rwInMessage[idx].u_current);
                    for (int i=0; i<3; i++){
                        rwheel->add_position(it->rwInMessage[idx].rWB_B[i]);
                        rwheel->add_spinaxisvector(it->rwInMessage[idx].gsHat_B[i]);
                    }
                    //rwInMsgID[idx].dataFresh = false;
                }
            }

            /*! Write Thr output msg */
            for (int idx =0; idx < it->numThr; idx++)
            {
                if (it->thrMsgID[idx].msgID != -1 && it->thrMsgID[idx].dataFresh){
                    vizProtobufferMessage::VizMessage::Thruster* thr = scp->add_thrusters();
                    thr->set_maxthrust(it->thrOutputMessage[idx].maxThrust);
                    thr->set_currentthrust(it->thrOutputMessage[idx].thrustForce);
                    for (int i=0; i<3; i++){
                        thr->add_position(it->thrOutputMessage[idx].thrusterLocation[i]);
                        thr->add_thrustvector(it->thrOutputMessage[idx].thrusterDirection[i]);
                    }
                    //thrMsgID[idx].dataFresh = false;
                }
            }
            
            /*! Write camera output msg */
            if (it->cameraConfMsgId.msgID != -1 && it->cameraConfMsgId.dataFresh){
                vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
                for (int j=0; j<3; j++){
                    if (j < 2){
                    camera->add_resolution(it->cameraConfigMessage.resolution[j]);
                    camera->add_sensorsize(it->cameraConfigMessage.sensorSize[j]);
                    }
                    camera->add_cameradir_b(it->cameraConfigMessage.sigma_BC[j]);
                    camera->add_camerapos_b(it->cameraConfigMessage.cameraPos_B[j]);            }
                camera->set_renderrate(it->cameraConfigMessage.renderRate);
                camera->set_cameraid(it->cameraConfigMessage.cameraID);
                camera->set_fieldofview(it->cameraConfigMessage.fieldOfView);
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

    /*! Write spice output msgs */
    int k=0;
    std::vector<std::string>::iterator it2;
    for(it2 = this->planetNames.begin(); it2 != this->planetNames.end(); it2++)
    {
        if (spiceInMsgID[k].msgID != -1 && spiceInMsgID[k].dataFresh){
            vizProtobufferMessage::VizMessage::CelestialBody* spice = message->add_celestialbodies();
            spice->set_bodyname(*it2);
            for (int i=0; i<3; i++){
                spice->add_position(this->spiceMessage[k].PositionVector[i]);
                spice->add_velocity(this->spiceMessage[k].VelocityVector[i]);
                for (int j=0; j<3; j++){
                    spice->add_rotation(this->spiceMessage[k].J20002Pfix[i][j]);
                }
            }
//            spiceInMsgID[k].dataFresh = false;
        }
        k++;
    }

    {
        google::protobuf::uint8 varIntBuffer[4];
        uint32_t byteCount = message->ByteSizeLong();
        google::protobuf::uint8 *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount, varIntBuffer);
        unsigned long varIntBytes = end - varIntBuffer;
        //std::cout << "byteCount = " << byteCount << std::endl;
        if (this->saveFile == 1) {
            this->outputStream->write(reinterpret_cast<char* > (varIntBuffer), varIntBytes);
        }
        
        /* Viz Message Buffer */
        memset(&(this->viz_msg), 0x0, sizeof(VizMsg));
        void* serialized_viz_msg = malloc(byteCount);
        message->SerializeToArray(serialized_viz_msg, byteCount);
        memcpy(&(this->viz_msg.buffer), serialized_viz_msg, byteCount);
        viz_msg.byte_count = byteCount;
        
        SystemMessaging::GetInstance()->WriteMessage(this->vizOutMsgID, CurrentSimNanos, sizeof(VizMsg),
                                                      reinterpret_cast<uint8_t *>(&(this->viz_msg)), this->moduleID);
        
        

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
            //if (CurrentSimNanos%this->cameraConfigMessage.renderRate == 0){
            //    char buffer[10];
            //    zmq_recv(requester_socket, buffer, 10, 0);
            //    /*! -- Send request */
            //    void* img_message = malloc(13 * sizeof(char));
            //    memcpy(img_message, "REQUEST_IMAGE", 13);
            //    zmq_msg_t img_request;
            //    zmq_msg_init_data(&img_request, img_message, 13, message_buffer_deallocate, NULL);
            //    zmq_msg_send(&img_request, requester_socket, 0);
            //    
            //    zmq_msg_t length;
            //    zmq_msg_t image;
            //    zmq_msg_init(&length);
            //    zmq_msg_init(&image);
            //    zmq_msg_recv(&length, requester_socket, 0);
            //    zmq_msg_recv(&image, requester_socket, 0);
            //    
            //    int32_t *lengthPoint= (int32_t *)zmq_msg_data(&length);
            //    void *imagePoint= zmq_msg_data(&image);
            //    int32_t length_unswapped = *lengthPoint;
            //    /*! --  Endianness switch for the length of the buffer */
            //    int32_t imageBufferLength =((length_unswapped>>24)&0xff) | // move byte 3 to byte 0
            //    ((length_unswapped<<8)&0xff0000) | // move byte 1 to byte 2
            //    ((length_unswapped>>8)&0xff00) | // move byte 2 to byte 1
            //    ((length_unswapped<<24)&0xff000000); // byte 0 to byte 3
            //    
            //    /*! -- Write out the image information to the Image message */
            //    CameraImageMsg imageData;
            //    imageData.imagePointer = imagePoint;
            //    imageData.imageBufferLength = imageBufferLength;
            //    imageData.cameraID = this->cameraConfigMessage.cameraID;
            //    imageData.imageType = 4;
            //    SystemMessaging::GetInstance()->WriteMessage(this->imageOutMsgID, CurrentSimNanos, sizeof(CameraImageMsg), reinterpret_cast<uint8_t *>(&imageData), this->moduleID);

            //    /*! -- Clean the messages to avoid memory leaks */
            //    zmq_msg_close(&length);
            //    zmq_msg_close(&image);
            //    
            //    /*! -- Ping the Viz back to continue the lock-step */
            //    void* keep_alive = malloc(4 * sizeof(char));
            //    memcpy(keep_alive, "PING", 4);
            //    zmq_msg_t request_life;
            //    zmq_msg_init_data(&request_life, keep_alive, 4, message_buffer_deallocate, NULL);
            //    zmq_msg_send(&request_life, requester_socket, 0);
            //    return;
            //    
            //}
            
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
