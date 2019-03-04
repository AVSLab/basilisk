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


#include "vizInterface.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "architecture/messaging/system_messaging.h"
#include "sensors/sun_sensor/coarse_sun_sensor.h"
#include "utilities/linearAlgebra.h"
#include "vizMessage.pb.h"
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <fstream>
#include <iostream>
#include <cstdio>
#include <architecture/messaging/system_messaging.h>

void message_buffer_deallocate(void *data, void *hint);


VizInterface::VizInterface()
{
    this->liveStream = 0;
    this->FrameNumber=-1;
    this->numOutputBuffers = 2;
    this->scPlusInMsgName = "inertial_state_output";
    this->cssDataInMsgName = "css_sensors_data";
    this->cssConfInMsgName = "css_config_data";
    this->starTrackerInMsgName = "star_tracker_state";
    this->cameraConfInMsgName = "camera_config_data";
    this->numRW = 0;
    this->numThr = 0;
    this->planetNames = {};
    return;
}


VizInterface::~VizInterface()
{
    return;
}

void VizInterface::SelfInit()
{
    // This module does not output messages, but files containing the protobuffers for Vizard
    return;
}

void VizInterface::CrossInit()
{
    /* Define CSS input messages */
    this->cssDataInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cssDataInMsgName,
                                                                              sizeof(CSSArraySensorIntMsg), moduleID);
    this->cssDataInMsgId.dataFresh = false;
    this->cssDataInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    this->cssConfInMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cssConfInMsgName,
                                                                              sizeof(CSSConfigFswMsg), moduleID);
    this->cssConfInMsgId.dataFresh = false;
    this->cssConfInMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    
    /* Define Camera input messages */
    this->cameraConfMsgId.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->cameraConfInMsgName,
                                                                                    sizeof(CameraConfigMsg), moduleID);
    this->cameraConfMsgId.dataFresh = false;
    this->cameraConfMsgId.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    
    /* Define SCPlus input message */
    this->scPlusInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(scPlusInMsgName,
            sizeof(SCPlusStatesSimMsg), moduleID);
    this->scPlusInMsgID.dataFresh = false;
    this->scPlusInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;

    /* Define Spice input message */
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
        spiceStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->spiceInMsgName[i],
                                                           sizeof(SpicePlanetStateSimMsg), moduleID);
        this->spiceInMsgID.push_back(spiceStatus);
        i++;
    }
    this->spiceMessage.resize(this->spiceInMsgID.size());
    }
    
    /* Define StarTracker input message */
    this->starTrackerInMsgID.msgID = SystemMessaging::GetInstance()->subscribeToMessage(starTrackerInMsgName,
            sizeof(STSensorIntMsg), moduleID);
    this->starTrackerInMsgID.dataFresh = false;
    this->starTrackerInMsgID.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    
    /* Define RW input message */
    {
        MsgCurrStatus rwStatus;
        rwStatus.dataFresh = false;
        rwStatus.lastTimeTag = 0xFFFFFFFFFFFFFFFF;
    for (int idx = 0; idx < this->numRW; idx++)
    {
        std::string tmpWheelMsgName = "rw_config_" + std::to_string(idx) + "_data";
        this->rwInMsgName.push_back(tmpWheelMsgName);

        rwStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(this->rwInMsgName[idx],
                                                                            sizeof(RWConfigLogSimMsg), moduleID);

        this->rwInMsgID.push_back(rwStatus);
    }
    this->rwInMessage.resize(this->rwInMsgID.size());
    }
    
    /* Define Thr input message */
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
            thrStatus.msgID = SystemMessaging::GetInstance()->subscribeToMessage(tmpThrustMsgName,
                                                                                 sizeof(THROutputSimMsg), moduleID);
            this->thrMsgID.push_back(thrStatus);
            this->numThr++;
        }
    }
    this->thrOutputMessage.resize(this->thrMsgID.size());
    }

}

void VizInterface::Reset(uint64_t CurrentSimNanos)
{
    this->FrameNumber=-1;
    outputStream = new std::ofstream(this->protoFilename, std::ios::out |std::ios::binary);


    return;
}


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
    /* Read BSK Spice constellation msg */
    {
    int i=0;
    std::vector<std::string>::iterator it;
    for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
    {
        if (this->spiceInMsgID[i].msgID != -1){
        SpicePlanetStateSimMsg localSpiceArray;
        SingleMessageHeader localSpiceHeader;
        memset(&localSpiceHeader, 0x0, sizeof(SingleMessageHeader));
        SystemMessaging::GetInstance()->ReadMessage(this->spiceInMsgID[i].msgID, &localSpiceHeader, sizeof(SpicePlanetStateSimMsg),
                                                    reinterpret_cast<uint8_t*>(&localSpiceArray));
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

    /* Read BSK RW constellation msg */
    {
    for (int idx=0;idx< this->numRW; idx++)
    {
        if (this->rwInMsgID[idx].msgID != -1){
        RWConfigLogSimMsg localRWArray;
        SingleMessageHeader localRWHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->rwInMsgID[idx].msgID, &localRWHeader,
                                                    sizeof(RWConfigLogSimMsg), reinterpret_cast<uint8_t*>(&localRWArray));
            if(localRWHeader.WriteSize > 0 && localRWHeader.WriteClockNanos != this->rwInMsgID[idx].lastTimeTag)
            {
                this->rwInMsgID[idx].lastTimeTag = localRWHeader.WriteClockNanos;
                this->rwInMsgID[idx].dataFresh = true;
                this->rwInMessage[idx] = localRWArray;
            }


        }
    }
    }
    
     // Read incoming Thruster constellation msg
    {
    for (int idx=0;idx< this->numThr; idx++)
    {
        if (this->thrMsgID[idx].msgID != -1){
        THROutputSimMsg localThrusterArray;
        SingleMessageHeader localThrusterHeader;
        SystemMessaging::GetInstance()->ReadMessage(this->thrMsgID[idx].msgID, &localThrusterHeader,
                                                    sizeof(THROutputSimMsg), reinterpret_cast<uint8_t*>(&localThrusterArray));
            if(localThrusterHeader.WriteSize > 0 && localThrusterHeader.WriteClockNanos != this->thrMsgID[idx].lastTimeTag)
            {
                this->thrMsgID[idx].lastTimeTag = localThrusterHeader.WriteClockNanos;
                this->thrMsgID[idx].dataFresh = true;
                this->thrOutputMessage[idx] = localThrusterArray;
            }

        }
    }
    }
    
    
    /* Read CSS data msg */
    if (this->cssDataInMsgId.msgID != -1){
        CSSArraySensorIntMsg localCSSDataArray;
        SingleMessageHeader localCSSDataHeader;
        SystemMessaging::GetInstance()->ReadMessage(cssDataInMsgId.msgID, &localCSSDataHeader, sizeof(CSSArraySensorIntMsg),
                                                    reinterpret_cast<uint8_t*>(&localCSSDataArray));

        if(localCSSDataHeader.WriteSize > 0 && localCSSDataHeader.WriteClockNanos != cssDataInMsgId.lastTimeTag)
        {
            cssDataInMsgId.lastTimeTag = localCSSDataHeader.WriteClockNanos;
            cssDataInMsgId.dataFresh = true;
        }
    }
    
    /* Read incoming CSS config msg */
    if (this->cssConfInMsgId.msgID != -1){
        CSSConfigFswMsg localCSSConfigArray;
        SingleMessageHeader localCSSConfigHeader;
        SystemMessaging::GetInstance()->ReadMessage(cssConfInMsgId.msgID, &localCSSConfigHeader, sizeof(CSSConfigFswMsg),
                                                    reinterpret_cast<uint8_t*>(&localCSSConfigArray));
        if(localCSSConfigHeader.WriteSize > 0 && localCSSConfigHeader.WriteClockNanos != cssConfInMsgId.lastTimeTag)
        {
            cssConfInMsgId.lastTimeTag = localCSSConfigHeader.WriteClockNanos;
            cssConfInMsgId.dataFresh = true;
        }
        this->cssConfigMessage = localCSSConfigArray;
    }
    
    /* Read incoming camera config msg */
    if (this->cameraConfMsgId.msgID != -1){
        CameraConfigMsg localCameraConfigArray;
        SingleMessageHeader localCameraConfigHeader;
        SystemMessaging::GetInstance()->ReadMessage(cameraConfMsgId.msgID, &localCameraConfigHeader, sizeof(CameraConfigMsg),
                                                    reinterpret_cast<uint8_t*>(&localCameraConfigArray));
        if(localCameraConfigHeader.WriteSize > 0 && localCameraConfigHeader.WriteClockNanos != cssConfInMsgId.lastTimeTag)
        {
            cameraConfMsgId.lastTimeTag = localCameraConfigHeader.WriteClockNanos;
            cameraConfMsgId.dataFresh = true;
        }
        this->cameraConfigMessage = localCameraConfigArray;
    }
    
    
    /* Read incoming ST constellation msg */
    if (this->starTrackerInMsgID.msgID != -1){
        STSensorIntMsg localSTArray;
        SingleMessageHeader localSTHeader;
        SystemMessaging::GetInstance()->ReadMessage(starTrackerInMsgID.msgID, &localSTHeader,
                                                    sizeof(STSensorIntMsg), reinterpret_cast<uint8_t*>(&localSTArray));
        if(localSTHeader.WriteSize > 0 && localSTHeader.WriteClockNanos != starTrackerInMsgID.lastTimeTag)
        {
            starTrackerInMsgID.lastTimeTag = localSTHeader.WriteClockNanos;
            starTrackerInMsgID.dataFresh = true;
        }
        this->STMessage = localSTArray;
    }
}

void VizInterface::WriteProtobuffer(uint64_t CurrentSimNanos)
{
    vizProtobufferMessage::VizMessage* message = new vizProtobufferMessage::VizMessage;


    /* Write timestamp output msg */
    vizProtobufferMessage::VizMessage::TimeStamp* time = new vizProtobufferMessage::VizMessage::TimeStamp;
    time->set_framenumber(this->FrameNumber);
    time->set_simtimeelapsed(CurrentSimNanos);
    message->set_allocated_currenttime(time);


    /* Write SCPlus output msg */
    if (scPlusInMsgID.msgID != -1 && scPlusInMsgID.dataFresh){
        vizProtobufferMessage::VizMessage::Spacecraft* scp = message->add_spacecraft();
        scp->set_spacecraftname("inertial");
        for (int i=0; i<3; i++){
            scp->add_position(this->scPlusMessage.r_BN_N[i]);
            scp->add_velocity(this->scPlusMessage.v_BN_N[i]);
            scp->add_rotation(this->scPlusMessage.sigma_BN[i]);
        }
        //scPlusInMsgID.dataFresh = false;

        /* Write RW output msg */
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

        /* Write Thr output msg */
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
        
        // Write camera output msg
        if (cameraConfMsgId.msgID != -1 && cameraConfMsgId.dataFresh){
            vizProtobufferMessage::VizMessage::CameraConfig* camera = message->add_cameras();
            for (int j=0; j<3; j++){
                if (j < 2){
                camera->add_resolution(this->cameraConfigMessage.resolution[j]);
                camera->add_sensorsize(this->cameraConfigMessage.sensorSize[j]);
                }
                camera->add_cameradir_b(this->cameraConfigMessage.cameraDir_B[j]);
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



        /* Write spice output msgs */
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
                //spiceInMsgID[k].dataFresh = false;
            }
            k++;
        }
    }

    {
        google::protobuf::uint8 varIntBuffer[4];
        uint32_t byteCount = message->ByteSizeLong();
        google::protobuf::uint8 *end = google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(byteCount, varIntBuffer);
        unsigned long varIntBytes = end - varIntBuffer;
        this->outputStream->write(reinterpret_cast<char* > (varIntBuffer), varIntBytes);

        // this is actually a pretty good place to receive the pong
        // we should think about a push pull architecture if we have performance problems
        // receive pong
        char buffer[10];
        zmq_recv (requester_socket, buffer, 10, 0);
        // send protobuffer raw over zmq_socket as ping
        std::string serialized_message;
        message->SerializeToString(&serialized_message);
        zmq_send(requester_socket, serialized_message.c_str(), serialized_message.length(), 0);

        // Write protobuffer to file
        if (!message->SerializeToOstream(this->outputStream)) {
            std::cerr << "Failed to write part book." << std::endl;
            return;
        }
    }


    delete message;
    google::protobuf::ShutdownProtobufLibrary();


}


void VizInterface::ReadVizMessage(std::string protoFilename){
    
    vizProtobufferMessage::VizMessage* message = new vizProtobufferMessage::VizMessage;

    // Read.
//    std::fstream input(this->protoFilename, std::ios::in | std::ios::binary);
//    if (!input) {
//        std::cout << this->protoFilename << ": File not found.  Creating a new file." << std::endl;
//    } else if (!message->ParseFromIstream(&input)) {
//        std::cerr << "Failed to parse address book." << std::endl;
//        return;
//    }
    
    // Read the timestamp data
    const  vizProtobufferMessage::VizMessage::TimeStamp time = message->currenttime();
    std::cout << "  Frame number is :" << time.framenumber() << std::endl;
    std::cout << "  Current sim time in Nano is :" << time.simtimeelapsed() << std::endl;
    
    // Read the spacecraft data
    for (int i = 0; i < message->spacecraft_size(); i++) {
        const  vizProtobufferMessage::VizMessage::Spacecraft sc = message->spacecraft(i);
        
        if (sc.spacecraftname()!= "") {
            std::cout << "  Our spacecraft is : " << sc.spacecraftname() << std::endl;
        }
        
        for (int j = 0; j < sc.position_size(); j++) {
            std::cout << "  Position component " << j << " is : " << sc.position(j) << std::endl;
            std::cout << "  Velocity component " << j << " is : " << sc.velocity(j) << std::endl;
            std::cout << "  MRP component " << j << " is : " << sc.rotation(j) << std::endl;
        }
        
        // Read the RW data
        for (int k = 0; k < sc.reactionwheels_size(); k++) {
            const  vizProtobufferMessage::VizMessage::ReactionWheel rw = sc.reactionwheels(k);

            std::cout << " RW " << k <<"'s wheel speed" << rw.wheelspeed() << std::endl;
            std::cout << " RW " << k <<"'s wheel torque" << rw.wheeltorque() << std::endl;
            for (int j = 0; j < rw.position_size(); j++) {
                std::cout << " RW " << k <<"'s position component " << j << " is : " << rw.position(j) << std::endl;
                std::cout << " RW " << k <<"'s spin axis component " << j << " is : "  << rw.spinaxisvector(j) << std::endl;
            }
        }
        
        // Read the THR data
        for (int k = 0; k < sc.thrusters_size(); k++) {
            const  vizProtobufferMessage::VizMessage::Thruster thr = sc.thrusters(k);
            
            std::cout << " Thruster " << k <<"'s max thrust is " << thr.maxthrust() << std::endl;
            std::cout << " Thruster " << k <<"'s current thrust is " << thr.currentthrust() << std::endl;
            for (int j = 0; j < thr.position_size(); j++) {
                std::cout << " Thruster " << k <<"'s position component " << j << " is : " << thr.position(j) << std::endl;
                std::cout << " Thruster " << k <<"'s direction component  " << j << " is : "  << thr.thrustvector(j) << std::endl;
            }
        }
        
        // Read the CSS data
        for (int k = 0; k < sc.css_size(); k++) {
            const  vizProtobufferMessage::VizMessage::CoarseSunSensor css = sc.css(k);
            std::cout << " CSS " << k <<"'s measurement is : " << css.currentmsmt() << std::endl;
            for (int j = 0; j < css.position_size(); j++) {
                std::cout << " CSS " << k <<"'s position component " << j << " is : " << css.position(j) << std::endl;
            for (int j = 0; j < css.normalvector_size(); j++) {
                std::cout << " CSS " << k <<"'s normal component " << j << " is : " << css.normalvector(j) << std::endl;
            }

            }
        }
        
        // Read the ST data
        const  vizProtobufferMessage::VizMessage::StarTracker st = sc.startrackers(0);
        std::cout << " ST's FOV width is : " << st.fieldofviewwidth() << std::endl;
        std::cout << " ST's FOV height is : " << st.fieldofviewheight() << std::endl;
        for (int j = 0; j < st.rotation_size(); j++) {
            std::cout << " ST " << j <<"'s measurement is : " << st.rotation(j) << std::endl;
        }
        for (int j = 0; j < st.position_size(); j++) {
            std::cout << " ST " << j <<"'s position is : " << st.position(j) << std::endl;
        }
    }
    
    
    // Read the spice data
    for (int i = 0; i < message->celestialbodies_size(); i++) {
        const  vizProtobufferMessage::VizMessage::CelestialBody spice = message->celestialbodies(i);
        
        if (spice.bodyname() != "") {
            std::cout << "  Our body is : The " << spice.bodyname() << std::endl;
        }
        
        for (int j = 0; j < spice.position_size(); j++) {
            std::cout << "  Planet position component " << j << " is : " << spice.position(j) << std::endl;
            std::cout << "  Planet Velocity component " << j << " is : " << spice.velocity(j) << std::endl;
        }
        for (int j = 0; j < spice.rotation_size(); j++) {
            std::cout << "  Planet Pfix " << j << " is :" << spice.rotation(j) << std::endl;
        }
    }
    
    
}


void VizInterface::UpdateState(uint64_t CurrentSimNanos)
{

    this->FrameNumber+=1;
    
    ReadBSKMessages();
    if(CurrentSimNanos > 0) {
        WriteProtobuffer(CurrentSimNanos);
    }
//    ReadVizMessage(this->protoFilename);

}


void message_buffer_deallocate(void *data, void *hint)
{
    free (data);
}
