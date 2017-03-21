/*
 ISC License
 
 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
 
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

#include <string>
#include <Eigen/Dense>
#include "visMessageInterface.h"
#include "../External/cspice/include/SpiceUsr.h"
#include "utilities/avsEigenSupport.h"

extern "C" {
#include "utilities/linearAlgebra.h"
#include "utilities/orbitalMotion.h"
#include "utilities/astroConstants.h"
#include "utilities/rigidBodyKinematics.h"
}


VisMessageInterface::VisMessageInterface()
{
    this->scSim = new SpacecraftSim();
    this->spiceTimeDataInMsgId = -1;
    this->spiceTimeDataInMsg = {0.0, 0.0, 0.0, 0, 0};
    this->centralBodyInMsgId = -1;
    this->centralBodyInMsg = SpicePlanetStateSimMsg();
}

VisMessageInterface::~VisMessageInterface()
{
    delete this->scSim;
    delete this->openglIo;
}

void VisMessageInterface::SelfInit()
{
    this->openglIo = new OpenGLIO();
    this->openglIo->initialize();
}

void VisMessageInterface::CrossInit()
{
    this->subscribeToMessages();
}

void VisMessageInterface::setMessageProcesses()
{
    
}

void VisMessageInterface::subscribeToMessages()
{
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    int64_t bufferId = messageSys->findMessageBuffer("simProcess");
    std::set<std::string> msgNames = messageSys->getUniqueMessageNames();
    
    std::set<std::string>::iterator nameItr;
    int64_t tmpMsgId = -1;
    MessageHeaderData* tmpHeader;
    VisMessageData tmpMsgData;
    int numRW = 0;
    std::string tmpMsgName = "";
    for (nameItr = msgNames.begin(); nameItr != msgNames.end(); ++nameItr)
    {
        tmpMsgName = *nameItr;
        tmpMsgId = messageSys->FindMessageID(tmpMsgName, bufferId);
        if (tmpMsgId >= 0)
        {
            // Subscribe to everything...for now
            tmpHeader = messageSys->FindMsgHeader(tmpMsgId, bufferId);
            std::string tmpMsgStructName(tmpHeader->messageStruct);
            //            if (tmpMsgStructName == "THROutputSimMsg" || tmpMsgStructName == "SpicePlanetStateSimMsg" || tmpMsgStructName == "RWSpeedIntMsg" || tmpMsgStructName == "RWConfigSimMsg") {
            tmpMsgData = VisMessageData();
            tmpMsgData.msgId = tmpMsgId;
            tmpMsgData.maxMsgSize = tmpHeader->MaxMessageSize;
            tmpMsgData.msgName = tmpMsgName;
            tmpMsgData.msgStructName = std::string(tmpHeader->messageStruct);
            tmpMsgData.msgId = messageSys->subscribeToMessage(tmpMsgName, tmpMsgData.maxMsgSize, this->moduleID);
            
            if (tmpMsgStructName == "RWConfigSimMsg") numRW++;
            if (tmpMsgStructName == "THROutputSimMsg") {
                if (tmpMsgName.substr(0,28) == "thruster_ACSThrusterDynamics")
                {
                    this->acsThrusterMsgMap[*nameItr] = tmpMsgData;
                } else {
                    this->dvThrusterMsgMap[*nameItr] = tmpMsgData;
                }
            } else {
                this->msgMap[*nameItr] = tmpMsgData;
            }
        }
    }
    
    // Now that we have all the messages from the process go resize
    // the thruster and reaction wheel vectors so that later we can
    // use std::vector at() to reference a specific index
    this->rwConfigMsgs.resize(numRW);
    this->acsThrusters.resize(this->acsThrusterMsgMap.size());
    this->dvThrusters.resize(this->dvThrusterMsgMap.size());
    
    // We also need to initialize the spacecraftSim object
    // to accept the number of wheels and thrusters etc.
    this->scSim->reactionWheels.resize(numRW);
    this->scSim->acsThrusters.resize(this->acsThrusterMsgMap.size());
    this->scSim->dvThrusters.resize(this->dvThrusterMsgMap.size());
}

void VisMessageInterface::UpdateState(uint64_t CurrentSimNanos)
{
    SpacecraftSim tmpSim;
    std::string tmpString;
    this->openglIo->receive(&tmpSim, tmpString);
    
    this->readInputMessages();
    this->mapMessagesToScSim(CurrentSimNanos);
    this->openglIo->setOutputSpacecraft(*this->scSim);
    this->openglIo->send();
}

//int OpenGLIO::loadSpiceKernel(char *kernelName, const char *dataPath)
//{
//    uint32_t CharBufferSize = 512;
//
//    char *fileName = new char[CharBufferSize];
//    SpiceChar *name = new SpiceChar[CharBufferSize];
//
//    //! Begin method steps
//    //! - The required calls come from the SPICE documentation.
//    //! - The most critical call is furnsh_c
//    strcpy(name, "REPORT");
//    erract_c("SET", CharBufferSize, name);
//    strcpy(fileName, dataPath);
//    strcat(fileName, kernelName);
//    furnsh_c(fileName);
//
//    //! - Check to see if we had trouble loading a kernel and alert user if so
//    strcpy(name, "DEFAULT");
//    erract_c("SET", CharBufferSize, name);
//    delete[] fileName;
//    delete[] name;
//    if(failed_c()) {
//        return 1;
//    }
//    return 0;
//}

void VisMessageInterface::readInputMessages()
{
    SingleMessageHeader tmpHeader;
    SystemMessaging* messageSys = SystemMessaging::GetInstance();
    VisMessageData tmpMsgData;
    std::unordered_map<std::string, VisMessageData>::iterator it;
    
    // Clear out vectors so that we can refill them
    this->rwConfigMsgs.clear();
    
    // We iterate through all messages in the
    // message Map and read messages based on message struct.
    for (it = this->msgMap.begin(); it != this->msgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        
        if (tmpMsgData.msgStructName == "SCPlusStatesSimMsg") {
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->scStateInMsg), this->moduleID);
            
        } else if (tmpMsgData.msgStructName == "RWConfigSimMsg") {
            RWConfigSimMsg tmpRWMsg;
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpRWMsg), this->moduleID);
            this->rwConfigMsgs.push_back(tmpRWMsg);
            
        } else if (tmpMsgData.msgStructName == "SpiceTimeSimMsg") {
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->scSim->spiceTime), this->moduleID);
            
        } else if (tmpMsgData.msgStructName == "SpicePlanetStateSimMsg") {
            if (tmpMsgData.msgName == "central_body_spice"){
                messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->centralBodyInMsg), this->moduleID);
                
            } else {
                SpicePlanetStateSimMsg tmpSpicePlanet;
                messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpSpicePlanet), this->moduleID);
                this->spicePlanetStates[tmpMsgData.msgName] = tmpSpicePlanet;
            }
        }
    }
    
    this->acsThrusters.clear();
    this->dvThrusters.clear();
    for (it = this->acsThrusterMsgMap.begin(); it != this->acsThrusterMsgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        THROutputSimMsg tmpThruster;
        messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpThruster), this->moduleID);
        this->acsThrusters.push_back(tmpThruster);
    }
    
    for (it = this->dvThrusterMsgMap.begin(); it != this->dvThrusterMsgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        THROutputSimMsg tmpThruster;
        messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpThruster), this->moduleID);
        this->dvThrusters.push_back(tmpThruster);
    }
}

void VisMessageInterface::setUTCCalInit(std::string UTCCalInit)
{
    this->UTCCalInit = UTCCalInit;
}

void VisMessageInterface::addRwMessageName(std::string msgName)
{
    //    this->rwInMsgNames.push_back(msgName);
    //    this->rwMsgPairs.push_back(std::pair<std::string, VisMessageData>(msgName, VisMessageData()));
}

void VisMessageInterface::addThrusterMessageName(std::string msgName)
{
    this->thrusterInMsgNames.push_back(msgName);
}

void VisMessageInterface::computeSunHeadingData()
{
    double Sc2Sun_Inrtl[3];
    double dcm_BN[3][3];
    
    v3Scale(-1.0, this->scStateInMsg.r_BN_N, Sc2Sun_Inrtl);
    v3Add(Sc2Sun_Inrtl, this->sunEphmInMsg.PositionVector, Sc2Sun_Inrtl);
    v3Normalize(Sc2Sun_Inrtl, this->scSim->sHatN);
    MRP2C(this->scStateInMsg.sigma_BN, dcm_BN);
    m33MultV3(dcm_BN, this->scSim->sHatN, this->scSim->sHatB);
}

void VisMessageInterface::mapMessagesToScSim(uint64_t currentSimNanos)
{
    double m2km = 0.001;
    int i;
    
    // map the spacecraft state
    v3Set(this->scStateInMsg.sigma_BN[0], this->scStateInMsg.sigma_BN[1], this->scStateInMsg.sigma_BN[2], this->scSim->sigma);
    v3Set(this->scStateInMsg.r_BN_N[0]*m2km, this->scStateInMsg.r_BN_N[1]*m2km, this->scStateInMsg.r_BN_N[2]*m2km, this->scSim->r_N);
    v3Set(this->scStateInMsg.v_BN_N[0]*m2km, this->scStateInMsg.v_BN_N[1]*m2km, this->scStateInMsg.v_BN_N[2]*m2km, this->scSim->v_N);
    
    for (i = 0; i < this->acsThrusters.size(); i++)
    {
        eigenVector3d2CArray(this->acsThrusters[i].thrusterLocation, this->scSim->acsThrusters[i].r_B);
        eigenVector3d2CArray(this->acsThrusters[i].thrusterDirection, this->scSim->acsThrusters[i].gt_B);
        this->scSim->acsThrusters[i].maxThrust = this->acsThrusters[i].maxThrust;
        this->scSim->acsThrusters[i].level = this->acsThrusters[i].thrustFactor;
    }
    
    for (i = 0; i < this->dvThrusters.size(); i++)
    {
        eigenVector3d2CArray(this->dvThrusters[i].thrusterLocation, this->scSim->dvThrusters[i].r_B);
        eigenVector3d2CArray(this->dvThrusters[i].thrusterDirection, this->scSim->dvThrusters[i].gt_B);
        this->scSim->dvThrusters[i].maxThrust = this->dvThrusters[i].maxThrust;
        this->scSim->dvThrusters[i].level = this->dvThrusters[i].thrustFactor;
    }
    
    for (i = 0; i < this->rwConfigMsgs.size(); i++)
    {
        this->scSim->reactionWheels[i].state = COMPONENT_ON;
        RWConfigSimMsg rwData = this->rwConfigMsgs.at(i);
        eigenVector3d2CArray(rwData.rWB_S, this->scSim->reactionWheels[i].rWB_S);
        eigenVector3d2CArray(rwData.gsHat_S, this->scSim->reactionWheels[i].gsHat_S);
        eigenVector3d2CArray(rwData.w3Hat0_B, this->scSim->reactionWheels[i].w3Hat0_B);
        eigenVector3d2CArray(rwData.w2Hat0_B, this->scSim->reactionWheels[i].w2Hat0_B);
        this->scSim->reactionWheels[i].u_current = rwData.u_current;
        this->scSim->reactionWheels[i].u_max = rwData.u_max;
        this->scSim->reactionWheels[i].u_min = rwData.u_min;
        this->scSim->reactionWheels[i].theta = rwData.theta;
        this->scSim->reactionWheels[i].Omega = rwData.Omega;
        this->scSim->reactionWheels[i].Omega_max = rwData.Omega_max;
    }
    
    // Now that we have the spacecraft state all set up we can go ahead and
    // generate the required ephemeris and orbital information.
    
    double sc2Sun_N[3]; // position vector of sun relative to the spacecraft
    // map sim time
    this->scSim->time = currentSimNanos*1.0E-9;
    // map helicentric distance to sun from spacecraft
    v3Scale(-1.0, this->scStateInMsg.r_BN_N, sc2Sun_N);
    v3Add(sc2Sun_N, this->sunEphmInMsg.PositionVector, sc2Sun_N);
    this->scSim->helioRadius =  v3Norm(sc2Sun_N)*m2km;   /* km */
    
    // map the inertial and body frame sun unit direction vectors
    this->computeSunHeadingData();
    
    // map the primary celestial body
    this->setScSimCelestialObject();
    this->setScSimOrbitalElements();
    this->setScSimJulianDate(currentSimNanos);
}

void VisMessageInterface::setScSimJulianDate(uint64_t currentSimNanos)
{
    //    if (this->scSim->spiceTime.julianDateCurrent > 0)
    //    {
    //
    //    } else {
    //        // if no reasonable JD message is read then offer the below as default
    //        // A.D. 2017 Jan 1	00:00:00.0 UT
    //        this->scSim->julianDate = 2457754.5 + currentSimNanos/1E9;
    //    }
    
}

void VisMessageInterface::setCelestialObject(int celestialObject)
{
    switch (celestialObject) {
        case CELESTIAL_SUN:
            this->scSim->celestialObject = CELESTIAL_SUN;
            break;
        case CELESTIAL_MARS:
            this->scSim->celestialObject = CELESTIAL_MARS;
            break;
        default:
            this->scSim->celestialObject = CELESTIAL_EARTH;
            break;
    }
}

void VisMessageInterface::setScSimCelestialObject()
{
    if (strcmp(centralBodyInMsg.PlanetName, "sun") == 0) {
        this->scSim->celestialObject = CELESTIAL_SUN;
        this->scSim->mu = MU_SUN;
    } else if (strcmp(centralBodyInMsg.PlanetName, "mars") == 0) {
        this->scSim->celestialObject = CELESTIAL_MARS;
        this->scSim->mu = MU_MARS;
    } else {
        this->scSim->celestialObject = CELESTIAL_EARTH;
        this->scSim->mu = MU_EARTH;
    }
}

void VisMessageInterface::setScSimOrbitalElements()
{
    rv2elem(this->scSim->mu, this->scSim->r_N, this->scSim->v_N, &this->scSim->oe);
}


//int OpenGLIO::loadSpiceKernel(char *kernelName, const char *dataPath)
//{
//    uint32_t CharBufferSize = 512;
//
//    char *fileName = new char[CharBufferSize];
//    SpiceChar *name = new SpiceChar[CharBufferSize];
//
//    //! Begin method steps
//    //! - The required calls come from the SPICE documentation.
//    //! - The most critical call is furnsh_c
//    strcpy(name, "REPORT");
//    erract_c("SET", CharBufferSize, name);
//    strcpy(fileName, dataPath);
//    strcat(fileName, kernelName);
//    furnsh_c(fileName);
//
//    //! - Check to see if we had trouble loading a kernel and alert user if so
//    strcpy(name, "DEFAULT");
//    erract_c("SET", CharBufferSize, name);
//    delete[] fileName;
//    delete[] name;
//    if(failed_c()) {
//        return 1;
//    }
//    return 0;
//}
