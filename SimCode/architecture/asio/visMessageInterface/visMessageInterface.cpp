//
//  visMessageInterface.cpp
//  AVS basilisk
//
//  Created by Patrick Kenneally on 2/1/17.
//
//

#include "visMessageInterface.h"
#include "../External/cspice/include/SpiceUsr.h"
#include "utilities/avsEigenSupport.h"
#include <Eigen/Dense>

extern "C" {
#include "utilities/linearAlgebra.h"
#include "utilities/orbitalMotion.h"
#include "utilities/astroConstants.h"
#include "utilities/rigidBodyKinematics.h"
}

#include <string>


VisMessageInterface::VisMessageInterface()
{
    this->scSim = new SpacecraftSim();
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
    int numTHR = 0;
    for (nameItr = msgNames.begin(); nameItr != msgNames.end(); ++nameItr)
    {
        tmpMsgId = messageSys->FindMessageID(*nameItr, bufferId);
        if (tmpMsgId >=0)
        {
            if (tmpMsgData.msgStructName == "THROutputSimMsg" || "RWConfigSimMsg" || "SpicePlanetStateSimMsg" || "RWSpeedIntMsg") {
                if (tmpMsgData.msgStructName == "THROutputSimMsg") {
                    numTHR++;
                } else if (tmpMsgData.msgStructName == "RWConfigSimMsg") {
                    numRW++;
                }
                tmpHeader = messageSys->FindMsgHeader(tmpMsgId, bufferId);
                tmpMsgData = VisMessageData();
                tmpMsgData.maxMsgSize = tmpHeader->MaxMessageSize;
                tmpMsgData.msgName = *nameItr;
                tmpMsgData.msgStructName = std::string(tmpHeader->messageStruct);
                tmpMsgData.msgId = messageSys->subscribeToMessage(*nameItr, tmpMsgData.maxMsgSize, this->moduleID);
                this->msgMap[*nameItr] = tmpMsgData;
            }
        }
    }
    
    // Now that we have all the messages from the process go resize
    // the thruster and reaction wheel vectors so that later we can
    // use std::vector at() to reference a specific index
    this->reactionWheels.resize(numRW);
    this->thrusters.resize(numTHR);
    
    // We also need to initialize the spacecraftSim object
    // to accept the number of wheels and thrusters etc.
    this->scSim->reactionWheels.resize(numRW);
    this->scSim->acsThrusters.resize(numTHR);
}

void VisMessageInterface::UpdateState(uint64_t CurrentSimNanos)
{
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
    
    // We iterate through all messages in the
    // message Map and  read messages based on message struct.
    for (it = this->msgMap.begin(); it != this->msgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        
        if (tmpMsgData.msgStructName == "SCPlusStatesSimMsg") {
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->scStateInMsg), this->moduleID);
        } else if (tmpMsgData.msgStructName == "SpicePlanetStateSimMsg") {
            if (tmpMsgData.msgName == "central_body_spice"){
                SpicePlanetStateSimMsg tmpSpicePlanet;
                messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpSpicePlanet), this->moduleID);
                this->centralBodyInMsg = tmpSpicePlanet;
            } else {
            SpicePlanetStateSimMsg tmpSpicePlanet;
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpSpicePlanet), this->moduleID);
            this->spicePlanetStates[tmpMsgData.msgName] = tmpSpicePlanet;
            }
        }
    }
    
    int idx = 0;
    for (it = this->rwMsgMap.begin(); it != this->rwMsgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->reactionWheels.at(idx)), this->moduleID);
        idx++;
    }
    
    idx = 0;
    for (it = this->thursterMsgMap.begin(); it != this->thursterMsgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->thrusters.at(idx)), this->moduleID);
        idx++;
    }
}


void VisMessageInterface::setUTCCalInit(std::string UTCCalInit)
{
    this->UTCCalInit = UTCCalInit;
}

void VisMessageInterface::addRwMessageName(std::string msgName)
{
    this->rwInMsgNames.push_back(msgName);
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
    for (i = 0; i < this->thrusters.size(); i++)
    {
        eigenVector3d2CArray(this->thrusters[i].thrusterLocation, this->scSim->acsThrusters[i].r_B);
        eigenVector3d2CArray(this->thrusters[i].thrusterDirection, this->scSim->acsThrusters[i].gt_B);
        this->scSim->acsThrusters[i].maxThrust = this->thrusters[i].maxThrust;
        this->scSim->acsThrusters[i].level = this->thrusters[i].thrustFactor;
    }

    // map the spacecraft state
    v3Set(this->scStateInMsg.sigma_BN[0], this->scStateInMsg.sigma_BN[1], this->scStateInMsg.sigma_BN[2], this->scSim->sigma);
    v3Set(this->scStateInMsg.r_BN_N[0]*m2km, this->scStateInMsg.r_BN_N[1]*m2km, this->scStateInMsg.r_BN_N[2]*m2km, this->scSim->r_N);
    v3Set(this->scStateInMsg.v_BN_N[0]*m2km, this->scStateInMsg.v_BN_N[1]*m2km, this->scStateInMsg.v_BN_N[2]*m2km, this->scSim->v_N);
    
    for (i = 0; i < this->thrusters.size(); i++)
    {
        eigenVector3d2CArray(thrusters[i].thrusterLocation, this->scSim->acsThrusters[i].r_B);
        eigenVector3d2CArray(thrusters[i].thrusterDirection, this->scSim->acsThrusters[i].gt_B);
        scSim->acsThrusters[i].maxThrust = thrusters[i].maxThrust;
        scSim->acsThrusters[i].level = thrusters[i].thrustFactor;
    }

    for (i = 0; i < this->reactionWheels.size(); i++)
    {
        this->scSim->reactionWheels[i].state = COMPONENT_ON;
        RWConfigSimMsg rwData = this->reactionWheels.at(i);
        eigenVector3d2CArray(rwData.rWB_B, this->scSim->reactionWheels[i].rWB_B);
        eigenVector3d2CArray(rwData.gsHat_B, this->scSim->reactionWheels[i].gsHat_B);
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
//    str2et_c(this->UTCCalInit.c_str(), &this->scSim->ics.ET0);
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
