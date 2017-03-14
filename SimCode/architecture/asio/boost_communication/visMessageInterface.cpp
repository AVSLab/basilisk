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

#define THRUSTER_MSG_STRUCT_NAME "ThrusterOutputData"
#define RW_MSG_STRUCT_NAME "ReactionWheelConfigData"

VisMessageInterface::VisMessageInterface()
{
    this->scSim = new SpacecraftSim();
}

VisMessageInterface::~VisMessageInterface()
{
    
}

void VisMessageInterface::SelfInit()
{
    this->openglIo = new OpenGLIO();
    this->openglIo->initialize();
}

void VisMessageInterface::CrossInit()
{
    this->subscribeToMessages();
//    SystemMessaging *messageSys = SystemMessaging::GetInstance();
//
//    this->stateInMsgId = messageSys->subscribeToMessage(this->stateInMsgName, sizeof(SCPlusOutputStateData), moduleID);
//    this->sunEphmInMsgId = messageSys->subscribeToMessage(this->sunEphmInMsgName, sizeof(SpicePlanetState), moduleID);
//    this->centralBodyInMsgId = messageSys->subscribeToMessage(this->centralBodyInMsgName, sizeof(SpicePlanetState), moduleID);
//    this->spiceTimeDataInMsgId = messageSys->subscribeToMessage(this->spiceTimeDataInMsgName, sizeof(SpiceTimeOutput), this->moduleID);
//
//    int i;
//    for (i = 0; i < this->rwInMsgNames.size(); i++)
//    {
//        this->rwInMsgIds.push_back(messageSys->subscribeToMessage(this->rwInMsgNames.at(i), sizeof(ReactionWheelConfigData), moduleID));
//    }
//    this->reactionWheels.resize(i);
//
//    for (i = 0; i < this->thrusterInMsgNames.size(); i++)
//    {
//        this->thrusterInMsgIds.push_back(messageSys->subscribeToMessage(this->thrusterInMsgNames.at(i), sizeof(ThrusterOutputData), moduleID));
//    }
//    this->thrusters.resize(i);
//
//    for(i = 0; i < this->planetInMsgNames.size(); i++)
//    {
//        this->planetInMsgIds.push_back(messageSys->subscribeToMessage(this->planetInMsgNames.at(i), sizeof(SpicePlanetState), moduleID));
//    }
//    this->planets.resize(i);

    // Set scSim object initial ephemeris time
//    if(this->loadSpiceKernel((char *)"naif0011.tls", this->spiceDataPath.c_str())) {
//        printf("Unable to load %s", "naif0010.tls");
//    }
//    str2et_c(this->UTCCalInit.c_str(), &this->scSim->ics.ET0);
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
    for (nameItr = msgNames.begin(); nameItr != msgNames.end(); ++nameItr)
    {
        tmpMsgId = messageSys->FindMessageID(*nameItr, bufferId);
        if (tmpMsgId >=0)
        {
            tmpHeader = messageSys->FindMsgHeader(tmpMsgId, bufferId);
            tmpMsgData = VisMessageData();
            tmpMsgData.maxMsgSize = tmpHeader->MaxMessageSize;
            tmpMsgData.msgName = *nameItr;
            tmpMsgData.msgStructName = std::string(tmpHeader->messageStruct);
            tmpMsgData.msgId = messageSys->subscribeToMessage(*nameItr, tmpMsgData.maxMsgSize, this->moduleID);
            
            if (tmpMsgData.msgStructName == THRUSTER_MSG_STRUCT_NAME) {
                this->thursterMsgMap[*nameItr] = tmpMsgData;
            } else if (tmpMsgData.msgStructName == RW_MSG_STRUCT_NAME) {
                this->rwMsgMap[*nameItr] = tmpMsgData;
            }

            this->msgMap[*nameItr] = tmpMsgData;
        }
    }
    
    // Now that we have all the messages from the process go resize
    // the thruster and reaction wheel vectors so that later we can
    // use std::vector at() to reference a specific index
    this->reactionWheels.resize(this->rwMsgMap.size());
    this->thrusters.resize(this->thursterMsgMap.size());
    
    // We also need to initialize the spacecraftSim object
    // to accept the number of wheels and thrusters etc.
    this->scSim->reactionWheels.resize(this->rwMsgMap.size());
    this->scSim->acsThrusters.resize(this->thursterMsgMap.size());
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
    
    for (it = this->msgMap.begin(); it != this->msgMap.end(); ++it)
    {
        tmpMsgData = (*it).second;
        
        if (tmpMsgData.msgStructName == "SCPlusOutputStateData") {
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->scStateInMsgBuffer), this->moduleID);
        } else if (tmpMsgData.msgStructName == "RWSpeedData") {
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&this->rwSpeeds), this->moduleID);
        } else if (tmpMsgData.msgStructName == "SpicePlanetState") {
            SpicePlanetState tmpSpicePlanet;
            messageSys->ReadMessage(tmpMsgData.msgId, &tmpHeader, tmpMsgData.maxMsgSize, reinterpret_cast<uint8_t*>(&tmpSpicePlanet), this->moduleID);
            this->spicePlanetStates[tmpMsgData.msgName] = tmpSpicePlanet;
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
    
    v3Scale(-1.0, this->scStateInMsgBuffer.r_BN_N, Sc2Sun_Inrtl);
    v3Add(Sc2Sun_Inrtl, this->sunEphmInMsgBuffer.PositionVector, Sc2Sun_Inrtl);
    v3Normalize(Sc2Sun_Inrtl, this->scSim->sHatN);
    MRP2C(this->scStateInMsgBuffer.sigma_BN, dcm_BN);
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
    
    
//    double sc2Sun_N[3]; // position vector of sun relative to the spacecraft
//    int i = 0;
//    
//    // map sim time
//    this->scSim->time = currentSimNanos*1.0E-9;
//    // map helicentric distance to sun from spacecraft
//    v3Scale(-1.0, this->stateInMsgBuffer.r_BN_N, sc2Sun_N);
//    v3Add(sc2Sun_N, this->sunEphmInMsgBuffer.PositionVector, sc2Sun_N);
//    this->scSim->helioRadius =  v3Norm(sc2Sun_N)*m2km;   /* km */
//    
//    // map the inertial and body frame sun unit direction vectors
//    this->computeSunHeadingData();
//    
    // map the primary celestial body
    
//    if (strcmp(this->spicePlanetStates[0].PlanetName, "sun") == 0) {
//        this->scSim->celestialObject = CELESTIAL_SUN;
//        this->scSim->mu = MU_SUN;
//    } else if (strcmp(this->centralBodyInMsgBuffer.PlanetName, "mars") == 0) {
//        this->scSim->celestialObject = CELESTIAL_MARS;
//        this->scSim->mu = MU_MARS;
//    } else {
//        this->scSim->celestialObject = CELESTIAL_EARTH;
//        this->scSim->mu = MU_EARTH;
//    }
//
    // map the spacecraft state
    v3Set(this->scStateInMsgBuffer.sigma_BN[0], this->scStateInMsgBuffer.sigma_BN[1], this->scStateInMsgBuffer.sigma_BN[2], this->scSim->sigma);
    v3Set(this->scStateInMsgBuffer.r_BN_N[0]*m2km, this->scStateInMsgBuffer.r_BN_N[1]*m2km, this->scStateInMsgBuffer.r_BN_N[2]*m2km, this->scSim->r_N);
    v3Set(this->scStateInMsgBuffer.v_BN_N[0]*m2km, this->scStateInMsgBuffer.v_BN_N[1]*m2km, this->scStateInMsgBuffer.v_BN_N[2]*m2km, this->scSim->v_N);
    
//    rv2elem(this->scSim->mu, this->scSim->r_N, this->scSim->v_N, &this->scSim->oe);
//
//    for (i = 0; i < this->thrusters.size(); i++)
//    {
//        eigenVector3d2CArray(thrusters[i].thrusterLocation, this->scSim->thrusters[i].r_B);
//        eigenVector3d2CArray(thrusters[i].thrusterDirection, this->scSim->thrusters[i].gt_B);
//        scSim->thrusters[i].maxThrust = thrusters[i].maxThrust;
//        scSim->thrusters[i].level = thrusters[i].thrustFactor;
//    }
//
    for (i = 0; i < this->reactionWheels.size(); i++)
    {
//        this->scSim->reactionWheels[i].state = COMPONENT_ON;
        ReactionWheelConfigData rwData = this->reactionWheels.at(i);
//        eigenVector3d2CArray(rwData.rWB_S, this->scSim->reactionWheels[i].rWB_B);
//        eigenVector3d2CArray(rwData.gsHat_S, this->scSim->reactionWheels[i].gsHat_S);
//        eigenVector3d2CArray(rwData.ggHat0_S, this->scSim->reactionWheels[i].ggHat0_S);
//        eigenVector3d2CArray(rwData.gtHat0_S, this->scSim->reactionWheels[i].gtHat0_S);
//        this->scSim->reactionWheels[i].u_current = rwData.u_current;
//        this->scSim->reactionWheels[i].u_max = rwData.u_max;
//        this->scSim->reactionWheels[i].u_min = rwData.u_min;
//        this->scSim->reactionWheels[i].theta = rwData.theta;
        this->scSim->reactionWheels[i].Omega = rwData.Omega;
//        this->scSim->reactionWheels[i].Omega_max = 2000.0 * 2 * M_PI / 60;
    }
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
