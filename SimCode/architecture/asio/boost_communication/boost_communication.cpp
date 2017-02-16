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
//
// boost_communication.cpp
//
//

#include "boost_communication.h"
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

OpenGLIO::OpenGLIO()
    : scSim(new SpacecraftSim())
    , ipAddress("127.0.0.1")
    , stateInMsgName("inertial_state_output")
    , stateInMsgId(-1)
    , sunEphmInMsgName("sun_planet_data")
    , sunEphmInMsgId(-1)
    , centralBodyInMsgName("central_body_data")
    , centralBodyInMsgId(-1)
    , spiceTimeDataInMsgName("spice_time_output_data")
    , spiceTimeDataInMsgId(-1)
{
    
}

OpenGLIO::~OpenGLIO()
{
    shutdown();
}

void OpenGLIO::SelfInit()
{
    
    this->initialize();
}

void OpenGLIO::CrossInit()
{
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    
    this->stateInMsgId = messageSys->subscribeToMessage(this->stateInMsgName, sizeof(SCPlusStatesSimMsg), moduleID);
    this->sunEphmInMsgId = messageSys->subscribeToMessage(this->sunEphmInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    this->centralBodyInMsgId = messageSys->subscribeToMessage(this->centralBodyInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    this->spiceTimeDataInMsgId = messageSys->subscribeToMessage(this->spiceTimeDataInMsgName, sizeof(SpiceTimeSimMsg), this->moduleID);
    
    int i;
    for (i = 0; i < this->rwInMsgNames.size(); i++)
    {
        this->rwInMsgIds.push_back(messageSys->subscribeToMessage(this->rwInMsgNames.at(i), sizeof(RWConfigSimMsg), moduleID));
    }
    this->reactionWheels.resize(i);
    
    for (i = 0; i < this->thrusterInMsgNames.size(); i++)
    {
        this->thrusterInMsgIds.push_back(messageSys->subscribeToMessage(this->thrusterInMsgNames.at(i), sizeof(THROutputMessage), moduleID));
    }
    this->thrusters.resize(i);
    
    for(i = 0; i < this->planetInMsgNames.size(); i++)
    {
        this->planetInMsgIds.push_back(messageSys->subscribeToMessage(this->planetInMsgNames.at(i), sizeof(SpicePlanetStateSimMsg), moduleID));
    }
    this->planets.resize(i);
    
    // Set scSim object initial ephemeris time
    if(this->loadSpiceKernel((char *)"naif0011.tls", this->spiceDataPath.c_str())) {
        printf("Unable to load %s", "naif0010.tls");
    }
    str2et_c(this->UTCCalInit.c_str(), &this->scSim->ics.ET0);
}

int OpenGLIO::loadSpiceKernel(char *kernelName, const char *dataPath)
{
    uint32_t CharBufferSize = 512;

    char *fileName = new char[CharBufferSize];
    SpiceChar *name = new SpiceChar[CharBufferSize];
    
    //! Begin method steps
    //! - The required calls come from the SPICE documentation.
    //! - The most critical call is furnsh_c
    strcpy(name, "REPORT");
    erract_c("SET", CharBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    furnsh_c(fileName);
    
    //! - Check to see if we had trouble loading a kernel and alert user if so
    strcpy(name, "DEFAULT");
    erract_c("SET", CharBufferSize, name);
    delete[] fileName;
    delete[] name;
    if(failed_c()) {
        return 1;
    }
    return 0;
}

/*! This method reads in all of the messages and saves off the data to send 
 to the visualization.
 @return void
 */
void OpenGLIO::readInputMessages()
{
    SingleMessageHeader localHeader;
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    
    messageSys->ReadMessage(this->stateInMsgId, &localHeader, sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*> (&this->stateInMsgBuffer));
    messageSys->ReadMessage(this->sunEphmInMsgId, &localHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&this->sunEphmInMsgBuffer));
    messageSys->ReadMessage(this->centralBodyInMsgId, &localHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&this->centralBodyInMsgBuffer));
    messageSys->ReadMessage(this->spiceTimeDataInMsgId, &localHeader, sizeof(SpiceTimeSimMsg), reinterpret_cast<uint8_t*> (&this->spiceTimeDataInMsgBuffer));
    
    RWConfigSimMsg tmpWheelData;
    for (int i = 0; i < this->reactionWheels.size(); i++)
    {
        messageSys->ReadMessage(this->rwInMsgIds.at(i), &localHeader, sizeof(RWConfigSimMsg), reinterpret_cast<uint8_t*> (&tmpWheelData));
        this->reactionWheels.at(i) = tmpWheelData;
    }
    
    THROutputMessage tmpThrusterData;
    for (int i = 0; i < this->thrusters.size(); i++)
    {
        messageSys->ReadMessage(this->thrusterInMsgIds.at(i), &localHeader, sizeof(THROutputMessage), reinterpret_cast<uint8_t*> (&tmpThrusterData));
        this->thrusters.at(i) = tmpThrusterData;
    }

    SpicePlanetStateSimMsg tmpPlanetData;
    for (int i = 0; i < this->planets.size(); i++)
    {
        messageSys->ReadMessage(this->planetInMsgIds.at(i), &localHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&tmpPlanetData));
        this->planets.at(i) = tmpPlanetData;
    }
}

void OpenGLIO::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->mapMessagesToScSim(CurrentSimNanos);
    this->send();
}

bool OpenGLIO::initialize()
{
    std::stringstream ss;
    ss << IP_BASE_PORT;
    if(!this->server.acceptConnections(this->ipAddress, ss.str())) {
        return false;
    }
    return true;
}

bool OpenGLIO::shutdown()
{
    if(!this->server.close()) {
        return false;
    }
    return true;
}

bool OpenGLIO::send()
{
    this->server.setOutboundData(this->scSim);
    return true;
}

bool OpenGLIO::receive(SpacecraftSim *scSim, std::string tmpDataVariable1)
{
//    SpacecraftSimVisualization visualizationData = m_server.getInboundData();
//    SpacecraftSimVisualization temp;
//    if(visualizationData != temp) {
//        if(visualizationData.adcsState != temp.adcsState) {
//            sc->stateRequest = visualizationData.adcsState;
//        }
//        if(visualizationData.perturbRates) {
//            for(size_t i = 0; i < 3; i++) {
//                scSim->omega[i] = scSim->perturb.initialOmegaDisp * (2.0 * prn_uniform() - 1.0);
//            }
//        }
//        if(visualizationData.controlState != temp.controlState) {
//            scSim->ctrlState = visualizationData.controlState;
//        }
//        if(visualizationData.realTimeSpeedUpFactor != temp.realTimeSpeedUpFactor)
//        {
//            scSim->realTimeSpeedUpFactor = visualizationData.realTimeSpeedUpFactor;
//        }
//    }
    return true;
}

void OpenGLIO::setIpAddress(std::string ipAddress)
{
    // Maybe check for an excisting connection before allowing ipAddress to be set
    // or just ensure a graceful transition between connections.
    this->ipAddress = ipAddress;
}

void OpenGLIO::setUTCCalInit(std::string UTCCalInit)
{
    this->UTCCalInit = UTCCalInit;
}

void OpenGLIO::setCelestialObject(int celestialObject)
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

void OpenGLIO::addRwMessageName(int rwIdx)
{
    this->rwInMsgNames.push_back("rw_" + std::to_string(rwIdx) + "_data");
}

void OpenGLIO::addThrusterMessageName(int thrusterIdx)
{
    this->thrusterInMsgNames.push_back("acs_thruster_" + std::to_string(thrusterIdx) + "_data");
}

void OpenGLIO::addPlanetMessageName(std::string planetName)
{
    this->planetInMsgNames.push_back(planetName + "_planet_data");
}

void OpenGLIO::computeSunHeadingData()
{
    double Sc2Sun_Inrtl[3];
    double dcm_BN[3][3];
    
    v3Scale(-1.0, this->stateInMsgBuffer.r_BN_N, Sc2Sun_Inrtl);
    v3Add(Sc2Sun_Inrtl, this->sunEphmInMsgBuffer.PositionVector, Sc2Sun_Inrtl);
    v3Normalize(Sc2Sun_Inrtl, this->scSim->sHatN);
    MRP2C(this->stateInMsgBuffer.sigma_BN, dcm_BN);
    m33MultV3(dcm_BN, this->scSim->sHatN, this->scSim->sHatB);
}

void OpenGLIO::mapMessagesToScSim(uint64_t currentSimNanos) 
{
    double m2km = 0.001;
    double sc2Sun_N[3]; // position vector of sun relative to the spacecraft
    int i = 0;
    
    // map sim time
    this->scSim->time = currentSimNanos*1.0E-9;
    // map helicentric distance to sun from spacecraft
    v3Scale(-1.0, this->stateInMsgBuffer.r_BN_N, sc2Sun_N);
    v3Add(sc2Sun_N, this->sunEphmInMsgBuffer.PositionVector, sc2Sun_N);
    this->scSim->helioRadius =  v3Norm(sc2Sun_N)*m2km;   /* km */
  
    // map the inertial and body frame sun unit direction vectors
    this->computeSunHeadingData();
    
    // map the primary celestial body
    if (strcmp(this->centralBodyInMsgBuffer.PlanetName, "sun") == 0) {
        this->scSim->celestialObject = CELESTIAL_SUN;
        this->scSim->mu = MU_SUN;
    } else if (strcmp(this->centralBodyInMsgBuffer.PlanetName, "mars") == 0) {
        this->scSim->celestialObject = CELESTIAL_MARS;
        this->scSim->mu = MU_MARS;
    } else {
        this->scSim->celestialObject = CELESTIAL_EARTH;
        this->scSim->mu = MU_EARTH;
    }
    
    // map the spacecraft state
    v3Set(this->stateInMsgBuffer.sigma_BN[0], this->stateInMsgBuffer.sigma_BN[1], this->stateInMsgBuffer.sigma_BN[2], this->scSim->sigma);
    v3Set(this->stateInMsgBuffer.r_BN_N[0]*m2km, this->stateInMsgBuffer.r_BN_N[1]*m2km, this->stateInMsgBuffer.r_BN_N[2]*m2km, this->scSim->r_N);
    v3Set(this->stateInMsgBuffer.v_BN_N[0]*m2km, this->stateInMsgBuffer.v_BN_N[1]*m2km, this->stateInMsgBuffer.v_BN_N[2]*m2km, this->scSim->v_N);
    
    rv2elem(this->scSim->mu, this->scSim->r_N, this->scSim->v_N, &this->scSim->oe);
    
    for (i = 0; i < this->thrusters.size(); i++)
    {
        eigenVector3d2CArray(thrusters[i].thrusterLocation, this->scSim->thrusters[i].r_B);
        eigenVector3d2CArray(thrusters[i].thrusterDirection, this->scSim->thrusters[i].gt_B);
        scSim->thrusters[i].maxThrust = thrusters[i].maxThrust;
        scSim->thrusters[i].level = thrusters[i].thrustFactor;
    }
    
    for (i = 0; i < this->reactionWheels.size(); i++)
    {
        this->scSim->rw[i].state = COMPONENT_ON;
        eigenVector3d2CArray(reactionWheels[i].rWB_S, this->scSim->rw[i].r_B);
        eigenVector3d2CArray(reactionWheels[i].gsHat_S, this->scSim->rw[i].gs);
        eigenVector3d2CArray(reactionWheels[i].w3Hat0_S, this->scSim->rw[i].gg0);
        eigenVector3d2CArray(reactionWheels[i].w2Hat0_S, this->scSim->rw[i].gt0);
        this->scSim->rw[i].u = reactionWheels[i].u_current;
        this->scSim->rw[i].maxTorque = reactionWheels[i].u_max;
        this->scSim->rw[i].minTorque = reactionWheels[i].u_min;
        this->scSim->rw[i].wheelAngle =reactionWheels[i].theta;
        this->scSim->rw[i].Omega = reactionWheels[i].Omega;
        this->scSim->rw[i].maxTemp = 2000;
        this->scSim->rw[i].highSpeed = 2000.0 * 2 * M_PI / 60;
        this->scSim->rw[i].motorTemp1 = 0.0;
        this->scSim->rw[i].motorTemp2 = 0.0;
    }
}

BoostCommunication::BoostCommunication()
{

}

BoostCommunication::~BoostCommunication()
{

}

bool BoostCommunication::initializeAll(std::string tmpDataVariable)
{
//    if(scSim->useOpenGLVisualization && scSim->codeBehavior != CODE_FSW) {
//        if(!m_openGLIO.initialize(scSim)) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    switch(scSim->codeBehavior) {
//        case CODE_REALITY:
//        case CODE_REALITY_ON_DEMAND:
//            printf("codeBehavior: REALITY");
//            if(scSim->codeBehavior == CODE_REALITY_ON_DEMAND) {
//                printf("_ON_DEMAND");
//            }
//            printf("\nWaiting to connect to FSW or Flatsat Module.\n");
//            if(!m_realityCommIO.initialize(scSim)) {
//                std::cout << "called by " << __FUNCTION__ << std::endl;
//                return false;
//            }
//            break;
//
//        case CODE_FSW:
//            printf("codeBehavior: FSW\n");
//            if(!m_fswCommIO.initialize(scSim)) {
//                std::cout << "called by " << __FUNCTION__ << std::endl;
//                return false;
//            }
//            break;
//
//        case CODE_STANDALONE:
//            printf("codeBehavior: STAND_ALONE\n");
//            break;
//
//        default:
//            break;
//    }
    return true;
}

bool BoostCommunication::shutdownAll(std::string tmpDataVariable)
{
//    if(scSim->codeBehavior == CODE_REALITY || scSim->codeBehavior == CODE_REALITY_ON_DEMAND) {
//        if(!m_realityCommIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    if(scSim->codeBehavior == CODE_FSW) {
//        if(!m_fswCommIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
//    if(scSim->useOpenGLVisualization && scSim->codeBehavior != CODE_FSW) {
//        if(!m_openGLIO.shutdown()) {
//            std::cout << "called by " << __FUNCTION__ << std::endl;
//            return false;
//        }
//    }
    return true;
}
