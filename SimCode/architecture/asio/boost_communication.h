//
// boost_communication.h
//
// University of Colorado, Autonomous Vehicle Systems (AVS) Lab
// Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved
//

#ifndef _BOOST_COMMUNICATION_H_
#define _BOOST_COMMUNICATION_H_

#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread.hpp>
#include "architecture/messaging/system_messaging.h"
#include "utilities/sys_model.h"
#include "dynamics/SixDofEOM/six_dof_eom.h"
#include "architecture/messaging/system_messaging.h"
#include "environment/spice/spice_planet_state.h"
#include "environment/spice/spice_interface.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "dynamics/Thrusters/thruster_dynamics.h"
#include "dynamics/ReactionWheels/reactionwheel_dynamics.h"
#include "TcpServer.h"
#include "TcpSerializeServer.h"
#include "SpacecraftSimDefinitions.h"

#define IP_BASE_PORT         50000
#define MAX_CONNECT_ATTEMPTS 5
#define SERIAL_BAUD_RATE     115200

typedef enum ComStatus_t {
    COM_INIT,
    COM_SHUTDOWN,
    COM_SEND,
    COM_RECEIVE,
    COM_ON_DEMAND_INIT,
    COM_ON_DEMAND_POLL,
    MAX_COM_STATUS
} ComStatus_t;


class OpenGLIO : public SysModel
{
public:
    OpenGLIO();
    ~OpenGLIO();

    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();

    bool initialize();
    bool shutdown();
    bool send();
    bool receive(SpacecraftSim *scSim, std::string tmpDataVariable1);
    void setIpAddress(std::string ipAddress);
    void setUTCCalInit(std::string UTCCalInit);
    void setCelestialObject(int celestialObject);
    void addPlanetMessageName(std::string planetName);
    void addThrusterMessageName(int thrusterIdx);
    void addRwMessageName(int rwIdx);
    void computeSunHeadingData();
    
    SpacecraftSim *scSim;
    std::string spiceDataPath;           //!< -- Path on file to SPICE data
    int numACSThrusters;
    int numDVThrusters;

private:
    void mapMessagesToScSim(uint64_t currentSimNanos);
    int loadSpiceKernel(char *kernelName, const char *dataPath);

    TcpSerializeServer<SpacecraftSim, int> server;
    std::string ipAddress;
    // Message names, ids and buffers
    std::string UTCCalInit;
    std::string stateInMsgName;
    uint64_t stateInMsgId;
    OutputStateData stateInMsgBuffer;
    std::string sunEphmInMsgName;
    uint64_t sunEphmInMsgId;
    SpicePlanetState sunEphmInMsgBuffer;
    
    std::vector<std::string> planetInMsgNames;
    std::vector<uint64_t> planetInMsgIds;
    std::vector<SpicePlanetState> planets;
    
    std::string centralBodyInMsgName;
    uint64_t centralBodyInMsgId;
    SpicePlanetState centralBodyInMsgBuffer;
    std::string spiceTimeDataInMsgName;
    uint64_t spiceTimeDataInMsgId;
    SpiceTimeOutput spiceTimeDataInMsgBuffer;
    std::vector<GravityBodyData> m_gravityBodies;
    
    std::vector<std::string> rwInMsgNames;
    std::vector<uint64_t> rwInMsgIds;
    std::vector<ReactionWheelConfigData> reactionWheels;
    
    std::vector<std::string> thrusterInMsgNames;
    std::vector<uint64_t> thrusterInMsgIds;
    std::vector<ThrusterOutputData> thrusters;
};


class BoostCommunication
{
public:
    BoostCommunication();
    ~BoostCommunication();

    bool initializeAll(std::string tmpDataVariable);
    bool shutdownAll(std::string tmpDataVariable);

    OpenGLIO *getOpenGLIO() {
        return &this->openGLIO;
    }
   
private:
    OpenGLIO openGLIO;
    
};

#endif
