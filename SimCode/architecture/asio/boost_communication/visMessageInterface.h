//
//  visMessageInterface.h
//  AVS basilisk
//
//  Created by Patrick Kenneally on 2/1/17.
//
//

#ifndef visMessageInterface_h
#define visMessageInterface_h

#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <boost/ptr_container/ptr_vector.hpp>
#include "boost_communication.h"
#include "dynamics/spacecraftPlus/spacecraftPlus.h"
#include "dynamics/reactionWheels/reactionWheelStateEffector.h"
#include "dynamics/Thrusters/thrusterDynamicEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "dynamics/_GeneralModuleFiles/gravityEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "environment/spice/spice_planet_state.h"
#include "environment/spice/spice_interface.h"
#include "../ADCSAlgorithms/effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "SpacecraftSimDefinitions.h"


typedef struct {
    uint64_t msgId;
    std::string msgName;// -- It pains me, but let's fix name
    std::string msgStructName; // -- Again, pain, but it's better
    uint32_t currentReadBuffer;  // -- Index for the current read buffer
    uint64_t maxMsgSize;     // -- Maximum allowable message size
}VisMessageData;

class VisMessageInterface : public SysModel{

public:
    SpacecraftSim* scSim;
    std::string spiceDataPath;           //!< -- Path on file to SPICE data
    OpenGLIO* openglIo;
    
private:
    std::unordered_map<std::string, VisMessageData> msgMap;  // Map of <message string, message Id>
    
//    TcpSerializeServer<SpacecraftSim, SpacecraftSim> server;
    std::string ipAddress;
    // Message buffers
    std::string UTCCalInit;
    SCPlusOutputStateData scStateInMsgBuffer;
    SpicePlanetState sunEphmInMsgBuffer;
    std::vector<SpicePlanetState> planets;
    SpicePlanetState centralBodyInMsgBuffer;
    SpiceTimeOutput spiceTimeDataInMsgBuffer;
    std::unordered_map<std::string, SpicePlanetState> spicePlanetStates;
    std::vector<ReactionWheelConfigData> reactionWheels;
    RWSpeedData rwSpeeds;
    std::vector<ThrusterOutputData> thrusters;
    std::vector<std::string> rwInMsgNames;
    std::unordered_map<std::string, VisMessageData> rwMsgMap;
    std::unordered_map<std::string, VisMessageData> thursterMsgMap;
    std::vector<std::string> thrusterInMsgNames;
    
public:
    VisMessageInterface();
    ~VisMessageInterface();
    void setMessageProcesses();
    void subscribeToMessages();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void setUTCCalInit(std::string UTCCalInit);
    void setCelestialObject(int celestialObject);
    void addThrusterMessageName(std::string msgName);
    void addRwMessageName(std::string msgName);
    void computeSunHeadingData();
    
private:
    void mapMessagesToScSim(uint64_t currentSimNanos);

    
//    int loadSpiceKernel(char *kernelName, const char *dataPath);
};
#endif /* visMessageInterface_h */
