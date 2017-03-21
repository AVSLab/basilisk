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
#include "architecture/messaging/system_messaging.h"
#include "environment/spice/spice_interface.h"
#include "dynamics/spacecraftPlus/spacecraftPlus.h"
#include "dynamics/reactionWheels/reactionWheelStateEffector.h"
#include "dynamics/Thrusters/thrusterDynamicEffector.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "dynamics/_GeneralModuleFiles/gravityEffector.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/rwConfigSimMsg.h"
#include "simMessages/thrOutputSimMsg.h"
#include "simMessages/spiceTimeSimMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "SpacecraftSimDefinitions.h"


typedef struct {
    uint64_t msgId;
    std::string msgName;
    std::string msgStructName;
    uint32_t currentReadBuffer; // -- Index for the current read buffer
    uint64_t maxMsgSize;    // -- Maximum allowable message size
}VisMessageData;

class VisMessageInterface : public SysModel{

public:
    SpacecraftSim* scSim;
    std::string spiceDataPath;           //!< -- Path on file to SPICE data
    OpenGLIO* openglIo;
    
private:
    std::unordered_map<std::string, VisMessageData> msgMap;  // Map of <message string, message Id>
    std::unordered_map<std::string, VisMessageData> acsThrusterMsgMap;
    std::unordered_map<std::string, VisMessageData> dvThrusterMsgMap;
    // Message buffers
    std::string UTCCalInit;
    SCPlusStatesSimMsg scStateInMsg;
    SpicePlanetStateSimMsg sunEphmInMsg;
    std::vector<SpicePlanetStateSimMsg> planets;
    int64_t centralBodyInMsgId;
    SpicePlanetStateSimMsg centralBodyInMsg;
    
    std::vector<RWConfigSimMsg> rwConfigMsgs;
    RWConfigSimMsg rwSpeeds;
    std::vector<THROutputSimMsg> acsThrusters;
    std::vector<THROutputSimMsg> dvThrusters;
//  std::unordered_map<std::string, VisMessageData> rwMsgMap;
//    std::vector<std::pair<std::string, VisMessageData>> rwMsgPairs;
    std::vector<std::string> thrusterInMsgNames;
    int64_t spiceTimeDataInMsgId;
    SpiceTimeSimMsg spiceTimeDataInMsg;
    std::unordered_map<std::string, SpicePlanetStateSimMsg> spicePlanetStates;
    
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
    void setScSimCelestialObject();
    void setScSimOrbitalElements();
    void setScSimJulianDate(uint64_t currentSimNanos);
    
//    int loadSpiceKernel(char *kernelName, const char *dataPath);
};
#endif /* visMessageInterface_h */
