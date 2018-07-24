/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef visMessageInterface_h
#define visMessageInterface_h

#include <stdio.h>
#include <iostream>
#include <unordered_map>
#include <boost/ptr_container/ptr_vector.hpp>
#include "boost_communication.h"
#include "architecture/messaging/system_messaging.h"
#include "_GeneralModuleFiles/sys_model.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/rwConfigSimMsg.h"
#include "simMessages/thrOutputSimMsg.h"
#include "simMessages/spiceTimeSimMsg.h"
#include "simMessages/realTimeFactorSimMsg.h"
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
//    SpicePlanetStateSimMsg sunEphmInMsg;
    std::vector<SpicePlanetStateSimMsg> planets;
    int64_t centralBodyInMsgId;
    SpicePlanetStateSimMsg centralBodyInMsg;
    
    std::vector<RWConfigSimMsg> rwConfigMsgs;
    RWConfigSimMsg rwSpeeds;
    std::vector<THROutputSimMsg> acsThrusters;
    std::vector<THROutputSimMsg> dvThrusters;
    std::vector<std::string> thrusterInMsgNames;
    int64_t spiceTimeDataInMsgId;
    SpiceTimeSimMsg spiceTimeDataInMsg;
    std::unordered_map<std::string, SpicePlanetStateSimMsg> spicePlanetStates;
    int64_t realTimeOutMsgId;
    std::string realTimeOutMsgName;
    RealTimeFactorSimMsg realTimeFactor;
    
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
    
private:
    void mapMessagesToScSim(uint64_t currentSimNanos);
    void setScSimCelestialObject();
    void setScSimOrbitalElements();
    void setScSimJulianDate(uint64_t currentSimNanos);
    long generateTimeStamp();
    void writeOutputMessages(uint64_t currentSimNanos);
};
#endif /* visMessageInterface_h */
