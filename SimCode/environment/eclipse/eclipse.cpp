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
#include "eclipse.h"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include "architecture/messaging/system_messaging.h"


/*! This constructor initializes the variables that spice uses.  Most of them are
 not intended to be changed, but a couple are user configurable.
 */
Eclipse::Eclipse() :
    outputBufferCount(2)
{
    this->CallCounts = 0;
        return;
}

/*! The only needed activity in the destructor is to delete the spice I/O buffer
 that was allocated in the constructor*/
Eclipse::~Eclipse()
{
    return;
}

/*! This method initializes the object.  It creates the module's output 
 messages.
 @return void*/
void Eclipse::SelfInit()
{
    std::vector<std::string>::iterator it;
    for(it = this->positionMsgNames.begin(); it != this->positionMsgNames.end(); it++)
    {
        std::string eclipseMsgName = *it + "_eclipse_data";
        uint64_t msgID = SystemMessaging::GetInstance()->
        subscribeToMessage(eclipseMsgName, sizeof(EclipseData), this->moduleID);
        this->eclipseOutMsgId.push_back(msgID);
        this->eclipseOutMsgName.push_back(eclipseMsgName);
    }
}

/*! This method .
 @return void*/
void Eclipse::CrossInit()
{
    std::vector<std::string>::iterator it;
    for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
    {
        std::string planetMsgName = *it + "_planet_data";
        uint64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage(planetMsgName, sizeof(SpicePlanetState), this->moduleID);
        this->planetInMsgIdAndName[msgID] = planetMsgName;
    }
    
    std::vector<std::string>::iterator posIt;
    for(posIt = this->positionMsgNames.begin(); posIt != this->positionMsgNames.end(); posIt++)
    {
        uint64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage((*posIt), sizeof(SCPlusOutputStateData), this->moduleID);
        this->positionInMsgIdAndState[msgID] = SCPlusOutputStateData();
    }
}

/*! This method takes the values computed in the model and outputs them.
 It packages up the internal variables into the output structure definitions
 and puts them out on the messaging system
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void Eclipse::readInputMessages()
{
    SingleMessageHeader *tmpHeader;
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    
    //! - Iterate through all of the position Msgs
    std::map<uint64_t, SCPlusOutputStateData>::iterator it;
    for(it = this->positionInMsgIdAndState.begin(); it != this->positionInMsgIdAndState.end(); it++)
    {
        messageSys->ReadMessage(it->first, tmpHeader, sizeof(SCPlusOutputStateData), reinterpret_cast<uint8_t*>(&it->second), this->moduleID);
    }
}

/*! This method takes the values computed in the model and outputs them.
 It packages up the internal variables into the output structure definitions
 and puts them out on the messaging system
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void Eclipse::writeOutputMessages(uint64_t CurrentClock)
{
    //! - Iterate through all of the planets that are on and write their outputs
//    for(planit = PlanetData.begin(); planit != PlanetData.end(); planit++)
//    {
//        SystemMessaging::GetInstance()->WriteMessage(planit->first, CurrentClock,
//                                                     sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&planit->second), moduleID);
//    }
    
}

/*! This method is the interface point between the upper level simulation and
 the SPICE interface at runtime.  It calls all of the necessary lower level
 methods.
 @return void
 @param CurrentSimNanos The current clock time for the simulation
 */
void Eclipse::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    
    this->writeOutputMessages(CurrentSimNanos);
}

/*! This method adds spacecraft state data message names to a vector, creates
 a new unique output message name for the eclipse data message and returns
 this to the user so that they may assign the eclipse message name to other 
 modules requiring eclipse data.
 methods.
 @return std::string newEclipseMsgName The unique eclipse data msg name 
 associated with the given input state message name.
 @param std::string msgName The message name for the spacecraft state data
 for which to compute the eclipse data.
 */
std::string Eclipse::addPositionMsgName(std::string msgName)
{
    this->positionMsgNames.push_back(msgName);
    
    std::string newEclipseMsgName = "eclipse_data_" + std::to_string(this->eclipseOutMsgName.size()-1);
    this->eclipseOutMsgName.push_back(newEclipseMsgName);
    return newEclipseMsgName;
}
