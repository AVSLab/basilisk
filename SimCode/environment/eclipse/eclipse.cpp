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
#include "utilities/astroConstants.h"


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
    for(it = this->eclipseOutMsgNames.begin(); it != this->eclipseOutMsgNames.end(); it++)
    {
        uint64_t msgId =  SystemMessaging::GetInstance()->CreateNewMessage((*it), sizeof(EclipseSimMsg), 2, "EclipseSimMsg", this->moduleID);
        this->eclipseOutMsgId.push_back(msgId);
    }

    // Now that we know the number of output messages we can size and zero
    // the eclipse data vector
    this->eclipseShadowFactors.resize(this->eclipseOutMsgId.size());
}

/*! This method .
 @return void*/
void Eclipse::CrossInit()
{
    std::vector<std::string>::iterator it;
    for(it = this->planetInMsgNames.begin(); it != this->planetInMsgNames.end(); it++)
    {
        uint64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage((*it), sizeof(SpicePlanetStateSimMsg), this->moduleID);
        this->planetInMsgIdAndStates[msgID] = SpicePlanetStateSimMsg();
    }
    
    // If the user didn't set a custom sun meesage name then use
    // the default system name of sun_planet_data
    if (this->sunInMsgName.empty()) {
        this->sunInMsgName = "sun_planet_data";
    }
    this->sunInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunInMsgName, sizeof(SpicePlanetStateSimMsg), this->moduleID);
    
    std::vector<std::string>::iterator posIt;
    for(posIt = this->positionMsgNames.begin(); posIt != this->positionMsgNames.end(); posIt++)
    {
        uint64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage((*posIt), sizeof(SCPlusStatesSimMsg), this->moduleID);
        this->positionInMsgIdAndState[msgID] = SCPlusStatesSimMsg();
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
    SingleMessageHeader tmpHeader;
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    
    //! - Iterate through all of the position Msgs
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    std::map<uint64_t, SCPlusStatesSimMsg>::iterator stateIt;
    for(stateIt = this->positionInMsgIdAndState.begin(); stateIt != this->positionInMsgIdAndState.end(); stateIt++)
    {
        messageSys->ReadMessage(stateIt->first, &tmpHeader, sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&stateIt->second), this->moduleID);
    }
    
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    messageSys->ReadMessage(this->sunInMsgId, &tmpHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->sunInMsgState));
    
    //! - Iterate through all of the spice planet Msgs
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    std::map<uint64_t, SpicePlanetStateSimMsg>::iterator planetIt;
    for(planetIt = this->planetInMsgIdAndStates.begin(); planetIt != this->planetInMsgIdAndStates.end(); planetIt++)
    {
        messageSys->ReadMessage(planetIt->first, &tmpHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&planetIt->second), this->moduleID);
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
    std::vector<uint64_t>::iterator msgIt;
    for(msgIt = this->eclipseOutMsgId.begin(); msgIt != this->eclipseOutMsgId.end(); msgIt++)
    {
        EclipseSimMsg tmpEclipseSimMsg;
        tmpEclipseSimMsg.shadowFactor = this->eclipseShadowFactors.at(msgIt - this->eclipseOutMsgId.begin());
        SystemMessaging::GetInstance()->WriteMessage((*msgIt), CurrentClock,
                                                     sizeof(EclipseSimMsg), reinterpret_cast<uint8_t*>(&tmpEclipseSimMsg), this->moduleID);
    }
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
    
    // A lot of different vectors here. The below letters denote frames
    // P: planet frame
    // S: spacecraft frame
    // N: inertial frame
    // H: sun (helio) frame
    Eigen::Vector3d r_PN_N(0.0, 0.0, 0.0); // r_planet
    Eigen::Vector3d r_HN_N(this->sunInMsgState.PositionVector); // r_sun
    Eigen::Vector3d r_BN_N(0.0, 0.0, 0.0); // r_sc
    Eigen::Vector3d s_BP_N(0.0, 0.0, 0.0); //s_sc/B
    Eigen::Vector3d s_HP_N(0.0, 0.0, 0.0); //s_sun
    Eigen::Vector3d r_BH_N(0.0, 0.0, 0.0); //s_sc/sun

    std::map<uint64_t, SpicePlanetStateSimMsg>::iterator planetIt;
    std::map<uint64_t, SCPlusStatesSimMsg>::iterator scIt;
    // Index to move through the ordered planet radii vector
    int radiusIdx = -1;
    // Index to assign the shadowFactor for each body position (S/C)
    // being tracked
    int scIdx = 0;
    
    for(scIt = this->positionInMsgIdAndState.begin(); scIt != this->positionInMsgIdAndState.end(); scIt++)
    {
        double tmpShadowFactor = 1.0; // 1.0 means 100% illumination (no eclipse)
        r_BN_N = Eigen::Map<Eigen::Vector3d>(&(scIt->second.r_BN_N[0]), 3, 1);
        
        // Set min planet distance using first planet
        r_PN_N = Eigen::Map<Eigen::Vector3d>(&(this->planetInMsgIdAndStates.begin()->second.PositionVector[0]), 3, 1);
        s_BP_N = r_BN_N - r_PN_N;
        double minPlanetDistance = fabs(s_BP_N.norm());
        
        // Find the closest planet if there is one
        for(planetIt = this->planetInMsgIdAndStates.begin(); planetIt != this->planetInMsgIdAndStates.end(); planetIt++)
        {
            r_PN_N = Eigen::Map<Eigen::Vector3d>(&(planetIt->second.PositionVector[0]), 3, 1);
            s_HP_N = r_HN_N - r_PN_N;
            r_BH_N = r_BN_N - r_HN_N;
            s_BP_N = r_BN_N - r_PN_N;

            // Spacecraft is closer to sun than planet eclipse not possible
            if (r_BH_N.norm() < s_HP_N.norm()) {
                break;
            }
            
            if (fabs(s_BP_N.norm()) < minPlanetDistance) {
                minPlanetDistance = fabs(s_BP_N.norm());
            }
            // Keep track of the index for the planetary radius
            // of the closest planet
            radiusIdx++;
        }
        
        // If radius index is not -1 then we have a planet for which
        // we compute the eclipse conditions
        if (radiusIdx >= 0) {
                Eigen::Vector3d r_HB_N = r_HN_N - r_BN_N;
                tmpShadowFactor = this->computePercentShadow(this->planetRadii[radiusIdx], r_HB_N, s_BP_N);
        }
        
        this->eclipseShadowFactors.at(scIdx) = tmpShadowFactor;
        scIdx++;
    }
    this->writeOutputMessages(CurrentSimNanos);
}

/*! This method computes the fraction of sunlight given an eclipse.
 @return double fractionShadow The eclipse shadow fraction.
 @param std::string msgName .
 */
double Eclipse::computePercentShadow(double planetRadius, Eigen::Vector3d r_HB_N, Eigen::Vector3d s_BP_N)
{
    double area = 0.0;
    double shadowFraction = 1.0; // Initialise to value for no eclipse
    double normR_HB_N = r_HB_N.norm();
    double normS_BP_N = s_BP_N.norm();
    double a = asin(REQ_SUN*1000/normR_HB_N); // apparent radius of sun
    double b = asin(planetRadius/normS_BP_N); // apparent radius of occulting body
    double c = acos((-s_BP_N.dot(r_HB_N))/(normS_BP_N*normR_HB_N));
    
    // The order of these conditionals is important.
    // Inparticular (c < a + b) must chek last to avoid testing
    // with bad a, b and c values
    if (c < b - a) { // total eclipse, implying a < b
        shadowFraction = 0.0;
    } else if (c < a - b) { // partial maximum eclipse, implying a > b
        double areaSun = M_PI*a*a;
        double areaBody = M_PI*b*b;
        area = areaSun - areaBody;
        shadowFraction = 1 - area/(M_PI*a*a);
    } else if (c < a + b) { // partial eclipse
        double x = (c*c + a*a - b*b)/(2*c);
        double y = sqrt(a*a - x*x);
        area = a*a*acos(x/a) + b*b*acos((c-x)/b) - c*y;
        shadowFraction = 1 - area/(M_PI*a*a);
    }
    return shadowFraction;
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
    
    std::string newEclipseMsgName = "eclipse_data_" + std::to_string(this->positionMsgNames.size()-1);
    this->eclipseOutMsgNames.push_back(newEclipseMsgName);
    return newEclipseMsgName;
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
void Eclipse::addPlanetName(std::string planetName)
{
//    std::string planetMsgName = planetName + "_planet_data";
    this->planetInMsgNames.push_back(planetName + "_planet_data");
    if (planetName == "mercury") {
        this->planetRadii.push_back(REQ_MERCURY*1000); // [metres]
    } else if (planetName == "venus") {
        this->planetRadii.push_back(REQ_VENUS*1000);
    } else if (planetName == "earth") {
        this->planetRadii.push_back(REQ_EARTH*1000);
    } else if (planetName == "mars") {
        this->planetRadii.push_back(REQ_MARS*1000);
    }
    this->planetNames.push_back(planetName);
    
    return;
}
