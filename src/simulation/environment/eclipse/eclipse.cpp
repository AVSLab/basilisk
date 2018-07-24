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

#include "eclipse.h"
#include <iostream>
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"



Eclipse::Eclipse() :
    outputBufferCount(2)
{
    this->CallCounts = 0;
        return;
}

Eclipse::~Eclipse()
{
    return;
}

/*! This method initializes the object. It creates the module's output
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

/*! This method subscribes to the spice planet states and spaceccraft state messages.
 @return void*/
void Eclipse::CrossInit()
{
    std::vector<std::string>::iterator it;
    for(it = this->planetInMsgNames.begin(); it != this->planetInMsgNames.end(); it++)
    {
        int64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage((*it), sizeof(SpicePlanetStateSimMsg), this->moduleID);
        this->planetInMsgIdAndStates[msgID] = SpicePlanetStateSimMsg();
    }
    
    // If the user didn't b set a custom sun meesage name then use
    // the default system name of sun_planet_data
    if (this->sunInMsgName.empty()) {
        this->sunInMsgName = "sun_planet_data";
    }
    this->sunInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunInMsgName, sizeof(SpicePlanetStateSimMsg), this->moduleID);
    
    std::vector<std::string>::iterator posIt;
    for(posIt = this->positionMsgNames.begin(); posIt != this->positionMsgNames.end(); posIt++)
    {
        int64_t msgID = SystemMessaging::GetInstance()->subscribeToMessage((*posIt), sizeof(SCPlusStatesSimMsg), this->moduleID);
        this->positionInMsgIdAndState[msgID] = SCPlusStatesSimMsg();
    }
}

/*! This method reads the spacecraft state and spice planet states from the messaging system.
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void Eclipse::readInputMessages()
{
    SingleMessageHeader tmpHeader;
    SystemMessaging *messageSys = SystemMessaging::GetInstance();
    
    //! - Iterate through all of the position Msgs
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    std::map<int64_t, SCPlusStatesSimMsg>::iterator stateIt;
    
    for(stateIt = this->positionInMsgIdAndState.begin(); stateIt != this->positionInMsgIdAndState.end(); stateIt++)
    {
        messageSys->ReadMessage(stateIt->first, &tmpHeader, sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*>(&stateIt->second), this->moduleID);
    }
    
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    messageSys->ReadMessage(this->sunInMsgId, &tmpHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->sunInMsgState));
    
    //! - Iterate through all of the spice planet Msgs
    memset(&tmpHeader, 0x0, sizeof(tmpHeader));
    std::map<int64_t, SpicePlanetStateSimMsg>::iterator planetIt;
    for(planetIt = this->planetInMsgIdAndStates.begin(); planetIt != this->planetInMsgIdAndStates.end(); planetIt++)
    {
       messageSys->ReadMessage(planetIt->first, &tmpHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&planetIt->second), this->moduleID);
    }
}

/*! This method takes the computed shadow factors and outputs them to the m
 messaging system.
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void Eclipse::writeOutputMessages(uint64_t CurrentClock)
{
    //! - Iterate through all of the planets that are on and write their outputs
    std::vector<int64_t>::iterator msgIt;
    for(msgIt = this->eclipseOutMsgId.begin(); msgIt != this->eclipseOutMsgId.end(); msgIt++)
    {
        EclipseSimMsg tmpEclipseSimMsg;
        tmpEclipseSimMsg.shadowFactor = this->eclipseShadowFactors.at(msgIt - this->eclipseOutMsgId.begin());
        SystemMessaging::GetInstance()->WriteMessage((*msgIt), CurrentClock,
                                                     sizeof(EclipseSimMsg), reinterpret_cast<uint8_t*>(&tmpEclipseSimMsg), this->moduleID);
    }
}

/*! This method governs the calculation and checking for eclipse
 conditions.
 @return void
 @param CurrentSimNanos The current clock time for the simulation
 */
void Eclipse::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    
    // A lot of different vectors here. The below letters denote frames
    // P: planet frame
    // B: spacecraft body frame
    // N: inertial frame
    // H: sun (helio) frame
    Eigen::Vector3d r_PN_N(0.0, 0.0, 0.0); // r_planet
    Eigen::Vector3d r_HN_N(this->sunInMsgState.PositionVector); // r_sun
    Eigen::Vector3d r_BN_N(0.0, 0.0, 0.0); // r_sc
    Eigen::Vector3d s_BP_N(0.0, 0.0, 0.0); // s_sc wrt planet
    Eigen::Vector3d s_HP_N(0.0, 0.0, 0.0); // s_sun wrt planet
    Eigen::Vector3d r_HB_N(0.0, 0.0, 0.0); // r_sun wrt sc
    std::map<int64_t, SpicePlanetStateSimMsg>::iterator planetIt;
    std::map<int64_t, SCPlusStatesSimMsg>::iterator scIt;
    
    // Index to assign the shadowFactor for each body position (S/C)
    // being tracked
    int scIdx = 0;
    
    for(scIt = this->positionInMsgIdAndState.begin(); scIt != this->positionInMsgIdAndState.end(); scIt++)
    {
        double tmpShadowFactor = 1.0; // 1.0 means 100% illumination (no eclipse)
        int idx = 0;
        double eclipsePlanetDistance = 0.0;
        int64_t eclipsePlanetKey = -1;
        r_BN_N = Eigen::Map<Eigen::Vector3d>(&(scIt->second.r_BN_N[0]), 3, 1);
        
        // Find the closest planet if there is one
        for(planetIt = this->planetInMsgIdAndStates.begin(); planetIt != this->planetInMsgIdAndStates.end(); planetIt++)
        {
            r_PN_N = Eigen::Map<Eigen::Vector3d>(&(planetIt->second.PositionVector[0]), 3, 1);
            s_HP_N = r_HN_N - r_PN_N;
            r_HB_N = r_HN_N - r_BN_N;
            s_BP_N = r_BN_N - r_PN_N;

            // If spacecraft is closer to sun than planet
            // then eclipse not possible
            if (r_HB_N.norm() < s_HP_N.norm()) {
                break;
            }
            
            // Find the closest planet and save its distance and std::map key
            if (idx == 0) {
                eclipsePlanetDistance = s_BP_N.norm();
                eclipsePlanetKey = planetIt->first;
            } else if (s_BP_N.norm() < eclipsePlanetDistance) {
                eclipsePlanetDistance = s_BP_N.norm();
                eclipsePlanetKey = planetIt->first;
            }
            idx++;
        }
        
        // If planetkey is not -1 then we have a planet for which
        // we compute the eclipse conditions
        if (eclipsePlanetKey >= 0) {
            r_PN_N = Eigen::Map<Eigen::Vector3d>(&(this->planetInMsgIdAndStates[eclipsePlanetKey].PositionVector[0]), 3, 1);
            s_BP_N = r_BN_N - r_PN_N;
            r_HB_N = r_HN_N - r_BN_N;
            s_HP_N = r_HN_N - r_PN_N;
            
            double s = s_BP_N.norm();
            double planetRadius = this->getPlanetEquatorialRadius(this->planetInMsgIdAndStates[eclipsePlanetKey].PlanetName);
            double f_1 = asin((REQ_SUN*1000 + planetRadius)/s_HP_N.norm());
            double f_2 = asin((REQ_SUN*1000 - planetRadius)/s_HP_N.norm());
            double s_0 = (-s_BP_N.dot(s_HP_N))/s_HP_N.norm();
            double c_1 = s_0 + planetRadius/sin(f_1);
            double c_2 = s_0 - planetRadius/sin(f_2);
            double l = sqrt(s*s - s_0*s_0);
            double l_1 = c_1*tan(f_1);
            double l_2 = c_2*tan(f_2);
            
            if (fabs(l) < fabs(l_2)) {
                if (c_2 < 0) { // total eclipse
                    tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
                } else { //c_2 > 0 // annular
                    tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
                }
            } else if (fabs(l) < fabs(l_1)) { // partial
                tmpShadowFactor = this->computePercentShadow(planetRadius, r_HB_N, s_BP_N);
            }
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
    // In particular (c < a + b) must check last to avoid testing
    // with implausible a, b and c values
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
    if (planetName == "mercury") {
        this->planetNames.push_back(planetName);
        this->planetInMsgNames.push_back(planetName + "_planet_data");
    } else if (planetName == "venus") {
        this->planetNames.push_back(planetName);
        this->planetInMsgNames.push_back(planetName + "_planet_data");
    } else if (planetName == "earth") {
        this->planetNames.push_back(planetName);
        this->planetInMsgNames.push_back(planetName + "_planet_data");
    } else if (planetName == "mars barycenter") {
        this->planetNames.push_back(planetName);
        this->planetInMsgNames.push_back(planetName + "_planet_data");
    } else {
        BSK_PRINT(MSG_WARNING, "Planet name %s not found. %s will not be used to compute eclipse conditions.", planetName.c_str(), planetName.c_str());
    }
    
    return;
}

/*! This method return planet radii.
 methods.
 @return double  The equatorial radius in metres
 associated with the given planet name.
 @param std::string planetSpiceName The planet name according
 to the spice NAIF Integer ID codes.
 */
double Eclipse::getPlanetEquatorialRadius(std::string planetSpiceName)
{
    if (planetSpiceName == "mercury") {
        return REQ_MERCURY*1000.0; // [metres]
    } else if (planetSpiceName == "venus") {
        return REQ_VENUS*1000.0;
    } else if (planetSpiceName == "earth") {
        return REQ_EARTH*1000.0;
    } else if (planetSpiceName == "mars barycenter") {
        return REQ_MARS*1000.0;
    } else {
        return 0.0;
    }
}
