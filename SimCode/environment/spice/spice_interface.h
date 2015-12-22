
#ifndef SpiceInterface_H
#define SpiceInterface_H

#include <vector>
#include "utilities/sys_model.h"
#include "environment/spice/spice_planet_state.h"
#include <map>

/*! \addtogroup SimModelGroup
 *  This group is used to model parts of the vehicle and the surrounding environment
 *  in the simulation system.  All components/dynamics/environment models are a
 *  part of this group.
 * @{
 */

//! The SPICE time output structure outputs time information to the rest of the system
typedef struct {
    double J2000Current;        //!< s Current J2000 elapsed time
    double JulianDateCurrent;   //!< s Current JulianDate
    double GPSSeconds;          //!< s Current GPS seconds
    uint16_t GPSWeek;           //!< -- Current GPS week value
    uint64_t GPSRollovers;      //!< -- Count on the number of GPS rollovers
}SpiceTimeOutput;

//! The SPICE interface class gets time and planetary body information from the JPL ephemeris library
class SpiceInterface: public SysModel {
public:
    SpiceInterface();
    ~SpiceInterface();
    
    void UpdateState(uint64_t CurrentSimNanos);
    int loadSpiceKernel(char *kernelName, const char *dataPath);
	std::string getCurrentTimeString();
    void SelfInit();
    void InitTimeData();
    void ComputeGPSData();
    void ComputePlanetData();
    void SendOutputData(uint64_t CurrentClock);
    
public:
    std::string SPICEDataPath;           //!< -- Path on file to SPICE data
    std::string referenceBase;           //!< -- Base reference frame to use
    std::string zeroBase;                //!< -- Base zero point to use for states
	std::string timeOutPicture;          //!< -- Optional parameter used to extract time strings
    bool SPICELoaded;                    //!< -- Boolean indicating to reload spice
    uint32_t CharBufferSize;    //!< -- avert your eyes we're getting SPICE
    uint8_t *SpiceBuffer;       //!< -- General buffer to pass down to spice
    std::string UTCCalInit;     //!< -- UTC time string for init time
    std::string OutputTimePort; //!< -- Output time sampling port name to use
    uint64_t OutputBufferCount; //!< -- Number of output buffers to use
    std::vector<std::string>PlanetNames;  //!< -- Names of planets we want to track
    
    bool TimeDataInit;          //!< -- Flag indicating whether time has been init
    double J2000ETInit;         //!< s Seconds elapsed since J2000 at init
    double J2000Current;        //!< s Current J2000 elapsed time
    double JulianDateCurrent;   //!< s Current JulianDate
    double GPSSeconds;          //!< s Current GPS seconds
    uint16_t GPSWeek;           //!< -- Current GPS week value
    uint64_t GPSRollovers;      //!< -- Count on the number of GPS rollovers
    
private:
    std::string GPSEpochTime;   //!< -- String for the GPS epoch
    double JDGPSEpoch;          //!< s Epoch for GPS time.  Saved for efficiency
    int64_t TimeOutMsgID;       //!< -- Output time message ID
    std::map<uint32_t, SpicePlanetState> PlanetData; //!< -- Internal vector of planets
    
};

/*! @} */

#endif
