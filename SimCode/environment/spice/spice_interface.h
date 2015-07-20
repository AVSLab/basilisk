
#ifndef SpiceInterface_H
#define SpiceInterface_H

#include <vector>
#include "utilities/sys_model.h"
#include <map>

#define MAX_BODY_NAME_LENGTH 64

typedef struct {
   double J2000Current;        /// s Time of validity for the planet state
   double PositionVector[3];   /// m True position of the planet for the time
   double VelocityVector[3];   /// m/s True velocity of the planet for the time
   char PlanetName[MAX_BODY_NAME_LENGTH];        /// -- Name of the planet for the state
}SpicePlanetState;

typedef struct {
   double J2000Current;        /// s Current J2000 elapsed time
   double JulianDateCurrent;   /// s Current JulianDate
   double GPSSeconds;          /// s Current GPS seconds 
   uint16_t GPSWeek;           /// -- Current GPS week value
   uint64_t GPSRollovers;      /// -- Count on the number of GPS rollovers
}SpiceTimeOutput;

class SpiceInterface: public SysModel {
public:
   SpiceInterface();
   ~SpiceInterface();

   void UpdateState(uint64_t CurrentSimNanos);
   int loadSpiceKernel(char *kernelName, const char *dataPath);  
   void SelfInit();
   void InitTimeData(); 
   void ComputeGPSData();
   void ComputePlanetData();
   void SendOutputData(uint64_t CurrentClock);
 
public:
   std::string SPICEDataPath;           /// -- Path on file to SPICE data
   bool SPICELoaded;                    /// -- Boolean indicating to reload spice
   uint32_t CharBufferSize;    /// -- avert your eyes we're getting SPICE
   uint8_t *SpiceBuffer;       /// -- General buffer to pass down to spice
   std::string UTCCalInit;     /// -- UTC time string for init time
   std::string OutputTimePort; /// -- Output time sampling port name to use
   uint64_t OutputBufferCount; /// -- Number of output buffers to use
   std::vector<std::string>PlanetNames;  /// -- Names of planets we want to track

   bool TimeDataInit;          /// -- Flag indicating whether time has been init
   double J2000ETInit;         /// s Seconds elapsed since J2000 at init
   double J2000Current;        /// s Current J2000 elapsed time
   double JulianDateCurrent;   /// s Current JulianDate
   double GPSSeconds;          /// s Current GPS seconds 
   uint16_t GPSWeek;           /// -- Current GPS week value
   uint64_t GPSRollovers;      /// -- Count on the number of GPS rollovers

private:
   std::string GPSEpochTime;   /// -- String for the GPS epoch
   double JDGPSEpoch;          /// s Epoch for GPS time.  Saved for efficiency
   int64_t TimeOutMsgID;       /// -- Output time message ID
   std::map<uint32_t, SpicePlanetState> PlanetData; /// -- Internal vector of planets

};

#endif
