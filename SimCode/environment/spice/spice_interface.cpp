#include "environment/spice/spice_interface.h"
#include "../External/cspice/include/SpiceUsr.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include <boost/lexical_cast.hpp>

SpiceInterface::SpiceInterface() 
{
   SPICEDataPath = "";
   SPICELoaded = false;
   CharBufferSize = 512;
   CallCounts = 0;
   J2000ETInit = 0;
   J2000Current = 0.0;
   JulianDateCurrent = 0.0;
   GPSSeconds = 0.0;
   GPSWeek = 0;
   GPSRollovers = 0;
   SpiceBuffer = new uint8_t[CharBufferSize];
   TimeDataInit = false;
   JDGPSEpoch = 0.0;
   GPSEpochTime = "1980 January 6, 00:00:00.0";
   OutputTimePort = "spice_time_output_data";
   OutputBufferCount = 2;
   return;
}

SpiceInterface::~SpiceInterface()
{
   return;
}

void SpiceInterface::SelfInit()
{
   if(SPICEDataPath == "")
   {
      std::cerr << "Warning, SPICE data path was not set.  No SPICE."<<
         std::endl;
      return;
   }
   if(!SPICELoaded)
   {
      if(loadSpiceKernel((char *)"naif0010.tls", SPICEDataPath.c_str())) {
           printf("Unable to load %s", "naif0010.tls");
       }
       if(loadSpiceKernel((char *)"pck00010.tpc", SPICEDataPath.c_str())) {
           printf("Unable to load %s", "pck00010.tpc");
       }
       if(loadSpiceKernel((char *)"de-403-masses.tpc", SPICEDataPath.c_str())) {
           printf("Unable to load %s", "de-403-masses.tpc");
       }
       if(loadSpiceKernel((char *)"de421.bsp", SPICEDataPath.c_str())) {
           printf("Unable to load %s", "de421.bsp");
       }
       if(loadSpiceKernel((char *)"MAR033_2000-2025.bsp", SPICEDataPath.c_str())) {
           printf("Unable to load %s", "MAR033_2000-2025.bsp");
       }
      SPICELoaded = true;
   }
   InitTimeData();
   ComputePlanetData();
   TimeDataInit = true;
}

void SpiceInterface::InitTimeData()
{
   double EpochDelteET;
   str2et_c(GPSEpochTime.c_str(), &JDGPSEpoch);
   str2et_c(UTCCalInit.c_str(), &J2000ETInit);
 
   deltet_c(JDGPSEpoch, "ET", &EpochDelteET);

   TimeOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(
      OutputTimePort, sizeof(SpiceTimeOutput), OutputBufferCount); 
 
}

void SpiceInterface::ComputeGPSData()
{
   double JDDifference;
   double LeapSecondDouble;

   deltet_c(J2000Current, "ET", &LeapSecondDouble);

   JDDifference = J2000Current - JDGPSEpoch;
   GPSWeek = JDDifference/(7*86400);
   GPSSeconds = JDDifference - GPSWeek*7*86400;

   GPSRollovers = GPSWeek/1024;
   GPSWeek -= GPSRollovers*1024;
}
void SpiceInterface::SendOutputData(uint64_t CurrentClock)
{
   std::map<uint32_t, SpicePlanetState>::iterator planit;
   SpiceTimeOutput OutputData;
   OutputData.J2000Current = J2000Current;
   OutputData.JulianDateCurrent = JulianDateCurrent;
   OutputData.GPSSeconds = GPSSeconds;
   OutputData.GPSWeek = GPSWeek;
   OutputData.GPSRollovers = GPSRollovers;
   SystemMessaging::GetInstance()->WriteMessage(TimeOutMsgID, CurrentClock,
      sizeof(SpiceTimeOutput), reinterpret_cast<uint8_t*> (&OutputData));

   for(planit = PlanetData.begin(); planit != PlanetData.end(); planit++)
   {
      SystemMessaging::GetInstance()->WriteMessage(planit->first, CurrentClock,
         sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&planit->second));
   }

}

void SpiceInterface::UpdateState(uint64_t CurrentSimNanos)
{
   J2000Current = J2000ETInit + CurrentSimNanos*1.0E-9; 
   et2utc_c(J2000Current, "J", 14, CharBufferSize - 1, reinterpret_cast<SpiceChar*>
      (SpiceBuffer));
   std::string LocalString = reinterpret_cast<char*> (&SpiceBuffer[3]);
   JulianDateCurrent = boost::lexical_cast<double>(LocalString);
   ComputeGPSData();
   ComputePlanetData();
   SendOutputData(CurrentSimNanos);
}

void SpiceInterface::ComputePlanetData()
{
   std::vector<std::string>::iterator it;
   std::map<uint32_t, SpicePlanetState>::iterator planit;

   if(PlanetData.size() != PlanetNames.size())
   {
      PlanetData.clear();
      for(it=PlanetNames.begin(); it != PlanetNames.end(); it++)
      {
         SpicePlanetState NewPlanet;
         if(it->size() >= MAX_BODY_NAME_LENGTH)
         {
            std::cerr << "Warning, your planet name is too long for me. ";
            std::cerr << "Ignoring: " << *it <<std::endl;
            continue;
         }
         std::string PlanetMsgName = *it + "_planet_data";
         memset(&NewPlanet, 0x0, sizeof(SpicePlanetState));
         strcpy(NewPlanet.PlanetName, it->c_str());
         uint32_t MsgID = SystemMessaging::GetInstance()->CreateNewMessage(
            PlanetMsgName, sizeof(SpicePlanetState), OutputBufferCount); 
         PlanetData.insert(std::pair<uint32_t, SpicePlanetState> 
            (MsgID, NewPlanet));
      }
   }

   for(planit=PlanetData.begin(); planit != PlanetData.end(); planit++)
   {
      double lighttime;
      double LocalState[6];
      spkezr_c(planit->second.PlanetName, J2000Current, "j2000", "NONE", "SSB", 
         LocalState, &lighttime);
      memcpy(planit->second.PositionVector, &LocalState[0], 3*sizeof(double));
      memcpy(planit->second.VelocityVector, &LocalState[3], 3*sizeof(double));
      for(uint32_t i=0; i<3; i++)
      {
         planit->second.PositionVector[i]*=1000.0;
         planit->second.VelocityVector[i]*=1000.0;
      }
      planit->second.J2000Current = J2000Current;
   }

}

int SpiceInterface::loadSpiceKernel(char *kernelName, const char *dataPath)
{
    char fileName[CharBufferSize];
    SpiceChar name[CharBufferSize];

    strcpy(name, "REPORT");
    erract_c("SET", CharBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    furnsh_c(fileName);

    strcpy(name, "DEFAULT");
    erract_c("SET", CharBufferSize, name);
    if(failed_c()) {
        return 1;
    }
    return 0;
}
