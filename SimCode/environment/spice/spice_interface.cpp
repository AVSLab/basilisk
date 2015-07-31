#include "environment/spice/spice_interface.h"
#include "../External/cspice/include/SpiceUsr.h"
#include "architecture/messaging/system_messaging.h"
#include <iostream>
#include <boost/lexical_cast.hpp>

/*! This constructor initializes the variables that spice uses.  Most of them are 
    not intended to be changed, but a couple are user configurable.
*/

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

/*! The only needed activity in the destructor is to delete the spice I/O buffer
    that was allocated in the constructor*/
SpiceInterface::~SpiceInterface()
{
   delete [] SpiceBuffer;
   return;
}

/*! This method initializes the object.  It creates the output messages, 
    initializes the SPICE kernels, and initializes the planet/time data that 
    gets used at run. 
    @return void*/
void SpiceInterface::SelfInit()
{
   //! Begin method steps
   //! - Bail if the SPICEDataPath is not present
   if(SPICEDataPath == "")
   {
      std::cerr << "Warning, SPICE data path was not set.  No SPICE."<<
         std::endl;
      return;
   }
   //!- Load the SPICE kernels if they haven't already been loaded
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
   //! Set the zero time values that will be used to compute the system time
   InitTimeData();
   //! Compute planetary data so that it is present at time zero
   ComputePlanetData();
   TimeDataInit = true;
}

/*! This method is used to initialize the zero-time that will be used to 
    calculate all system time values in the Update method.  It also creates the 
    output message for time data 
    @return void
*/
void SpiceInterface::InitTimeData()
{
   double EpochDelteET;
   //! Begin method steps
   //! -Get the time value associated with the GPS epoch
   str2et_c(GPSEpochTime.c_str(), &JDGPSEpoch);
   //! - Get the time value associate with the requested UTC date
   str2et_c(UTCCalInit.c_str(), &J2000ETInit);

   //! - Take the JD epoch and get the elapsed time for it 
   deltet_c(JDGPSEpoch, "ET", &EpochDelteET);

   //! - Create the output time message for SPICE
   TimeOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(
      OutputTimePort, sizeof(SpiceTimeOutput), OutputBufferCount); 
 
}

/*! This method computes the GPS time data for the current elapsed time.  It uses
    the total elapsed times at both the GPS epoch time and the current time to 
    compute the GPS time (week, seconds, rollovers)
    @return void
*/
void SpiceInterface::ComputeGPSData()
{
   double JDDifference;

   //! Begin method steps
   //! - The difference between the epochs in julian date terms is the total 
   JDDifference = J2000Current - JDGPSEpoch;
   //! - Scale the elapsed by a week's worth of seconds to get week
   GPSWeek = JDDifference/(7*86400);
   //! - Subtract out the GPS week scaled up to seconds to get time in week
   GPSSeconds = JDDifference - GPSWeek*7*86400;

   //! - Maximum GPS week is 1024 so get rollovers and subtract out those weeks
   GPSRollovers = GPSWeek/1024;
   GPSWeek -= GPSRollovers*1024;
}

/*! This method takes the values computed in the model and outputs them.  
    It packages up the internal variables into the output structure definitions 
    and puts them out on the messaging system
    @return void
    @param CurrentClock The current simulation time (used for time stamping)
*/
void SpiceInterface::SendOutputData(uint64_t CurrentClock)
{
   std::map<uint32_t, SpicePlanetState>::iterator planit;
   SpiceTimeOutput OutputData;
   //! Begin method steps
   //! - Set the members of the time output message structure and write
   OutputData.J2000Current = J2000Current;
   OutputData.JulianDateCurrent = JulianDateCurrent;
   OutputData.GPSSeconds = GPSSeconds;
   OutputData.GPSWeek = GPSWeek;
   OutputData.GPSRollovers = GPSRollovers;
   SystemMessaging::GetInstance()->WriteMessage(TimeOutMsgID, CurrentClock,
      sizeof(SpiceTimeOutput), reinterpret_cast<uint8_t*> (&OutputData));

   //! - Iterate through all of the planets that are on and write their outputs
   for(planit = PlanetData.begin(); planit != PlanetData.end(); planit++)
   {
      SystemMessaging::GetInstance()->WriteMessage(planit->first, CurrentClock,
         sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&planit->second));
   }

}

/*! This method is the interface point between the upper level simulation and 
    the SPICE interface at runtime.  It calls all of the necessary lower level 
    methods.
    @return void
    @param CurrentSimNanos The current clock time for the simulation
*/
void SpiceInterface::UpdateState(uint64_t CurrentSimNanos)
{
   //! Begin method steps
   //! - Increment the J2000 elapsed time based on init value and Current sim
   J2000Current = J2000ETInit + CurrentSimNanos*1.0E-9; 
 
   //! - Compute the current Julian Date string and cast it over to the double
   et2utc_c(J2000Current, "J", 14, CharBufferSize - 1, reinterpret_cast<SpiceChar*>
      (SpiceBuffer));
   std::string LocalString = reinterpret_cast<char*> (&SpiceBuffer[3]);
   JulianDateCurrent = boost::lexical_cast<double>(LocalString);

   //! Get GPS and Planet data and then write the message outputs
   ComputeGPSData();
   ComputePlanetData();
   SendOutputData(CurrentSimNanos);
}

/*! This method gets the state of each planet that has been added to the model 
    and saves the information off into the planet array.
    @return void
*/
void SpiceInterface::ComputePlanetData()
{
   std::vector<std::string>::iterator it;
   std::map<uint32_t, SpicePlanetState>::iterator planit;

   //! Begin method steps

   //! - Check to see if our planet vectors don't match (new planet requested)
   if(PlanetData.size() != PlanetNames.size())
   {
      //! - If we have a new planet, clear the old output vector and reset
      PlanetData.clear();
      //! - Loop over the planet names and create new data
      for(it=PlanetNames.begin(); it != PlanetNames.end(); it++)
      {
         //! <pre>       Hard limit on the maximum name length </pre>
         SpicePlanetState NewPlanet;
         if(it->size() >= MAX_BODY_NAME_LENGTH)
         {
            std::cerr << "Warning, your planet name is too long for me. ";
            std::cerr << "Ignoring: " << *it <<std::endl;
            continue;
         }
         //! <pre>       Set the new planet name and zero the other struct elements </pre>
         std::string PlanetMsgName = *it + "_planet_data";
         memset(&NewPlanet, 0x0, sizeof(SpicePlanetState));
         strcpy(NewPlanet.PlanetName, it->c_str());
         //! <pre>       Create the new planet's ID and insert the planet into the vector </pre>
         uint32_t MsgID = SystemMessaging::GetInstance()->CreateNewMessage(
            PlanetMsgName, sizeof(SpicePlanetState), OutputBufferCount); 
         PlanetData.insert(std::pair<uint32_t, SpicePlanetState> 
            (MsgID, NewPlanet));
      }
   }

   /*! - Loop over the PlanetData vector and compute values.

          -# Call the Ephemeris file (spkezr)
          -# Copy out the position and velocity values (default in km)
          -# Convert the pos/vel over to meters.
          -# Time stamp the message appropriately
   */
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

/*! This method loads a requested SPICE kernel into the system memory.  It is 
    its own method because we have to load several SPICE kernels in for our 
    application.  Note that they are stored in the SPICE library and are not 
    held locally in this object.
    @return int Zero for success one for failure
    @param kernelName The name of the kernel we are loading
    @param dataPath The path to the data area on the filesystem
*/
int SpiceInterface::loadSpiceKernel(char *kernelName, const char *dataPath)
{
    char *fileName = new char[CharBufferSize];
    SpiceChar *name = new SpiceChar[CharBufferSize];

    //! Begin method steps
    //! - The required calls come from the SPICE documentation.  
    //! - The most critical call is furnsh_c
    strcpy(name, "REPORT");
    erract_c("SET", CharBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    furnsh_c(fileName);

    //! - Check to see if we had trouble loading a kernel and alert user if so
    strcpy(name, "DEFAULT");
    erract_c("SET", CharBufferSize, name);
	delete[] fileName;
	delete[] name;
    if(failed_c()) {
        return 1;
    }
    return 0;
}
