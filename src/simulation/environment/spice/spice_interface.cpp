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
#include "environment/spice/spice_interface.h"
#include <iostream>
#include <sstream>
#include "../libs/cspice/include/SpiceUsr.h"
#include "architecture/messaging/system_messaging.h"
#include <string.h>
#include "utilities/bsk_Print.h"

/*! This constructor initializes the variables that spice uses.  Most of them are
 not intended to be changed, but a couple are user configurable.
 */

SpiceInterface::SpiceInterface()
{
    SPICEDataPath = "";
    SPICELoaded = false;
    charBufferSize = 512;
    CallCounts = 0;
    J2000ETInit = 0;
    J2000Current = 0.0;
    julianDateCurrent = 0.0;
    GPSSeconds = 0.0;
    GPSWeek = 0;
    GPSRollovers = 0;
    spiceBuffer = new uint8_t[charBufferSize];
    timeDataInit = false;
    JDGPSEpoch = 0.0;
    GPSEpochTime = "1980 January 6, 00:00:00.0";
    outputTimePort = "spice_time_output_data";
    outputBufferCount = 2;
    referenceBase = "j2000";
    zeroBase = "SSB";
	timeOutPicture = "MON DD,YYYY  HR:MN:SC.#### (UTC) ::UTC";
    return;
}

/*! The only needed activity in the destructor is to delete the spice I/O buffer
 that was allocated in the constructor*/
SpiceInterface::~SpiceInterface()
{
    delete [] this->spiceBuffer;
//    if(this->SPICELoaded)
//    {
//        this->clearKeeper();
//    }
    return;
}

void SpiceInterface::clearKeeper()
{
    kclear_c();
}

/*! This method initializes the object.  It creates the output messages,
 initializes the SPICE kernels, and initializes the planet/time data that
 gets used at run.
 @return void*/
void SpiceInterface::SelfInit()
{
    //! Begin method steps
    //! - Bail if the SPICEDataPath is not present
    if(this->SPICEDataPath == "")
    {
        BSK_PRINT(MSG_WARNING, "SPICE data path was not set.  No SPICE.");
        return;
    }
    //!- Load the SPICE kernels if they haven't already been loaded
    if(!this->SPICELoaded)
    {
        if(loadSpiceKernel((char *)"naif0012.tls", this->SPICEDataPath.c_str())) {
            BSK_PRINT(MSG_ERROR, "Unable to load %s", "naif0012.tls");
        }
        if(loadSpiceKernel((char *)"pck00010.tpc", this->SPICEDataPath.c_str())) {
            BSK_PRINT(MSG_ERROR, "Unable to load %s", "pck00010.tpc");
        }
        if(loadSpiceKernel((char *)"de-403-masses.tpc", this->SPICEDataPath.c_str())) {
            BSK_PRINT(MSG_ERROR, "Unable to load %s", "de-403-masses.tpc");
        }
        if(loadSpiceKernel((char *)"de430.bsp", this->SPICEDataPath.c_str())) {
            BSK_PRINT(MSG_ERROR, "Unable to load %s", "de430.tpc");
        }
        this->SPICELoaded = true;
    }
    //! Set the zero time values that will be used to compute the system time
    this->initTimeData();
    this->J2000Current = this->J2000ETInit;
    //! Compute planetary data so that it is present at time zero
    this->planetData.clear();
    this->computePlanetData();
    this->timeDataInit = true;

    // - This method populates the output messages at time zero
    this->Reset(0);
}

/*! This method is used to initialize the zero-time that will be used to
 calculate all system time values in the Update method.  It also creates the
 output message for time data
 @return void
 */
void SpiceInterface::initTimeData()
{
    double EpochDelteET;
    //! Begin method steps
    //! -Get the time value associated with the GPS epoch
    str2et_c(this->GPSEpochTime.c_str(), &this->JDGPSEpoch);
    //! - Get the time value associate with the requested UTC date
    str2et_c(this->UTCCalInit.c_str(), &this->J2000ETInit);
    
    //! - Take the JD epoch and get the elapsed time for it
    deltet_c(this->JDGPSEpoch, "ET", &EpochDelteET);
    
    //! - Create the output time message for SPICE
    this->timeOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage(this->outputTimePort, sizeof(SpiceTimeSimMsg),
        this->outputBufferCount, "SpiceTimeSimMsg", this->moduleID);
}

/*! This method computes the GPS time data for the current elapsed time.  It uses
 the total elapsed times at both the GPS epoch time and the current time to
 compute the GPS time (week, seconds, rollovers)
 @return void
 */
void SpiceInterface::computeGPSData()
{
    double JDDifference;
    
    //! Begin method steps
    //! - The difference between the epochs in julian date terms is the total
    JDDifference = this->J2000Current - this->JDGPSEpoch;
    //! - Scale the elapsed by a week's worth of seconds to get week
    this->GPSWeek = JDDifference/(7*86400);
    //! - Subtract out the GPS week scaled up to seconds to get time in week
    this->GPSSeconds = JDDifference - this->GPSWeek*7*86400;
    
    //! - Maximum GPS week is 1024 so get rollovers and subtract out those weeks
    this->GPSRollovers = this->GPSWeek/1024;
    this->GPSWeek -= this->GPSRollovers*1024;
}

/*! This method takes the values computed in the model and outputs them.
 It packages up the internal variables into the output structure definitions
 and puts them out on the messaging system
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void SpiceInterface::writeOutputMessages(uint64_t CurrentClock)
{
    std::map<uint32_t, SpicePlanetStateSimMsg>::iterator planit;
    SpiceTimeSimMsg OutputData;
    //! Begin method steps
    //! - Set the members of the time output message structure and write
    OutputData.J2000Current = this->J2000Current;
    OutputData.JulianDateCurrent = this->julianDateCurrent;
    OutputData.GPSSeconds = this->GPSSeconds;
    OutputData.GPSWeek = this->GPSWeek;
    OutputData.GPSRollovers = this->GPSRollovers;
    SystemMessaging::GetInstance()->WriteMessage(this->timeOutMsgID, CurrentClock,
                                                 sizeof(SpiceTimeSimMsg), reinterpret_cast<uint8_t*> (&OutputData), this->moduleID);
    
    //! - Iterate through all of the planets that are on and write their outputs
    for(planit = this->planetData.begin(); planit != this->planetData.end(); planit++)
    {
        SystemMessaging::GetInstance()->WriteMessage(planit->first, CurrentClock,
                                                     sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&planit->second), this->moduleID);
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
    this->J2000Current = this->J2000ETInit + CurrentSimNanos*1.0E-9;
    
    //! - Compute the current Julian Date string and cast it over to the double
    et2utc_c(this->J2000Current, "J", 14, this->charBufferSize - 1, reinterpret_cast<SpiceChar*>
             (this->spiceBuffer));
    std::string localString = reinterpret_cast<char*> (&this->spiceBuffer[3]);
    this->julianDateCurrent = std::stod(localString);
    //! Get GPS and Planet data and then write the message outputs
    this->computeGPSData();
    this->computePlanetData();
    this->writeOutputMessages(CurrentSimNanos);
}

void SpiceInterface::Reset(uint64_t CurrenSimNanos)
{
    // - Call Update state so that the spice bodies are inputted into the messaging system on reset
    this->UpdateState(CurrenSimNanos);
}

/*! This method gets the state of each planet that has been added to the model
 and saves the information off into the planet array.
 @return void
 */
void SpiceInterface::computePlanetData()
{
    std::vector<std::string>::iterator it;
    std::map<uint32_t, SpicePlanetStateSimMsg>::iterator planit;
    
    //! Begin method steps
    
    //! - Check to see if our planet vectors don't match (new planet requested)
    if(this->planetData.size() != this->planetNames.size())
    {
        SpiceChar *name = new SpiceChar[this->charBufferSize];
        SpiceBoolean frmFound;
        SpiceInt frmCode;
        //! - If we have a new planet, clear the old output vector and reset
        this->planetData.clear();
        //! - Loop over the planet names and create new data
        for(it = this->planetNames.begin(); it != this->planetNames.end(); it++)
        {
            //! <pre>       Hard limit on the maximum name length </pre>
            SpicePlanetStateSimMsg newPlanet;
            if(it->size() >= MAX_BODY_NAME_LENGTH)
            {
                BSK_PRINT(MSG_WARNING, "Warning, your planet name is too long for me.  Ignoring: %s", (*it).c_str());
                continue;
            }
            //! <pre>       Set the new planet name and zero the other struct elements </pre>
            std::string planetMsgName = *it + "_planet_data";
            memset(&newPlanet, 0x0, sizeof(SpicePlanetStateSimMsg));
            strcpy(newPlanet.PlanetName, it->c_str());
            //! <pre>       Create the new planet's ID and insert the planet into the vector </pre>
            uint32_t msgID = (uint32_t)SystemMessaging::GetInstance()->
                CreateNewMessage(planetMsgName, sizeof(SpicePlanetStateSimMsg),
                this->outputBufferCount, "SpicePlanetStateSimMsg", this->moduleID);
            std::string planetFrame = *it;
            cnmfrm_c(planetFrame.c_str(), this->charBufferSize, &frmCode, name, &frmFound);
            newPlanet.computeOrient = frmFound;
            this->planetData.insert(std::pair<uint32_t, SpicePlanetStateSimMsg>
                              (msgID, newPlanet));
        }
        delete [] name;
    }
    
    /*! - Loop over the PlanetData vector and compute values.
     
     -# Call the Ephemeris file (spkezr)
     -# Copy out the position and velocity values (default in km)
     -# Convert the pos/vel over to meters.
     -# Time stamp the message appropriately
     */
    for(planit = this->planetData.begin(); planit != this->planetData.end(); planit++)
    {
        double lighttime;
        double localState[6];
        spkezr_c(planit->second.PlanetName, this->J2000Current, this->referenceBase.c_str(),
            "NONE", zeroBase.c_str(), localState, &lighttime);
        memcpy(planit->second.PositionVector, &localState[0], 3*sizeof(double));
        memcpy(planit->second.VelocityVector, &localState[3], 3*sizeof(double));
        for(uint32_t i=0; i<3; i++)
        {
            planit->second.PositionVector[i]*=1000.0;
            planit->second.VelocityVector[i]*=1000.0;
        }
        planit->second.J2000Current = this->J2000Current;
        std::string planetFrame = "IAU_";
        planetFrame += planit->second.PlanetName;
        if(planit->second.computeOrient)
        {
            //pxform_c ( referenceBase.c_str(), planetFrame.c_str(), J2000Current,
            //    planit->second.J20002Pfix);
            
            double aux[6][6];
            
            sxform_c(this->referenceBase.c_str(), planetFrame.c_str(), this->J2000Current, aux); //returns attude of planet (i.e. IAU_EARTH) wrt "j2000". note j2000 is actually ICRF in Spice.
            
            m66Get33Matrix(0, 0, aux, planit->second.J20002Pfix);
            
            m66Get33Matrix(1, 0, aux, planit->second.J20002Pfix_dot);
        }
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
    char *fileName = new char[this->charBufferSize];
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    
    //! Begin method steps
    //! - The required calls come from the SPICE documentation.
    //! - The most critical call is furnsh_c
    strcpy(name, "REPORT");
    erract_c("SET", this->charBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    furnsh_c(fileName);
    
    //! - Check to see if we had trouble loading a kernel and alert user if so
    strcpy(name, "DEFAULT");
    erract_c("SET", this->charBufferSize, name);
    delete[] fileName;
    delete[] name;
    if(failed_c()) {
        return 1;
    }
    return 0;
}

/*! This method unloads a requested SPICE kernel into the system memory.  It is
 its own method because we have to load several SPICE kernels in for our
 application.  Note that they are stored in the SPICE library and are not
 held locally in this object.
 @return int Zero for success one for failure
 @param kernelName The name of the kernel we are unloading
 @param dataPath The path to the data area on the filesystem
 */
int SpiceInterface::unloadSpiceKernel(char *kernelName, const char *dataPath)
{
    char *fileName = new char[this->charBufferSize];
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    
    //! Begin method steps
    //! - The required calls come from the SPICE documentation.
    //! - The most critical call is furnsh_c
    strcpy(name, "REPORT");
    erract_c("SET", this->charBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    unload_c(fileName);
    if(failed_c()) {
        return 1;
    }
    return 0;
}

std::string SpiceInterface::getCurrentTimeString()
{
	char *spiceOutputBuffer;
	int64_t allowedOutputLength;

	allowedOutputLength = (int64_t)this->timeOutPicture.size() - 5;

	if (allowedOutputLength < 0)
	{
        BSK_PRINT(MSG_ERROR, "The output format string is not long enough. It should be much larger than 5 characters.  It is currently: %s", this->timeOutPicture.c_str());
		return("");
	}

	spiceOutputBuffer = new char[allowedOutputLength];
	timout_c(this->J2000Current, this->timeOutPicture.c_str(), (SpiceInt) allowedOutputLength,
		spiceOutputBuffer);
	std::string returnTimeString = spiceOutputBuffer;
	delete[] spiceOutputBuffer;
	return(returnTimeString);
}
