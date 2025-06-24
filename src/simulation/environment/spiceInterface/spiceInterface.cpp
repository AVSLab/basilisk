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
#include "simulation/environment/spiceInterface/spiceInterface.h"
#include <sstream>
#include "SpiceUsr.h"
#include <string.h>
#include "architecture/utilities/simDefinitions.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"

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

    referenceBase = "j2000";
    zeroBase = "SSB";
	timeOutPicture = "MON DD,YYYY  HR:MN:SC.#### (UTC) ::UTC";

    //! - set default epoch time information
    char string[255];
    snprintf(string, 255, "%4d/%02d/%02d, %02d:%02d:%04.1f (UTC)", EPOCH_YEAR, EPOCH_MONTH, EPOCH_DAY, EPOCH_HOUR, EPOCH_MIN, EPOCH_SEC);
    this->UTCCalInit = string;

    return;
}

/*! The only needed activity in the destructor is to delete the spice I/O buffer
 that was allocated in the constructor*/
SpiceInterface::~SpiceInterface()
{
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++) {
        delete this->planetStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++) {
        delete this->scStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->attRefStateOutMsgs.size(); c++) {
        delete this->attRefStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->transRefStateOutMsgs.size(); c++) {
        delete this->transRefStateOutMsgs.at(c);
    }
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


/*! Reset the module to origina configuration values.

 */
void SpiceInterface::Reset(uint64_t CurrenSimNanos)
{
    //! - Bail if the SPICEDataPath is not present
    if(this->SPICEDataPath == "")
    {
        bskLogger.bskLog(BSK_ERROR, "SPICE data path was not set.  No SPICE.");
        return;
    }
    //!- Load the SPICE kernels if they haven't already been loaded
    if(!this->SPICELoaded)
    {
        if(loadSpiceKernel((char *)"naif0012.tls", this->SPICEDataPath.c_str())) {
            bskLogger.bskLog(BSK_ERROR, "Unable to load %s", "naif0012.tls");
        }
        if(loadSpiceKernel((char *)"pck00010.tpc", this->SPICEDataPath.c_str())) {
            bskLogger.bskLog(BSK_ERROR, "Unable to load %s", "pck00010.tpc");
        }
        if(loadSpiceKernel((char *)"de-403-masses.tpc", this->SPICEDataPath.c_str())) {
            bskLogger.bskLog(BSK_ERROR, "Unable to load %s", "de-403-masses.tpc");
        }
        if(loadSpiceKernel((char *)"de430.bsp", this->SPICEDataPath.c_str())) {
            bskLogger.bskLog(BSK_ERROR, "Unable to load %s", "de430.tpc");
        }
        this->SPICELoaded = true;
    }

    //! Set the zero time values that will be used to compute the system time
    this->initTimeData();
    this->J2000Current = this->J2000ETInit;
    this->timeDataInit = true;

    std::vector<SpicePlanetStateMsgPayload>::iterator planit;
    size_t c = 0;  // celestial object counter
    int autoFrame;  // flag to set the frame automatically
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    SpiceBoolean frmFound;
    SpiceInt frmCode;
    for(planit = this->planetData.begin(); planit != planetData.end(); planit++)
    {
        autoFrame = 1;
        planit->computeOrient = 0;  // turn off by default
        if (this->planetFrames.size() > 0) {
            if (c < this->planetFrames.size()) {
                if (this->planetFrames[c].length() > 0) {
                    planit->computeOrient = 1;  // turn on as a custom name is provided
                    autoFrame = 0;
                }
            }
        }
        if (autoFrame > 0) {
            std::string planetFrame = planit->PlanetName;
            cnmfrm_c(planetFrame.c_str(), this->charBufferSize, &frmCode, name, &frmFound);
            planit->computeOrient = frmFound;  // set the flag to the Spice response on finding this frame
        }
        c++;
    }
    delete [] name;

    // - Call Update state so that the spice bodies are inputted into the messaging system on reset
    this->UpdateState(CurrenSimNanos);
}


/*! This method is used to initialize the zero-time that will be used to
 calculate all system time values in the Update method.  It also creates the
 output message for time data

 */
void SpiceInterface::initTimeData()
{
    double EpochDelteET;

    /* set epoch information.  If provided, then the epoch message information should be used.  */
    if (this->epochInMsg.isLinked()) {
        // Read in the epoch message and set the internal time structure
        EpochMsgPayload epochMsg;
        epochMsg = this->epochInMsg();
        if (!this->epochInMsg.isWritten()) {
            bskLogger.bskLog(BSK_ERROR, "The input epoch message name was set, but the message was never written.  Not using the input message.");
        } else {
            // Set the epoch information from the input message
            char string[255];
            snprintf(string, 255, "%4d/%02d/%02d, %02d:%02d:%04.6f (UTC)", epochMsg.year, epochMsg.month, epochMsg.day, epochMsg.hours, epochMsg.minutes, epochMsg.seconds);
            this->UTCCalInit = string;
        }
    }

    //! -Get the time value associated with the GPS epoch
    str2et_c(this->GPSEpochTime.c_str(), &this->JDGPSEpoch);
    //! - Get the time value associate with the requested UTC date
    str2et_c(this->UTCCalInit.c_str(), &this->J2000ETInit);
    //! - Take the JD epoch and get the elapsed time for it
    deltet_c(this->JDGPSEpoch, "ET", &EpochDelteET);

}

/*! This method computes the GPS time data for the current elapsed time.  It uses
 the total elapsed times at both the GPS epoch time and the current time to
 compute the GPS time (week, seconds, rollovers)

 */
void SpiceInterface::computeGPSData()
{
    double JDDifference;

    //! - The difference between the epochs in julian date terms is the total
    JDDifference = this->J2000Current - this->JDGPSEpoch;
    //! - Scale the elapsed by a week's worth of seconds to get week
    this->GPSWeek = (uint16_t) (JDDifference/(7*86400));
    //! - Subtract out the GPS week scaled up to seconds to get time in week
    this->GPSSeconds = JDDifference - this->GPSWeek*7*86400;

    //! - Maximum GPS week is 1024 so get rollovers and subtract out those weeks
    this->GPSRollovers = this->GPSWeek/1024;
    this->GPSWeek = (uint16_t)(this->GPSWeek-this->GPSRollovers*1024);
}

/*! This method takes the values computed in the model and outputs them.
 It packages up the internal variables into the output structure definitions
 and puts them out on the messaging system

 @param CurrentClock The current simulation time (used for time stamping)
 */
void SpiceInterface::writeOutputMessages(uint64_t CurrentClock)
{
    SpiceTimeMsgPayload OutputData;

    //! - Set the members of the time output message structure and write
    OutputData.J2000Current = this->J2000Current;
    OutputData.JulianDateCurrent = this->julianDateCurrent;
    OutputData.GPSSeconds = this->GPSSeconds;
    OutputData.GPSWeek = this->GPSWeek;
    OutputData.GPSRollovers = this->GPSRollovers;
    this->spiceTimeOutMsg.write(&OutputData, this->moduleID, CurrentClock);

    //! - Iterate through all of the planets that are on and write their outputs
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++)
    {
        this->planetStateOutMsgs[c]->write(&this->planetData[c], this->moduleID, CurrentClock);
    }

    //! - Iterate through all of the spacecraft that are on and write their outputs
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++)
    {
        SCStatesMsgPayload scStateMsgData = {};
        v3Copy(this->scData[c].PositionVector, scStateMsgData.r_BN_N);
        v3Copy(this->scData[c].PositionVector, scStateMsgData.r_CN_N);
        v3Copy(this->scData[c].VelocityVector, scStateMsgData.v_BN_N);
        v3Copy(this->scData[c].VelocityVector, scStateMsgData.v_CN_N);
        C2MRP(this->scData[c].J20002Pfix, scStateMsgData.sigma_BN);
        this->scStateOutMsgs[c]->write(&scStateMsgData, this->moduleID, CurrentClock);

        AttRefMsgPayload attRefMsgData = {};
        C2MRP(this->scData[c].J20002Pfix, attRefMsgData.sigma_RN);
        this->attRefStateOutMsgs[c]->write(&attRefMsgData, this->moduleID, CurrentClock);

        TransRefMsgPayload transRefMsgData = {};
        v3Copy(this->scData[c].PositionVector, transRefMsgData.r_RN_N);
        v3Copy(this->scData[c].VelocityVector, transRefMsgData.v_RN_N);
        this->transRefStateOutMsgs[c]->write(&transRefMsgData, this->moduleID, CurrentClock);

    }
}

/*! This method is the interface point between the upper level simulation and
 the SPICE interface at runtime.  It calls all of the necessary lower level
 methods.

 @param CurrentSimNanos The current clock time for the simulation
 */
void SpiceInterface::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Increment the J2000 elapsed time based on init value and Current sim
    this->J2000Current = this->J2000ETInit + CurrentSimNanos*NANO2SEC;

    //! - Compute the current Julian Date string and cast it over to the double
    et2utc_c(this->J2000Current, "J", 14, this->charBufferSize - 1, reinterpret_cast<SpiceChar*>
             (this->spiceBuffer));
    std::string localString = reinterpret_cast<char*> (&this->spiceBuffer[3]);
    this->julianDateCurrent = std::stod(localString);
    //! Get GPS and Planet data and then write the message outputs
    this->computeGPSData();
    this->pullSpiceData(&this->planetData);
    this->pullSpiceData(&this->scData);
    this->writeOutputMessages(CurrentSimNanos);
}

/*! take a vector of planet name strings and create the vector of
    planet state output messages and the vector of planet state message payloads */
void SpiceInterface::addPlanetNames(std::vector<std::string> planetNames) {
    std::vector<std::string>::iterator it;

    /* clear the planet state message and payload vectors */
    for (long unsigned int c=0; c<this->planetStateOutMsgs.size(); c++) {
        delete this->planetStateOutMsgs.at(c);
    }
    this->planetStateOutMsgs.clear();
    this->planetData.clear();

    for (it = planetNames.begin(); it != planetNames.end(); it++) {
        Message<SpicePlanetStateMsgPayload> *spiceOutMsg;
        spiceOutMsg = new Message<SpicePlanetStateMsgPayload>;
        this->planetStateOutMsgs.push_back(spiceOutMsg);

        SpicePlanetStateMsgPayload newPlanet = {};
        m33SetIdentity(newPlanet.J20002Pfix);
        if(it->size() >= MAX_BODY_NAME_LENGTH)
        {
            bskLogger.bskLog(BSK_WARNING, "spiceInterface: Warning, your planet name is too long for me.  Ignoring: %s", (*it).c_str());
            continue;
        }
        strcpy(newPlanet.PlanetName, it->c_str());

        this->planetData.push_back(newPlanet);
    }

    return;
}

/*! take a vector of spacecraft name strings and create the vectors of
    spacecraft state output messages and the vector of spacecraft state message payloads */
void SpiceInterface::addSpacecraftNames(std::vector<std::string> spacecraftNames) {
    std::vector<std::string>::iterator it;
    SpiceChar *name = new SpiceChar[this->charBufferSize];
    SpiceBoolean frmFound;
    SpiceInt frmCode;

    /* clear the spacecraft state message and payload vectors */
    for (long unsigned int c=0; c<this->scStateOutMsgs.size(); c++) {
        delete this->scStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->attRefStateOutMsgs.size(); c++) {
        delete this->attRefStateOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->transRefStateOutMsgs.size(); c++) {
        delete this->transRefStateOutMsgs.at(c);
    }
    this->scStateOutMsgs.clear();
    this->attRefStateOutMsgs.clear();
    this->transRefStateOutMsgs.clear();
    this->scData.clear();

    for (it = spacecraftNames.begin(); it != spacecraftNames.end(); it++) {
        /* append to spacecraft related output messages */
        Message<SCStatesMsgPayload> *scStateOutMsg;
        scStateOutMsg = new Message<SCStatesMsgPayload>;
        this->scStateOutMsgs.push_back(scStateOutMsg);

        Message<AttRefMsgPayload> *attRefOutMsg;
        attRefOutMsg = new Message<AttRefMsgPayload>;
        this->attRefStateOutMsgs.push_back(attRefOutMsg);

        Message<TransRefMsgPayload> *transRefOutMsg;
        transRefOutMsg = new Message<TransRefMsgPayload>;
        this->transRefStateOutMsgs.push_back(transRefOutMsg);

        SpicePlanetStateMsgPayload newSpacecraft = {};
        m33SetIdentity(newSpacecraft.J20002Pfix);
        if(it->size() >= MAX_BODY_NAME_LENGTH)
        {
            bskLogger.bskLog(BSK_WARNING, "spiceInterface: Warning, your spacecraft name is too long for me.  Ignoring: %s", (*it).c_str());
            continue;
        }
        strcpy(newSpacecraft.PlanetName, it->c_str());

        std::string planetFrame = *it;
        cnmfrm_c(planetFrame.c_str(), this->charBufferSize, &frmCode, name, &frmFound);
        newSpacecraft.computeOrient = frmFound;
        this->scData.push_back(newSpacecraft);
    }
    delete [] name;

    return;
}


/*! This method gets the state of each spice item that has been added to the module
 and saves the information off into the array.

 */
void SpiceInterface::pullSpiceData(std::vector<SpicePlanetStateMsgPayload> *spiceData)
{
    std::vector<SpicePlanetStateMsgPayload>::iterator planit;

    /*! - Loop over the vector of Spice objects and compute values.

     -# Call the Ephemeris file (spkezr)
     -# Copy out the position and velocity values (default in km)
     -# Convert the pos/vel over to meters.
     -# Time stamp the message appropriately
     */
    size_t c = 0; // celestial body counter
    for(planit = spiceData->begin(); planit != spiceData->end(); planit++)
    {
        double lighttime;
        double localState[6];
        std::string planetFrame = "";

        spkezr_c(planit->PlanetName, this->J2000Current, this->referenceBase.c_str(),
            "NONE", this->zeroBase.c_str(), localState, &lighttime);
        v3Copy(&localState[0], planit->PositionVector);
        v3Copy(&localState[3], planit->VelocityVector);
        v3Scale(1000., planit->PositionVector, planit->PositionVector);
        v3Scale(1000., planit->VelocityVector, planit->VelocityVector);
        planit->J2000Current = this->J2000Current;
        /* use default IAU planet frame name */
        planetFrame = "IAU_";
        planetFrame += planit->PlanetName;

        /* use specific planet frame if specified */
        if (this->planetFrames.size() > 0) {
            if (c < this->planetFrames.size()) {
                if (this->planetFrames[c].length() > 0){
                    /* use custom planet frame name */
                    planetFrame = this->planetFrames[c];
                }
            }
        }

        if(planit->computeOrient)
        {
            //pxform_c ( referenceBase.c_str(), planetFrame.c_str(), J2000Current,
            //    planit->second.J20002Pfix);

            double aux[6][6];

            sxform_c(this->referenceBase.c_str(), planetFrame.c_str(), this->J2000Current, aux); //returns attitude of planet (i.e. IAU_EARTH) wrt "j2000". note j2000 is actually ICRF in Spice.

            m66Get33Matrix(0, 0, aux, planit->J20002Pfix);

            m66Get33Matrix(1, 0, aux, planit->J20002Pfix_dot);
        }
        c++;
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

    //! - The required calls come from the SPICE documentation.
    //! - The most critical call is furnsh_c
    strcpy(name, "REPORT");
    erract_c("SET", this->charBufferSize, name);
    strcpy(fileName, dataPath);
    strcat(fileName, kernelName);
    unload_c(fileName);
    delete[] fileName;
    delete[] name;
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
        bskLogger.bskLog(BSK_ERROR, "The output format string is not long enough. It should be much larger than 5 characters.  It is currently: %s", this->timeOutPicture.c_str());
		return("");
	}

	spiceOutputBuffer = new char[allowedOutputLength];
	timout_c(this->J2000Current, this->timeOutPicture.c_str(), (SpiceInt) allowedOutputLength,
		spiceOutputBuffer);
	std::string returnTimeString = spiceOutputBuffer;
	delete[] spiceOutputBuffer;
	return(returnTimeString);
}
