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

#include "magneticFieldWMM.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/bsk_Print.h"
#include "utilities/simDefinitions.h"
#include "EGM9615.h"
#include "architecture/messaging/system_messaging.h"
#include <time.h>

#define MAX_CHAR_LENGTH 100


/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.
 @return void
 */
MagneticFieldWMM::MagneticFieldWMM()
{
    //! - Set the default magnetic field properties
    this->planetRadius = REQ_EARTH*1000.;
    this->magneticModels[0] = nullptr;

    //! - Set default epoch date.  This is over-written if an epoch message is provided
    MAGtype_Date calendar;
    char Error_Message[255];
    calendar.Year = EPOCH_YEAR;
    calendar.Month = EPOCH_MONTH;
    calendar.Day = EPOCH_DAY;
    if (!MAG_DateToYear(&calendar, Error_Message)){
        BSK_PRINT(MSG_ERROR, "Could not convert default date to decimal year in constructor. \nError message: %s", Error_Message);
    }
    this->epochDateFractionalYear = calendar.DecimalYear + EPOCH_HOUR/(24.*365);

    return;
}

/*! Clean up any memory allocations.
 @return void
 */
MagneticFieldWMM::~MagneticFieldWMM()
{
    if (this->magneticModels[0] != nullptr) {
        cleanupEarthMagFieldModel();
    }

    return;
}


/*! Custom CrossInit() method.  Subscribe to the epoch message.
 @return void
 */
void MagneticFieldWMM::customCrossInit()
{
    //! - Subscribe to the optional Epoch Date/Time message
    this->epochInMsgId = -1;
    if (this->epochInMsgName.length() > 0) {
        this->epochInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->epochInMsgName, sizeof(EpochSimMsg), moduleID);
    }

    return;
}



/*! Custom Reset() method.  This loads the WMM coefficient file and gets the model setup.
 @return void
 */
void MagneticFieldWMM::customReset(uint64_t CurrentClock)
{
    if (this->magneticModels[0] != nullptr) {
        /* clean up the prior initialization */
        cleanupEarthMagFieldModel();
        this->magneticModels[0] = nullptr;
    }

    //! - Check that required module variables are set
    if(this->dataPath == "") {
        BSK_PRINT(MSG_ERROR, "WMM data path was not set.  No WMM.");
        return;
    }

    //! - Initialize the WMM evaluation routines
    initializeWmm(this->dataPath.c_str());

    //! - Set the epoch time in terms of a fractional year
    if (this->epochInMsgId>=0) {
        // Read in the epoch message and set the internal time structure
        EpochSimMsg epochMsg;
        SingleMessageHeader LocalHeader;
        memset(&epochMsg, 0x0, sizeof(EpochSimMsg));
        SystemMessaging::GetInstance()->ReadMessage(this->epochInMsgId, &LocalHeader,
                                                    sizeof(EpochSimMsg),
                                                    reinterpret_cast<uint8_t*> (&epochMsg), moduleID);
        this->epochDateTime.tm_year = epochMsg.year - 1900;
        this->epochDateTime.tm_mon = epochMsg.month - 1;
        this->epochDateTime.tm_mday = epochMsg.day;
        this->epochDateTime.tm_hour = epochMsg.hours;
        this->epochDateTime.tm_min = epochMsg.minutes;
        this->epochDateTime.tm_sec = (int) round(epochMsg.seconds);
    } else {
        // If an epoch message is not provided, the epochTime variable must be set directly
        MAGtype_Date calendar;
        char Error[255];
        calendar.DecimalYear = this->epochDateFractionalYear;
        MAG_YearToDate(&calendar);
        this->epochDateTime.tm_year = calendar.Year- 1900;
        this->epochDateTime.tm_mon = calendar.Month - 1;
        this->epochDateTime.tm_mday = calendar.Day;
        MAG_DateToYear(&calendar, Error);
        double diff = this->epochDateFractionalYear - calendar.DecimalYear;
        this->epochDateTime.tm_hour = (int) round(diff*(24.*365));
        this->epochDateTime.tm_min = 0;
        this->epochDateTime.tm_sec = 0;
    }
    this->epochDateTime.tm_isdst = -1;
    mktime(&this->epochDateTime);

    return;
}

/*! This method is evaluates the centered dipole magnetic field model.
 @param msg magnetic field message structure
 @return void
 */
void MagneticFieldWMM::evaluateMagneticFieldModel(MagneticFieldSimMsg *msg, double currentTime)
{
    Eigen::Vector3d rHat_P;             // []    normalized position vector in E frame components
    double phi;                         // [rad] latitude
    double lambda;                      // [rad] longitude
    double h;                           // [m]   height above geoid
    double PM[3][3];                    // []    DCM from magnetic field frame to planet frame P
    double NM[3][3];                    // []    DCM from magnetic field frame to inertial frame N
    double B_M[3];                      // [T]   magnetic field in Magnetic field aligned frame
    double M2[3][3];                    // []    2nd axis rotation DCM
    double M3[3][3];                    // []    3rd axis rotation DCM
    struct tm localDateTime;            // []    date/time structure

    if (this->magneticModels[0] == NULL) {
        // no magnetic field was setup, set field to zero and return
        v3SetZero(msg->magField_N);
        return;
    }

    //! - compute normalized E-frame position vector
    rHat_P = this->r_BP_P.normalized();

    //! - compute spacecraft latitude and longitude
    phi = asin(rHat_P[2]);
    lambda = atan2(rHat_P[1], rHat_P[0]);
    h = (this->orbitRadius - this->planetRadius)/1000.; /* must be in km */

    //! - compute current decimalYear value
    MAGtype_Date calendar;
    char Error_Message[255];
    localDateTime = this->epochDateTime;
    localDateTime.tm_sec = (int) round(currentTime);   // sets the current seconds
    mktime(&localDateTime);

    calendar.Year = localDateTime.tm_year + 1900;
    calendar.Month = localDateTime.tm_mon + 1;
    calendar.Day = localDateTime.tm_mday;
    if (!MAG_DateToYear(&calendar, Error_Message)){
        BSK_PRINT(MSG_ERROR, "Could not convert default date to decimal year. \nError message: %s", Error_Message);
    }

    //! - evaluate NED magnetic field
    computeWmmField(calendar.DecimalYear + localDateTime.tm_hour/(24.*365), phi, lambda, h, B_M);

    //! - convert NED magnetic field M vector components into N-frame components and store in output message
    Euler2(phi + M_PI_2, M2);
    Euler3(-lambda, M3);
    m33MultM33(M3, M2, PM);
    m33tMultM33(this->planetState.J20002Pfix, PM, NM);
    m33MultV3(NM, B_M, msg->magField_N);

    return;
}


/*! Performs memory cleanup necessary for magnetic field models
 @param msg magnetic field message structure
 @return void
 */
void MagneticFieldWMM::cleanupEarthMagFieldModel()
{
    MAG_FreeMagneticModelMemory(timedMagneticModel);
    MAG_FreeMagneticModelMemory(magneticModels[0]);
}


void MagneticFieldWMM::computeWmmField(double decimalYear, double phi, double lambda, double h, double B_M[3])
{
    MAGtype_CoordSpherical      coordSpherical;
    MAGtype_CoordGeodetic       coordGeodetic;
    MAGtype_GeoMagneticElements geoMagneticElements;
    MAGtype_GeoMagneticElements errors;

    this->userDate.DecimalYear = decimalYear;

    /* set the Geodetic coordinates of the satellite */
    coordGeodetic.phi = phi * R2D; /* degrees North */
    coordGeodetic.lambda = lambda * R2D; /* degrees East  */
    /* If height is given above WGS-84 */
    coordGeodetic.HeightAboveEllipsoid = h; /* km */

    this->geoid.UseGeoid = 0;
    /* If height is given above MSL */
    //coordGeodetic.HeightAboveGeoid = h; /* km */
    //geoid.UseGeoid = 1;
    //MAG_ConvertGeoidToEllipsoidHeight(&coordGeodetic, &geoid);

    /* Convert from geodetic to Spherical Equations: 17-18, WMM Technical report */
    MAG_GeodeticToSpherical(this->ellip, coordGeodetic, &coordSpherical);

    /* Time adjust the coefficients, Equation 19, WMM Technical report */
    MAG_TimelyModifyMagneticModel(this->userDate, this->magneticModels[0], this->timedMagneticModel);
    /* Computes the geoMagnetic field elements and their time change */
    MAG_Geomag(this->ellip, coordSpherical, coordGeodetic, this->timedMagneticModel, &geoMagneticElements);
    MAG_CalculateGridVariation(coordGeodetic, &geoMagneticElements);
    MAG_WMMErrorCalc(geoMagneticElements.H, &errors);
    v3Set(geoMagneticElements.X, geoMagneticElements.Y, geoMagneticElements.Z, B_M);

    v3Scale(1e-9, B_M, B_M); /* convert nano-Tesla to Tesla */

    return;
}


void MagneticFieldWMM::initializeWmm(const char *dataPath)
{
    char fileName[MAX_CHAR_LENGTH];
    int nMax = 0;
    int nTerms;

    strcpy(fileName, dataPath);
    strcat(fileName, "WMM.COF");
    if (!MAG_robustReadMagModels(fileName, &(this->magneticModels), 1)) {
        BSK_PRINT(MSG_ERROR, "WMM unable to load file %s", fileName);
        return;
    }

    if(nMax < magneticModels[0]->nMax) {
        nMax = magneticModels[0]->nMax;
    }
    nTerms = ((nMax + 1) * (nMax + 2) / 2);
    /* For storing the time modified WMM Model parameters */
    this->timedMagneticModel = MAG_AllocateModelMemory(nTerms);
    if(this->magneticModels[0] == NULL || this->timedMagneticModel == NULL) {
        MAG_Error(2);
    }
    /* Set default values and constants */
    MAG_SetDefaults(&this->ellip, &this->geoid);

    this->geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    this->geoid.Geoid_Initialized = 1;
}
