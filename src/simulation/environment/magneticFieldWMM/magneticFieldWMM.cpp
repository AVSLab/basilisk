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
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "EGM9615.h"

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.

 */
MagneticFieldWMM::MagneticFieldWMM()
{
    //! - Set the default magnetic field properties
    this->planetRadius = REQ_EARTH*1000.;   // must be the radius of Earth for WMM
    this->magneticModel = nullptr;      // a nullptr means no WMM coefficients have been loaded
    this->epochDateFractionalYear = -1;     // negative value means this variable has not been set
}

/*! Clean up any memory allocations.

 */
MagneticFieldWMM::~MagneticFieldWMM()
{
    if (this->magneticModel != nullptr) {
        cleanupEarthMagFieldModel();
    }
}

/*! Custom Reset() method.  This loads the WMM coefficient file and gets the model setup.

 */
void MagneticFieldWMM::customReset(uint64_t CurrentClock)
{
    if (this->magneticModel != nullptr) {
        /* clean up the prior initialization */
        cleanupEarthMagFieldModel();
        this->magneticModel = nullptr;
    }

    //! - Check that required module variables are set
    if(this->dataPath == "") {
        bskLogger.bskLog(BSK_ERROR, "WMM data path was not set.  No WMM.");
        return;
    }

    //! - Initialize the WMM evaluation routines
    initializeWmm();
}

/*! Custom customSetEpochFromVariable() method.  This allows specifying epochDateFractionYear directly from Python.  If an epoch message is set then this variable is not used.

 */
void MagneticFieldWMM::customSetEpochFromVariable()
{
    //! - only convert if the fraction year variable was set to a non-zero value.  Otherwise use the BSK epoch default setup by the base class.
    if (this->epochDateFractionalYear > 0.0) {
        decimalYear2Gregorian(this->epochDateFractionalYear, &this->epochDateTime);
    }
}

/*! Convert a fraction year double value into a time structure with gregorian date/time information

 */
void MagneticFieldWMM::decimalYear2Gregorian(double fractionalYear, struct tm *gregorian)
{
    //! -Use the WMM routine to get the year, month and day information
    MAGtype_Date calendar;
    char Error[255];
    calendar.DecimalYear = this->epochDateFractionalYear;
    MAG_YearToDate(&calendar);
    gregorian->tm_year = calendar.Year - 1900;
    gregorian->tm_mon = calendar.Month - 1;
    gregorian->tm_mday = calendar.Day;

    //! - Find the unused remained of the fractional year and add to the gregorian calendar
    //! - determine number of days in this year
    double daysInYear = 365;
    if((calendar.Year % 4 == 0 && calendar.Year % 100 != 0) || calendar.Year % 400 == 0)
        daysInYear = 366;

    //! - determine missing hours
    MAG_DateToYear(&calendar, Error);
    double diff = this->epochDateFractionalYear - calendar.DecimalYear;
    this->epochDateTime.tm_hour = (int) round(diff * (24. * daysInYear));
    diff -= this->epochDateTime.tm_hour / ( 24. * daysInYear);

    //! - determine missing minutes
    this->epochDateTime.tm_min = (int) round(diff * (24. * 60 * daysInYear));
    diff -= this->epochDateTime.tm_min / (24. * 60 * daysInYear);

    //! - determine missing seconds
    this->epochDateTime.tm_sec = (int) round(diff * (24. * 60 * 60 * daysInYear));

    //! - ensure that daylight saving flag is off
    this->epochDateTime.tm_isdst = 0;

    //! - make sure a proper time structure is setup
    mktime(&this->epochDateTime);
}

/*! Convert a time structure with gregorian date/time information into a fraction year value.
 @return double
 */
double MagneticFieldWMM::gregorian2DecimalYear(double currentTime)
{
    double decimalYear;                 // [years]  fraction year date/time format
    struct tm localDateTime{};            // []       date/time structure

    //! - compute current decimalYear value
    MAGtype_Date calendar;
    char Error_Message[255];
    localDateTime = this->epochDateTime;
    localDateTime.tm_sec += (int) round(currentTime);   // sets the current seconds
    mktime(&localDateTime);

    calendar.Year = localDateTime.tm_year + 1900;
    calendar.Month = localDateTime.tm_mon + 1;
    calendar.Day = localDateTime.tm_mday;
    if (!MAG_DateToYear(&calendar, Error_Message)){
        bskLogger.bskLog(BSK_ERROR, "Could not convert date to decimal year. \nError message: %s", Error_Message);
    }

    //! - determine number of days in this year
    double daysInYear = 365;
    if((calendar.Year % 4 == 0 && calendar.Year % 100 != 0) || calendar.Year % 400 == 0)
        daysInYear = 366;

    decimalYear = calendar.DecimalYear;
    decimalYear += localDateTime.tm_hour / (24. * daysInYear);
    decimalYear += localDateTime.tm_min / (24. * 60. * daysInYear);
    decimalYear += localDateTime.tm_sec / (24. * 60. * 60. * daysInYear);

    return decimalYear;
}

/*! This method is evaluates the centered dipole magnetic field model.
 @param msg magnetic field message structure
 @param currentTime current time (s)

 */
void MagneticFieldWMM::evaluateMagneticFieldModel(MagneticFieldMsgPayload *msg, double currentTime)
{
    Eigen::Vector3d rHat_P;             // []    normalized position vector in E frame components
    double PM[3][3];                    // []    DCM from magnetic field frame to planet frame P
    double NM[3][3];                    // []    DCM from magnetic field frame to inertial frame N
    double B_M[3];                      // [T]   magnetic field in Magnetic field aligned frame
    double M2[3][3];                    // []    2nd axis rotation DCM
    double M3[3][3];                    // []    3rd axis rotation DCM

    if (this->magneticModel == nullptr) {
        // no magnetic field was setup, set field to zero and return
        v3SetZero(msg->magField_N);
        return;
    }

    //! - compute normalized E-frame position vector
    rHat_P = this->r_BP_P.normalized();

    //! - compute spacecraft latitude and longitude in spherical (geocentric) coordinates
    MAGtype_CoordSpherical coordSpherical;
    coordSpherical.phig = R2D * safeAsin(rHat_P[2]); /* degrees North */
    coordSpherical.lambda = R2D * atan2(rHat_P[1], rHat_P[0]); /* degrees East  */
    coordSpherical.r = this->orbitRadius / 1000.0; /* must be in km */

    //! - evaluate NED magnetic field
    computeWmmField(gregorian2DecimalYear(currentTime), coordSpherical, B_M);

    //! - convert NED magnetic field M vector components into N-frame components and store in output message
    Euler2(D2R * coordSpherical.phig + M_PI_2, M2);
    Euler3(D2R * -coordSpherical.lambda, M3);
    m33MultM33(M3, M2, PM);
    m33tMultM33(this->planetState.J20002Pfix, PM, NM);
    m33MultV3(NM, B_M, msg->magField_N);
}

/*! Performs memory cleanup necessary for magnetic field models

 */
void MagneticFieldWMM::cleanupEarthMagFieldModel()
{
    MAG_FreeMagneticModelMemory(this->timedMagneticModel);
    MAG_FreeMagneticModelMemory(this->magneticModel);

    MAG_FreeLegendreMemory(this->LegendreFunction);
    MAG_FreeSphVarMemory(this->SphVariables);
}

void MagneticFieldWMM::computeWmmField(double decimalYear, MAGtype_CoordSpherical coordSpherical, double B_M[3])
{
    MAGtype_MagneticResults MagneticResultsSph;

    this->userDate.DecimalYear = decimalYear;
    this->geoid.UseGeoid = 0;

    /* Time adjust the coefficients, Equation 19, WMM Technical report */
    MAG_TimelyModifyMagneticModel(this->userDate, this->magneticModel, this->timedMagneticModel);

    /* Compute Spherical Harmonic variables  */
    MAG_ComputeSphericalHarmonicVariables(this->ellip, coordSpherical, this->timedMagneticModel->nMax, this->SphVariables);

    /* Compute ALF  */
    MAG_AssociatedLegendreFunction(coordSpherical, this->timedMagneticModel->nMax, this->LegendreFunction);

    /* Accumulate the spherical harmonic coefficients*/
    MAG_Summation(this->LegendreFunction, this->timedMagneticModel, *this->SphVariables, coordSpherical, &MagneticResultsSph);

    v3Set(MagneticResultsSph.Bx, MagneticResultsSph.By, MagneticResultsSph.Bz, B_M);
    v3Scale(1e-9, B_M, B_M); /* convert nano-Tesla to Tesla */
}

void MagneticFieldWMM::initializeWmm()
{
    int nMax = 0;
    int nTerms;
    auto fileName = this->dataPath + "WMM.COF";

    MAGtype_MagneticModel *models[1];
    if (!MAG_robustReadMagModels(const_cast<char*>(fileName.c_str()), &models, 1)) {
        bskLogger.bskLog(BSK_ERROR, "WMM unable to load file %s", fileName.c_str());
        return;
    }
    this->magneticModel = models[0];

    if(nMax < magneticModel->nMax) {
        nMax = magneticModel->nMax;
    }
    nTerms = ((nMax + 1) * (nMax + 2) / 2);
    /* For storing the time modified WMM Model parameters */
    this->timedMagneticModel = MAG_AllocateModelMemory(nTerms);
    if(this->magneticModel == nullptr || this->timedMagneticModel == nullptr) {
        MAG_Error(2);
    }
    /* Set default values and constants */
    MAG_SetDefaults(&this->ellip, &this->geoid);

    this->LegendreFunction = MAG_AllocateLegendreFunctionMemory(nTerms); /* For storing the ALF functions */
    this->SphVariables = MAG_AllocateSphVarMemory(nMax);

    this->geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    this->geoid.Geoid_Initialized = 1;
}
