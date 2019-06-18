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
#include "cspice/include/SpiceUsr.h"
#include "WMM/EGM9615.h"

#define MAX_CHAR_LENGTH 100


/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.
 @return void
 */
MagneticFieldWMM::MagneticFieldWMM()
{
    //! - Set the default magnetic field properties
    this->planetRadius = REQ_EARTH*1000; // [m]

    return;
}

/*! Clean up any memory allocations.
 @return void
 */
MagneticFieldWMM::~MagneticFieldWMM()
{
    cleanupEarthMagFieldModel();

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
    double m33[3][3];
    double m33_2[3][3];

    //! - compute normalized E-frame position vector
    rHat_P = this->r_BP_P.normalized();

    //! - compute spacecraft latitude and longitude
    phi = asin(rHat_P[2]);
    lambda = atan2(rHat_P[1], rHat_P[0]);
    h = rHat_P.norm() - this->planetRadius;

    //! - compute the WMM magnetic field model
    char str[MAX_CHAR_LENGTH];
    double ET0;
    double decimalYear;
    sprintf(str, "JD %.17f", this->epochDate);
    str2et_c(str, &ET0);
    timout_c(ET0 + currentTime, "YYYY.#########", 14, str);
//    decimalYear = shuntingYard(str);

    //computeWmmField(decimalYear, phi, lambda, h, B_M);

    //! - convert magnetic field vector into N-frame components and store in output message
    Euler2(phi + 90 * D2R, m33);
    Euler3(lambda, m33_2);
    m33MultM33(m33_2, m33, PM);
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

    v3Scale(pow(10.0, -9.0), B_M, B_M); /* convert nano-Tesla to Tesla */

    return;
}


void MagneticFieldWMM::initializeWmm(const char *dataPath)
{
    char fileName[MAX_CHAR_LENGTH];
    int nMax = 0;
    int nTerms;

    strcpy(fileName, dataPath);
    strcat(fileName, "WMM.cof");
//    MAG_robustReadMagModels(fileName, &magneticModels, this->epochs);
    MAG_robustReadMagModels(fileName, &(this->magneticModels), this->epochs);
    MAG_robustReadMagModels(<#char *filename#>, <#MAGtype_MagneticModel *(*magneticmodels)[]#>, <#int array_size#>);

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

    geoid.GeoidHeightBuffer = GeoidHeightBuffer;
    geoid.Geoid_Initialized = 1;
}
