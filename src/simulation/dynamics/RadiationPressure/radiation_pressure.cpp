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

#include <iostream>
#include "dynamics/RadiationPressure/radiation_pressure.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/avsEigenSupport.h"
#include "utilities/avsEigenMRP.h"
#include "utilities/bsk_Print.h"

/*! This is the constructor.  It sets some default initializers that can be
 overriden by the user.*/
RadiationPressure::RadiationPressure()
    :area(0.0f)
    ,coefficientReflection(1.2)
    ,sunEphmInMsgName("sun_planet_data")
    ,stateInMsgName("inertial_state_output")
    ,srpModel(SRP_CANNONBALL_MODEL)
    ,sunEphmInMsgId(-1)
    ,stateInMsgId(-1)
    ,stateRead(false)
{
    this->sunEclipseInMsgId = -1;
    this->sunVisibilityFactor.shadowFactor = 1.0;
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
RadiationPressure::~RadiationPressure()
{
    return;
}

/*! This method is used to create any messages published by this module.
 @return void
 */
void RadiationPressure::SelfInit()
{
}

/*! This method is used to subscribe to systems message from this module.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void RadiationPressure::CrossInit()
{
    //! - Find the message ID associated with the ephmInMsgID string.
    //! - Warn the user if the message is not successfully linked.
    this->sunEphmInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEphmInMsgName, sizeof(SpicePlanetStateSimMsg), this->moduleID);
 
    if(sunEphmInMsgId < 0)
    {
        BSK_PRINT(MSG_WARNING, "Did not find a valid sun ephemeris message with name: %s", this->sunEphmInMsgName.c_str());
    }
    
    this->stateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->stateInMsgName, sizeof(SCPlusStatesSimMsg), this->moduleID);
    
    if(this->stateInMsgId < 0)
    {
        BSK_PRINT(MSG_WARNING, "Did not find a valid state input message with name: %lld", this->stateInMsgId);
    }

    /* reading in the sun eclipse message is optional.  It only gets used if this message is successfully suscribed.  */
    if (this->sunEclipseInMsgName.length() > 0) {
        this->sunEclipseInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEclipseInMsgName,
                                                                                     sizeof(EclipseSimMsg), moduleID);
    }
}

/*! This method retrieves pointers to parameters/data stored
 in the dynamic parameter manager
 @return void
 @param statesIn Dynamic parameter manager
 */
void RadiationPressure::linkInStates(DynParamManager& statesIn)
{
}

/*! This method is used to read the incoming ephmeris and 
 spacecraft state messages. The data is stored in the associated
 buffer structure.
 @return void
 */
void RadiationPressure::readInputMessages()
{
    bool succesfulRead;
    //! Begin method steps
    //! - Zero the command buffer and read the incoming command array
    SingleMessageHeader localHeader;
    memset(&localHeader, 0x0, sizeof(localHeader));
    
    if(this->sunEphmInMsgId >= 0)
    {
        memset(&this->sunEphmInBuffer, 0x0, sizeof(SpicePlanetStateSimMsg));
        succesfulRead = SystemMessaging::GetInstance()->ReadMessage(this->sunEphmInMsgId, &localHeader, sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*> (&this->sunEphmInBuffer));
    }
    
    memset(&localHeader, 0x0, sizeof(localHeader));
    this->stateRead = false;
    if(this->stateInMsgId >= 0)
    {
        memset(&this->stateInBuffer, 0x0, sizeof(SCPlusStatesSimMsg));
        this->stateRead = SystemMessaging::GetInstance()->ReadMessage(this->stateInMsgId, &localHeader, sizeof(SCPlusStatesSimMsg), reinterpret_cast<uint8_t*> (&this->stateInBuffer));
    }

    memset(&localHeader, 0x0, sizeof(localHeader));
    if(this->sunEclipseInMsgId >= 0) {
        SystemMessaging::GetInstance()->ReadMessage(this->sunEclipseInMsgId, &localHeader,
                                                    sizeof(EclipseSimMsg), reinterpret_cast<uint8_t*> (&this->sunVisibilityFactor), moduleID);
    }
}

/*! This method computes the dynamic effect due to solar raidation pressure.
 It is an inherited method from the DynamicEffector class and 
 is designed to be called by the simulation dynamics engine.
 @return void
 @param integTime Current simulation integration time
 */
void RadiationPressure::computeForceTorque(double integTime)
{
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    Eigen::Vector3d r_N(this->stateInBuffer.r_BN_N);
    Eigen::Vector3d sun_r_N(this->sunEphmInBuffer.PositionVector);
    Eigen::Vector3d s_N = sun_r_N - r_N;
    
    if (this->srpModel == SRP_CANNONBALL_MODEL) {
        this->computeCannonballModel(s_N);
        this->forceExternal_N = this->forceExternal_N * this->sunVisibilityFactor.shadowFactor;
    }
    else if (this->srpModel == SRP_FACETED_CPU_MODEL) {
        Eigen::MRPd sigmaLocal_BN(this->stateInBuffer.sigma_BN);
        Eigen::Matrix3d dcmLocal_BN = sigmaLocal_BN.toRotationMatrix().transpose();
        Eigen::Vector3d s_B = dcmLocal_BN*(sun_r_N - r_N);
        this->computeLookupModel(s_B);
        this->forceExternal_B = this->forceExternal_B * this->sunVisibilityFactor.shadowFactor;
        this->torqueExternalPntB_B = this->torqueExternalPntB_B * this->sunVisibilityFactor.shadowFactor;
    } else {
        BSK_PRINT(MSG_ERROR,"Requested SRF Model not implemented.\n");
    }
}

/*! Update model state by reading in new message data
 @return void
 @param CurrentSimNanos current simulation time in nanoseconds
 */
void RadiationPressure::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}

/*! Sets the model to the cannonball model in computing the solar radiation force
 @return void
 */
void RadiationPressure::setUseCannonballModel()
{
    this->srpModel = SRP_CANNONBALL_MODEL;
}

/*! Sets the model to the faceted table-lookup model, evaluted on the CPU, in computing the solar radiation force
 @return void
 */
void RadiationPressure::setUseFacetedCPUModel()
{
    this->srpModel = SRP_FACETED_CPU_MODEL;
}


/*! Computes the solar radiation force vector
 *   based on cross-sectional Area and mass of the spacecraft
 *   and the position vector of the spacecraft to the sun.
 *   The solar radiation pressure decreases
 *   quadratically with distance from sun (in AU)
 *
 *   Solar Radiation Equations obtained from
 *   Earth Space and Planets Journal Vol. 51, 1999 pp. 979-986
 @return void
 @param s_N (m) Position vector to the Sun relative to the inertial frame
 */
void RadiationPressure::computeCannonballModel(Eigen::Vector3d s_N)
{
    /* Magnitude of sun vector in the body frame */
    double sunDist = s_N.norm();
    /* Computing the force vector [N]*/
    double scaleFactor = (-this->coefficientReflection * this->area * SOLAR_FLUX_EARTH * pow(AU*1000.,2)) / (SPEED_LIGHT * pow(sunDist, 3));
    if (stateRead)
        this->forceExternal_N = scaleFactor*(s_N);
    else
        this->forceExternal_N.setZero();
}

/*! Computes the solar radiation force vector
 *   using a lookup table given the current spacecraft attitude
 *   and the position vector of the spacecraft to the sun.
 *   It is assumed that the lookup table has been generated
 *   with a solar flux at 1AU. Force and torque values are scaled.
 *
 @return void
 @param s_B (m) Position vector of the Sun relative to the body frame
 */
void RadiationPressure::computeLookupModel(Eigen::Vector3d s_B)
{
    double tmpDotProduct = 0;
    double currentDotProduct = 0;
    double currentIdx = 0;
    double sunDist = s_B.norm();
    Eigen::Vector3d sHat_B = s_B/sunDist;
    Eigen::Vector3d tmpLookupSHat_B(0,0,0);
    
    if (!this->stateRead) {
        this->forceExternal_B.setZero();
        this->torqueExternalPntB_B.setZero();
        return;
    }
    
    // Find the lookup entry that most closely aligns with the current sHat_B direction
    // Look up force is expected to be evaluated at 1AU.
    // Therefore, we must scale the force by its distance from the sun squared.
    // @TODO: this lookup search should be optimized, possibly by saving the
    // index for later use and generate lookup table as azimuth and elevation
    // because then we can use a simple gradient decent search to find the nearest next attitude
    for(int i = 0; i < this->lookupSHat_B.size(); i++) {
        tmpLookupSHat_B = this->lookupSHat_B[i];
        tmpDotProduct = tmpLookupSHat_B.dot(sHat_B);
        if (tmpDotProduct > currentDotProduct)
        {
            currentIdx = i;
            currentDotProduct = tmpDotProduct;
        }
    }
    
    this->forceExternal_B = this->lookupForce_B[currentIdx]*pow(AU*1000/sunDist, 2);
    this->torqueExternalPntB_B = this->lookupTorque_B[currentIdx]*pow(AU*1000/sunDist, 2);
}

/*! Add force vector in the body frame to lookup table.
 *
 @return void
 @param vec (N) Force vector for particular attitude in lookup table
 */
void RadiationPressure::addForceLookupBEntry(Eigen::Vector3d vec)
{
    this->lookupForce_B.push_back(vec);
}

/*! Add torque vector to lookup table.
 *
 @return void
 @param vec (Nm) Torque vector for particular attitude in lookup table
 */
void RadiationPressure::addTorqueLookupBEntry(Eigen::Vector3d vec)
{
    this->lookupTorque_B.push_back(vec);
}

/*! Add sun unit direction vector in body frame to lookup table.
 *
 @return void
 @param vec sun unit direction vector in body frame
 */
void RadiationPressure::addSHatLookupBEntry(Eigen::Vector3d vec)
{
    this->lookupSHat_B.push_back(vec);
}
