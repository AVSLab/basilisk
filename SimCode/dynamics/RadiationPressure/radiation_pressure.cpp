#include "dynamics/RadiationPressure/radiation_pressure.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

/*! This is the constructor.  It sets some defaul initializers that can be
 overriden by the user.*/
RadiationPressure::RadiationPressure()
    :area(0.0f)
    ,coefficientReflection(1.2)
    ,sunEphmInMsgName("sun_planet_data")
    ,useCannonballModel(true)
    ,sunEphmInMsgID(-1)
{
    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
RadiationPressure::~RadiationPressure()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firings
 @return void
 */
void RadiationPressure::SelfInit()
{
}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void RadiationPressure::CrossInit()
{
    //! Begin method steps
    //! - Find the message ID associated with the ephmInMsgID string.
    //! - Warn the user if the message is not successfully linked.
    sunEphmInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(sunEphmInMsgName,
    sizeof(SpicePlanetState), moduleID);
 
    if(sunEphmInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
        std::cerr << sunEphmInMsgName << "  :" << __FILE__ << std::endl;
    }
}

void RadiationPressure::linkInStates(DynParamManager& statesIn)
{
    this->hubPosition = statesIn.getStateObject("hubPosition");
    this->hubSigma = statesIn.getStateObject("hubSigma");
}

/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void RadiationPressure::writeOutputMessages(uint64_t currentClock)
{
    
}

/*! This method is used to read the incoming ephmeris message and set the
 associated buffer structure.
 @return void
 */
void RadiationPressure::readInputMessages()
{
    bool dataGood;
    //! Begin method steps
    //! - Zero the command buffer and read the incoming command array
    SingleMessageHeader localHeader;
    memset(&localHeader, 0x0, sizeof(localHeader));
    
    if(sunEphmInMsgID >= 0)
    {
        memset(&sunEphmInBuffer, 0x0, sizeof(SpicePlanetState));
        dataGood = SystemMessaging::GetInstance()->ReadMessage(sunEphmInMsgID, &localHeader,
                                                sizeof(SpicePlanetState),
                                                reinterpret_cast<uint8_t*> (&sunEphmInBuffer));
    }
}

/*! This method is used to compute all the dynamical effects for the thruster set.
 It is an inherited method from the DynEffector class and is designed to be called
 by the dynamics plant for the simulation.  It uses the thruster force magnitude
 computed for the current time as well as the current vehicle state and mass
 properties to get the current body force/torque which serve as the API to
 dynamics
 @return void
 @param Props Current mass properties of the vehicle (using center of mass and str2bdy transformation
 @param Bstate Current state of the vehicle (not used by thruster dynamics)
 @param CurrentTime Current simulation time converted to double precision
 */
void RadiationPressure::computeBodyForceTorque(uint64_t currentTime)
{
    //! Begin method steps
    //! - Zero out the structure force/torque for the thruster set
    this->forceExternal_N.setZero();
    Eigen::Vector3d s_B(0, 0, 0); // (m)
    Eigen::MatrixXd r_N = this->hubPosition->getState();
    Eigen::Vector3d sun_r_N(sunEphmInBuffer.PositionVector);
    s_B = r_N - sun_r_N;
    
    if (useCannonballModel) {
        this->computeCannonballModel(s_B);
    } else {
        this->computeLookupModel(s_B);
    }
}

void RadiationPressure::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
}

/*! Sets the model to use in computing the solar radiation force
 @return void
 @param use True is cannonball model, False to use lookup table model
 */
void RadiationPressure::setUseCannonballModel(bool use)
{
    this->useCannonballModel = use;
}

/*! Computes the solar radiation force vector
 *   based on cross-sectional Area and mass of the spacecraft
 *   and the position vector of the spacecraft to the sun.
 *   Note: The solar radiation pressure decreases
 *   quadratically with distance from sun (in AU)
 *
 *   Solar Radiation Equations obtained from
 *   Earth Space and Planets Journal Vol. 51, 1999 pp. 979-986
 @return void
 @param s_B (m) Position vector to the Sun relative to the body frame
 */
void RadiationPressure::computeCannonballModel(Eigen::Vector3d s_B)
{
    /* Magnitude of position vector */
    double sunDist = s_B.norm();
    Eigen::Vector3d sHat_B = s_B*s_B.norm();
    /* Computing the force vector (N)*/
    double scaleFactor = (-this->coefficientReflection * this->area * SOLAR_FLUX_EARTH * pow(AU*1000.,2)) / (SPEED_LIGHT * pow(sunDist, 3));
    
    this->forceExternal_B = scaleFactor*(sHat_B);
}

/*! Computes the solar radiation force vector
 *   using a lookup table given the current spacecraft attitude
 *   and the position vector of the spacecraft to the sun.
 *   Note: It is assumed that the solar radiation pressure decreases
 *   quadratically with distance from sun (in AU)
 *
 @return void
 @param s_B (m) Position vector to the Sun relative to the body frame
 */
void RadiationPressure::computeLookupModel(Eigen::Vector3d s_B)
{
    double tmpDotProduct = 0;
    double currentDotProduct = 0;
    double currentIdx = 0;
    Eigen::Vector3d sHat_B = s_B*s_B.norm();
    Eigen::Vector3d tmpLookupSHat_B(0,0,0);
    
    // Find the lookup entry that most closely aligns with the current sHat_B direction
    // Save the index for later use in looking up the force and torque for that attitude
    for(int i = 0; i < lookupSHat_B.size(); i++) {
        tmpLookupSHat_B << this->lookupSHat_B[i].at(0), this->lookupSHat_B[i].at(1), this->lookupSHat_B[i].at(2);
        tmpDotProduct = tmpLookupSHat_B.dot(sHat_B);
        if (tmpDotProduct > currentDotProduct)
        {
            currentIdx = i;
            currentDotProduct = tmpDotProduct;
        }
    }

    // Look up force is expected to be evaluated at 1AU.
    // Therefore, we must scale the force by its distance from the sun squared.
    this->forceExternal_B = Eigen::Vector3d(lookupForce_B[currentIdx].data())*pow(AU/(s_B.norm()/1000), 2);
    this->torqueExternalPntB_B = Eigen::Vector3d(lookupTorque_B[currentIdx].data());
}
