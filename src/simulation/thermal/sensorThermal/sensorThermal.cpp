#include <math.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include "sensorThermal.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/avsEigenMRP.h"

SensorThermal::SensorThermal(){

    this->shadowFactor = 1;

    // Set the required parameters from the constructor
    this->nHat_B.setZero();
    this->sensorArea = -1;
    this->sensorAbsorptivity = -1;
    this->sensorEmissivity = -1;

    // Initialize the optional parameters
    this->sensorMass = 1; //! - Default to 1 kg
    this->sensorSpecificHeat = 890; //! - Specific heat of aluminum
    this->T_0 = 30; //! - Initial temperature of 30 deg C
    this->sensorPowerDraw = 0.0; //! - Assuming 0 power draw
    this->S = 1366; //! - Solar constant at 1AU
    this->boltzmannConst = 5.76051e-8;  //! -  Boltzmann constant
    this->CurrentSimSecondsOld = 0;

    // Initialize fault parameers
    this->senNoiseStd = 0.0;
    this->senBias = 0.0;
    this->walkBounds = 1E-15; //don't allow random walk by default
    this->noiseModel = GaussMarkov(1, this->RNGSeed);
    this->faultState = TEMP_FAULT_NOMINAL;
    this->pastValue = 0.0;
    this->stuckValue = 0.0;
    this->spikeProbability = 0.1;
    this->spikeAmount = 2.0;

    return;

}

SensorThermal::~SensorThermal(){

    return;
}

/*! Thermal sensor reset function
 */
void SensorThermal::Reset(uint64_t CurrentClock) {

    this->shadowFactor = 1.0;

    if (this->sensorArea <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The sensorArea must be a positive value");
    }
    if (this->sensorMass <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The sensorMass must be a positive value");
    }
    if (this->sensorSpecificHeat <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The sensorSpecificHeat must be a positive value");
    }
    if (this->sensorAbsorptivity <= 0.0 || this->sensorAbsorptivity > 1.0) {
        bskLogger.bskLog(BSK_ERROR, "The sensor absorptivity must be between 0 and 1");
    }
    if (this->sensorEmissivity <= 0.0 || this->sensorEmissivity > 1.0) {
        bskLogger.bskLog(BSK_ERROR, "The sensor emissivity must be between 0 and 1");
    }
    if (this->nHat_B.norm() > 0.1) {
        this->nHat_B.normalize();
    } else {
        bskLogger.bskLog(BSK_ERROR, "The nHat_B must be set to a non-zero vector");
    }
    if (this->spikeProbability > 1.0 || this->spikeProbability < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "The probability of temperature spike on fault must be between 0 and 1.");
    }
    // check if required input messages are connected
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "sensorThermal.sunInMsg was not linked.");
    }
    if (!this->stateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "sensorThermal.stateInMsg was not linked.");
    }

    Eigen::VectorXd nMatrix;
    Eigen::VectorXd pMatrix;
    Eigen::VectorXd bounds;
    nMatrix.resize(1,1);
    pMatrix.resize(1,1);
    bounds.resize(1,1);
    
    this->noiseModel.setRNGSeed(this->RNGSeed);
    
    nMatrix(0,0) = this->senNoiseStd*1.5;
    this->noiseModel.setNoiseMatrix(nMatrix);
    
    bounds(0,0) = this->walkBounds;
    this->noiseModel.setUpperBounds(bounds);
    
    pMatrix(0,0) = 1.;
    this->noiseModel.setPropMatrix(pMatrix);


    this->trueT = this->T_0;

    this->spikeProbabilityGenerator.seed((unsigned int)this->RNGSeed);
    //this->spikeProbabilityDistribution.a = 0.0;
    //this->spikeProbabilityDistribution.b = 1.0;

    return;
}

bool SensorThermal::readMessages()
{
    //! - Zero ephemeris information
    this->sunData = sunInMsg.zeroMsgPayload;
    this->stateCurrent = stateInMsg.zeroMsgPayload;

    //! - If we have a valid sun ID, read Sun ephemeris message
    if(this->sunInMsg.isLinked())
    {
        this->sunData = this->sunInMsg();
    }
    //! - If we have a valid state ID, read vehicle state ephemeris message
    if(this->stateInMsg.isLinked())
    {
        this->stateCurrent = this->stateInMsg();
    }
    //! - Read in optional sun eclipse input message 
    if(this->sunEclipseInMsg.isLinked()) {
        EclipseMsgPayload sunVisibilityFactor;          // sun visiblity input message
        sunVisibilityFactor = this->sunEclipseInMsg();
        this->shadowFactor = sunVisibilityFactor.shadowFactor;
    }

    DeviceStatusMsgPayload statusMsg;

    //! - read in the device status msg
    if(this->sensorStatusInMsg.isLinked())
    {
        statusMsg = this->sensorStatusInMsg();
        this->sensorStatus = statusMsg.deviceStatus;
    } else {
        this->sensorStatus = 1;
    }

    return(true);
}

/*! Provides logic for running the read / compute / write operation that is the module's function.
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void SensorThermal::UpdateState(uint64_t CurrentSimNanos)
{

    //! - Only update the thermal status if we were able to read in messages.
    if(this->readMessages())
    {
        this->evaluateThermalModel(CurrentSimNanos*NANO2SEC);
    } else {
        /* if the read was not successful then zero the output message */
        this->temperatureMsgBuffer = this->temperatureOutMsg.zeroMsgPayload;
    }

    this->writeMessages(CurrentSimNanos);

    return;
}

/*! This method writes out a message.
 @return void
 */
void SensorThermal::writeMessages(uint64_t CurrentClock)
{
    //! - write temperature output message
    this->temperatureOutMsg.write(&this->temperatureMsgBuffer, this->moduleID, CurrentClock);

    return;
}

/*! This method computes the spacecraft-sun vector, the sensor's projected area, and the sunDistanceFactor based on the magnitude of the spacecraft sun vector.
 @return void
 */
void SensorThermal::computeSunData()
{
    Eigen::Vector3d r_SB_N;         //!< [m] Sun position relative to spacecraft body B
    Eigen::Vector3d sHat_N;         //!< [] unit sun heading vector in inertial frame N
    Eigen::Matrix3d dcm_BN;         //!< [] DCM from inertial N to body B

    Eigen::Vector3d r_BN_N;         //!< [m] spacecraft B position relative to sun S
    Eigen::Vector3d r_SN_N;         //!< [m] sun position relative to inertial
    Eigen::MRPd sigma_BN;           //!< [] MRP attitude of body relative to inertial
    Eigen::Vector3d sHat_B;         //!< [] unit Sun heading vector relative to the spacecraft in B frame.

    //! - Read Message data to eigen
    r_BN_N = cArray2EigenVector3d(this->stateCurrent.r_BN_N);
    r_SN_N = cArray2EigenVector3d(this->sunData.PositionVector);
    sigma_BN = cArray2EigenVector3d(this->stateCurrent.sigma_BN);

    //! - Find sun heading unit vector
    r_SB_N = r_SN_N - r_BN_N;
    sHat_N = r_SB_N.normalized();

    //! - Get the inertial to body frame transformation information and convert sHat to body frame
    dcm_BN = sigma_BN.toRotationMatrix().transpose();
    sHat_B = dcm_BN * sHat_N;

    //! - Compute the sensor projected area
    this->projectedArea = this->sensorArea * (sHat_B.dot(this->nHat_B));
    if(this->projectedArea<0){
        this->projectedArea = 0;
    }
}

void SensorThermal::applySensorErrors()
{
    // apply noise and bias
    double sensorError;
    if(this->senNoiseStd <= 0.0){
        sensorError = this->senBias;
    } else {
        // get current error from gaussMarkov random number generator
        this->noiseModel.computeNextState();
        Eigen::VectorXd currentErrorEigen = this->noiseModel.getCurrentState();
        double sensorNoise = currentErrorEigen.coeff(0,0);
        sensorError = this->senBias + sensorNoise;
    }

    this->sensedT = this->trueT + sensorError;
        
    if(this->faultState == TEMP_FAULT_STUCK_VALUE){ // stuck at specified value
        this->sensedT = this->stuckValue;
    } else if (this->faultState == TEMP_FAULT_STUCK_CURRENT){ // stuck at last value before flag turned on
        this->sensedT = this->pastValue; 
    } else if (this->faultState == TEMP_FAULT_SPIKING){ // spiking periodically with specified probability
        // have to make a new distribution every time because SWIG can't parse putting this in the H file....?
        std::uniform_real_distribution<double> spikeProbabilityDistribution(0.0,1.0); 
        double n = spikeProbabilityDistribution(spikeProbabilityGenerator); // draw from uniform distribution
        if (n <= this->spikeProbability) { // if drawn number within probability of spiking
            this->sensedT = this->sensedT*this->spikeAmount;
        }
    } else { // Nominal

    }

    this->pastValue = this->sensedT; // update past value
    
}
/*! This method evaluates the thermal model. This is evaluated in five steps:
1. Computing the projected area of the sensor exposed to the sun using this->computeSunData();
2. Computing the incoming thermal power from the sun
3. Computing the thermal power radiated to the environment
4. Computing the change in temperature
5. Computing the current temperature based on the change in temperature
 @return void
 */
void SensorThermal::evaluateThermalModel(uint64_t CurrentSimSeconds) {

    //! - Compute the sun data
    this->computeSunData();

    //! - Compute Q_in
    this->Q_in = this->shadowFactor * this->S * this->projectedArea * this->sensorAbsorptivity + this->sensorPowerDraw * this->sensorStatus;

    //! - Compute Q_out
    this->Q_out = this->sensorArea * this->sensorEmissivity * this->boltzmannConst * pow((this->trueT + 273.15), 4);

    //! - Compute change in energy using Euler integration
    double dT = (this->Q_in - this->Q_out)/(this->sensorSpecificHeat*this->sensorMass);

    //! - Compute the current temperature
    this->trueT = this->trueT + dT*(CurrentSimSeconds - this->CurrentSimSecondsOld);

    //! - Set the old CurrentSimSeconds to the current timestep
    this->CurrentSimSecondsOld = CurrentSimSeconds;

    //! - Apply sensor errors
    this->applySensorErrors();

    //! - Write to the message buffer
    this->temperatureMsgBuffer.temperature = this->sensedT;

    return;
}

