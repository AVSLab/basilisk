/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
#include "navigation/simple_nav/simple_nav.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the simple nav model.  It sets default variable 
    values and initializes the various parts of the model */
SimpleNav::SimpleNav()
{
    this->inputStateName = "inertial_state_output";
    this->outputNavName = "simple_nav_output";
    this->inputSunName = "sun_planet_data";
    this->crossTrans = false;
    this->crossAtt = false;
    this->outputBufferCount = 2;
    this->inputStateID = -1;
    this->outputDataID = -1;
    this->AMatrix.clear();
    this->PMatrix.clear();
    this->prevTime = 0;
    this->isOutputTruth = false;
    memset(&outState, 0x0, sizeof(NavStateOut));
    return;
}

/*! Destructor.  Nothing here. */
SimpleNav::~SimpleNav()
{
    return;
}

/*! This is the self-init routine for the simple navigation model.  It 
    initializes the various containers used in the model as well as creates the 
    output message.  The error states are allocated as follows:
    Total states: 18
    Position errors [0-2]
    Velocity errors [3-5]
    Attitude errors [6-8]
    Body Rate errors [9-11]
    Sun Point error [12-14]
    Accumulated DV errors [15-17]
    @return void
*/
void SimpleNav::SelfInit()
{
    //! Begin method steps
    uint64_t numStates = 18;
    std::vector<double>::iterator it;
    //! - Create a new message for the output simple nav state data
    outputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage(outputNavName, sizeof(NavStateOut), outputBufferCount,
        "NavStateOut", moduleID);
    //! - Initialize the propagation matrix to default values for use in update
    AMatrix.clear();
    AMatrix.insert(AMatrix.begin(), numStates*numStates, 0.0);
    mSetIdentity(AMatrix.data(), numStates, numStates);
    it = AMatrix.begin();
    //! - Depending on whether we cross-prop position/attitude, set off-diagonal terms
    for(uint32_t i=0; i<3 && (crossTrans || crossAtt); i++)
    {
        it += i + 3;  // Velocity propagation location
        *it = crossTrans ? 1.0 : 0.0; // Scale this by dt in the UpdateState method
        it += 6;
        *it = crossAtt ? 0.25 : 0.0; // MRPs propagate as 1/4 omega when near zero (SAA for errors)
        it += 9 - i;
    }
    //! - Alert the user if the noise matrix was not the right size.  That'd be bad.
    if(PMatrix.size() != numStates*numStates)
    {
        std::cerr << "Your process noise matrix (PMatrix) is not 18*18.";
        std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
        PMatrix.insert(PMatrix.begin()+PMatrix.size(), numStates*numStates - PMatrix.size(),
                       0.0);
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    errorModel.setNoiseMatrix(PMatrix);
    errorModel.setRNGSeed(RNGSeed);
    errorModel.setUpperBounds(walkBounds);
}

/*! This method pulls the input message IDs from the messaging system.  It will 
    alert the user if either of them are not found in the messaging database
    @return void
*/
void SimpleNav::CrossInit()
{
    //! Begin method steps
    //! - Obtain the ID associated with the input state name and alert if not found.
    inputStateID = SystemMessaging::GetInstance()->
    SystemMessaging::GetInstance()->
       subscribeToMessage(inputStateName, sizeof(OutputStateData), moduleID);
    if(inputStateID < 0)
    {
        std::cerr << "Warning: input state message name: " << inputStateName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
    //! - Obtain the ID associated with the input Sun name and alert if not found.
    inputSunID =SystemMessaging::GetInstance()->
    subscribeToMessage(inputSunName, sizeof(SpicePlanetState), moduleID);
    if(inputSunID < 0)
    {
        std::cerr << "Warning: input Sun message name: " << inputSunName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
}

/*! This method reads the input messages associated with the vehicle state and 
 the sun state
 */
void SimpleNav::readInputs()
{
    //! Begin method steps
    SingleMessageHeader localHeader;
    memset(&this->sunState, 0x0, sizeof(SpicePlanetState));
    memset(&this->inertialState, 0x0, sizeof(OutputStateData));
    if(inputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputStateID, &localHeader,
                                                    sizeof(OutputStateData), reinterpret_cast<uint8_t*>(&this->inertialState), moduleID);
    }
    if(inputSunID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputSunID, &localHeader,
                                                    sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&this->sunState), moduleID);
    }
}

/*! This method writes the aggregate nav information into the output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void SimpleNav::writeOutput(uint64_t Clock)
{
    //! Begin method steps
    SystemMessaging::GetInstance()->
    WriteMessage(outputDataID, Clock, sizeof(NavStateOut),
                 reinterpret_cast<uint8_t*> (&outState), moduleID);
}

void SimpleNav::applyErrors()
{
    //! - Add errors to the simple cases (everything except sun-pointing)
    v3Add(outState.r_BN_N, &(navErrors.data()[0]), outState.r_BN_N);
    v3Add(outState.v_BN_N, &(navErrors.data()[3]), outState.v_BN_N);
    addMRP(outState.sigma_BN, &(navErrors.data()[6]), outState.sigma_BN);
    v3Add(outState.omega_BN_B, &(navErrors.data()[9]), outState.omega_BN_B);
    v3Add(outState.vehAccumDV, &(navErrors.data()[15]), outState.vehAccumDV);
}

/*! This method uses the input messages as well as the calculated model errors to 
 compute what the output navigation state should be.
    @return void
    @param Clock The clock time associated with the model's update call
*/
void SimpleNav::computeOutput(uint64_t Clock)
{
    //! Begin method steps
    double sc2SunInrtl[3];
    double T_inrtl2bdy[3][3];
    double T_bdyT2bdyO[3][3];
    
    //! - Set output state to truth data
    v3Copy(inertialState.r_N, outState.r_BN_N);
    v3Copy(inertialState.v_N, outState.v_BN_N);
    v3Copy(inertialState.sigma, outState.sigma_BN);
    v3Copy(inertialState.omega, outState.omega_BN_B);
    v3Copy(inertialState.TotalAccumDVBdy, outState.vehAccumDV);
    
    //! - For the sun pointing output, compute the spacecraft to sun vector, normalize, and trans 2 body.
    v3Subtract(sunState.PositionVector, inertialState.r_N, sc2SunInrtl);
    v3Normalize(sc2SunInrtl, sc2SunInrtl);
    MRP2C(inertialState.sigma, T_inrtl2bdy);
    m33MultV3(T_inrtl2bdy, sc2SunInrtl, outState.vehSunPntBdy);
    MRP2C(&(navErrors.data()[12]), T_bdyT2bdyO);
    m33MultV3(T_bdyT2bdyO, outState.vehSunPntBdy, outState.vehSunPntBdy);
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.
 @return void
 @param CurrentSimNanos The clock time associated with the model call
 */
void SimpleNav::computeErrors(uint64_t CurrentSimNanos)
{
    double timeStep;
    std::vector<double>::iterator it;
    std::vector<double> localProp = AMatrix;
    //! - Compute timestep since the last call
    timeStep = (CurrentSimNanos - this->prevTime)*1.0E-9;
    it = localProp.begin();
    //! - Apply that time step to the pos/vel att/rate cross correlation terms
    for(uint32_t i=0; i<3; i++)
    {
        it += i+3;
        *it *= timeStep;
        it += 6;
        *it *= timeStep;
        it += 9-i;
    }
    //! - Set the GaussMarkov propagation matrix and compute errors
    errorModel.setPropMatrix(localProp);
    errorModel.computeNextState();
    navErrors = errorModel.getCurrentState();
}

/*! This method calls all of the run-time operations for the simple nav model.
    @return void
    @param CurrentSimNanos The clock time associated with the model call
*/
void SimpleNav::UpdateState(uint64_t CurrentSimNanos)
{
    readInputs();
    computeErrors(CurrentSimNanos);
    computeOutput(CurrentSimNanos);
    if (!this->isOutputTruth)
    {
        applyErrors();
    }
    writeOutput(CurrentSimNanos);
    this->prevTime = CurrentSimNanos;
}
