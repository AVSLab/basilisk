/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
    this->outputAttName = "simple_att_nav_output";
    this->outputTransName = "simple_trans_nav_output";
    this->inputSunName = "sun_planet_data";
    this->crossTrans = false;
    this->crossAtt = false;
    this->outputBufferCount = 2;
    this->inputStateID = -1;
    this->outputAttID = -1;
    this->outputTransID = -1;
    this->AMatrix.clear();
    this->PMatrix.clear();
    this->prevTime = 0;
    memset(&estAttState, 0x0, sizeof(NavAttIntMsg));
    memset(&trueAttState, 0x0, sizeof(NavAttIntMsg));
    memset(&estTransState, 0x0, sizeof(NavTransIntMsg));
    memset(&trueTransState, 0x0, sizeof(NavTransIntMsg));
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
    outputAttID = SystemMessaging::GetInstance()->
        CreateNewMessage(outputAttName, sizeof(NavAttIntMsg), outputBufferCount,
        "NavAttIntMsg", moduleID);
    outputTransID = SystemMessaging::GetInstance()->
    CreateNewMessage(outputTransName, sizeof(NavTransIntMsg), outputBufferCount,
                     "NavTransIntMsg", moduleID);

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
    if (PMatrix.size() == 0) {
        PMatrix.insert(PMatrix.begin(), numStates*numStates, 0.0);
    }
    else if(PMatrix.size() != numStates*numStates)
    {
        std::cerr << "Your process noise matrix (PMatrix) is not 18*18.";
        std::cerr << "  You should fix that.  Popping zeros onto end"<<std::endl;
        PMatrix.insert(PMatrix.begin()+PMatrix.size(), numStates*numStates - PMatrix.size(),
                       0.0);
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    errorModel.setNoiseMatrix(PMatrix);
    errorModel.setRNGSeed(RNGSeed);
    if (this->walkBounds.size() == 0) {
        walkBounds.insert(walkBounds.begin(), numStates, 0.0);
    }
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
       subscribeToMessage(inputStateName, sizeof(SCPlusStatesMessage), moduleID);
    if(inputStateID < 0)
    {
        std::cerr << "Warning: input state message name: " << inputStateName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
    //! - Obtain the ID associated with the input Sun name and alert if not found.
    inputSunID =SystemMessaging::GetInstance()->
    subscribeToMessage(inputSunName, sizeof(SpicePlanetStateMessage), moduleID);
    if(inputSunID < 0)
    {
        std::cerr << "Warning: input Sun message name: " << inputSunName;
        std::cerr << " could not be isolated, message disabled" << std::endl;
    }
}

/*! This method reads the input messages associated with the vehicle state and 
 the sun state
 */
void SimpleNav::readInputMessages()
{
    //! Begin method steps
    SingleMessageHeader localHeader;
    memset(&this->sunState, 0x0, sizeof(SpicePlanetStateMessage));
    memset(&this->inertialState, 0x0, sizeof(SCPlusStatesMessage));
    if(inputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputStateID, &localHeader,
                                                    sizeof(SCPlusStatesMessage), reinterpret_cast<uint8_t*>(&this->inertialState), moduleID);
    }
    if(inputSunID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(inputSunID, &localHeader,
                                                    sizeof(SpicePlanetStateMessage), reinterpret_cast<uint8_t*>(&this->sunState), moduleID);
    }
}

/*! This method writes the aggregate nav information into the output state message.
 @return void
 @param Clock The clock time associated with the model call
 */
void SimpleNav::writeOutputMessages(uint64_t Clock)
{
    //! Begin method steps
    SystemMessaging::GetInstance()->
    WriteMessage(outputAttID, Clock, sizeof(NavAttIntMsg),
                 reinterpret_cast<uint8_t*> (&estAttState), moduleID);
    SystemMessaging::GetInstance()->
    WriteMessage(outputTransID, Clock, sizeof(NavTransIntMsg),
                 reinterpret_cast<uint8_t*> (&estTransState), moduleID);
}

void SimpleNav::applyErrors()
{
    //! - Add errors to the simple cases (everything except sun-pointing)
    v3Add(trueTransState.r_BN_N, &(navErrors.data()[0]), estTransState.r_BN_N);
    v3Add(trueTransState.v_BN_N, &(navErrors.data()[3]), estTransState.v_BN_N);
    addMRP(trueAttState.sigma_BN, &(navErrors.data()[6]), estAttState.sigma_BN);
    v3Add(trueAttState.omega_BN_B, &(navErrors.data()[9]), estAttState.omega_BN_B);
    v3Add(trueTransState.vehAccumDV, &(navErrors.data()[15]), estTransState.vehAccumDV);
    //! - Add errors to  sun-pointing
    if(inputSunID >= 0) {
        double dcm_OT[3][3];       /* dcm, body T to body O */
        MRP2C(&(navErrors.data()[12]), dcm_OT);
        m33MultV3(dcm_OT, trueAttState.vehSunPntBdy, estAttState.vehSunPntBdy);
    }
}


/*! This method uses the input messages as well as the calculated model errors to
 compute what the output navigation state should be.
    @return void
    @param Clock The clock time associated with the model's update call
*/
void SimpleNav::computeTrueOutput(uint64_t Clock)
{
    //! - Set output state to truth data
    v3Copy(inertialState.r_BN_N, trueTransState.r_BN_N);
    v3Copy(inertialState.v_BN_N, trueTransState.v_BN_N);
    v3Copy(inertialState.sigma_BN, trueAttState.sigma_BN);
    v3Copy(inertialState.omega_BN_B, trueAttState.omega_BN_B);
    v3Copy(inertialState.TotalAccumDVBdy, trueTransState.vehAccumDV);

    //! - For the sun pointing output, compute the spacecraft to sun vector, normalize, and trans 2 body.
    if(inputSunID >= 0) {
        double sc2SunInrtl[3];
        double dcm_BN[3][3];        /* dcm, inertial to body */
        v3Subtract(sunState.PositionVector, inertialState.r_BN_N, sc2SunInrtl);
        v3Normalize(sc2SunInrtl, sc2SunInrtl);
        MRP2C(inertialState.sigma_BN, dcm_BN);
        m33MultV3(dcm_BN, sc2SunInrtl, trueAttState.vehSunPntBdy);
    }
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
    readInputMessages();
    computeTrueOutput(CurrentSimNanos);
    computeErrors(CurrentSimNanos);
    applyErrors();
    writeOutputMessages(CurrentSimNanos);
    this->prevTime = CurrentSimNanos;
}
