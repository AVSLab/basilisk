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
#include "simulation/navigation/simpleNav/simpleNav.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

/*! This is the constructor for the simple nav model.  It sets default variable
    values and initializes the various parts of the model */
SimpleNav::SimpleNav()
{
    this->crossTrans = false;
    this->crossAtt = false;
    this->prevTime = 0;
    this->estAttState = this->attOutMsg.zeroMsgPayload;
    this->trueAttState = this->attOutMsg.zeroMsgPayload;
    this->estTransState = this->transOutMsg.zeroMsgPayload;
    this->trueTransState = this->transOutMsg.zeroMsgPayload;
    this->PMatrix.resize(18,18);
    this->PMatrix.fill(0.0);
    this->walkBounds.resize(18);
    this->walkBounds.fill(0.0);
    this->errorModel =  GaussMarkov(18, this->RNGSeed);
}

/*! Destructor.  Nothing here. */
SimpleNav::~SimpleNav()
{
    return;
}


/*! This method is used to reset the module. It
 initializes the various containers used in the model as well as creates the
 output message.  The error states are allocated as follows:
 Total states: 18
     - Position errors [0-2]
     - Velocity errors [3-5]
     - Attitude errors [6-8]
     - Body Rate errors [9-11]
     - Sun Point error [12-14]
     - Accumulated DV errors [15-17]

 */
void SimpleNav::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SimpleNav.scStateInMsg was not linked.");
    }

    //! - Initialize the propagation matrix to default values for use in update
    this->AMatrix.setIdentity(this->numStates, this->numStates);
    this->AMatrix(0,3) = this->AMatrix(1,4) = this->AMatrix(2,5) = this->crossTrans ? 1.0 : 0.0;
    this->AMatrix(6,9) = this->AMatrix(7,10) = this->AMatrix(8, 11) = this->crossAtt ? 1.0 : 0.0;

    //! - Alert the user and stop if the noise matrix is the wrong size.  That'd be bad.
    if (this->PMatrix.size() != this->numStates*this->numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 18*18. Size is %ld.  Quitting", this->PMatrix.size());
        return;
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    if (this->walkBounds.size() != this->numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds vector  is not 18 elements. Quitting");
    }
    this->errorModel.setUpperBounds(this->walkBounds);
}


/*! This method reads the input messages associated with the vehicle state and
 the sun state
 */
void SimpleNav::readInputMessages()
{
    this->inertialState = this->scStateInMsg();

    this->sunState = this->sunStateInMsg.zeroMsgPayload;
    if(this->sunStateInMsg.isLinked())
    {
        this->sunState = this->sunStateInMsg();
    }
}

/*! This method writes the aggregate nav information into the output state message.

 @param Clock The clock time associated with the model call
 */
void SimpleNav::writeOutputMessages(uint64_t Clock)
{
    /* time tage the output message */
    this->estAttState.timeTag = (double) Clock * NANO2SEC;
    this->estTransState.timeTag = (double) Clock * NANO2SEC;

    this->attOutMsg.write(&this->estAttState, this->moduleID, Clock);
    this->transOutMsg.write(&this->estTransState, this->moduleID, Clock);
}

void SimpleNav::applyErrors()
{
    //! - Add errors to the simple cases (everything except sun-pointing)
    v3Add(this->trueTransState.r_BN_N, &(this->navErrors.data()[0]), this->estTransState.r_BN_N);
    v3Add(this->trueTransState.v_BN_N, &(this->navErrors.data()[3]), this->estTransState.v_BN_N);
    addMRP(this->trueAttState.sigma_BN, &(this->navErrors.data()[6]), this->estAttState.sigma_BN);
    v3Add(this->trueAttState.omega_BN_B, &(this->navErrors.data()[9]), this->estAttState.omega_BN_B);
    v3Add(this->trueTransState.vehAccumDV, &(this->navErrors.data()[15]), this->estTransState.vehAccumDV);
    //! - Add errors to  sun-pointing
    if(this->sunStateInMsg.isLinked()){
        double dcm_OT[3][3];       /* dcm, body T to body O */
        MRP2C(&(this->navErrors.data()[12]), dcm_OT);
        m33MultV3(dcm_OT, this->trueAttState.vehSunPntBdy, this->estAttState.vehSunPntBdy);
    } else {
        v3SetZero(this->estAttState.vehSunPntBdy);
    }
}


/*! This method uses the input messages as well as the calculated model errors to
 compute what the output navigation state should be.

    @param Clock The clock time associated with the model's update call
*/
void SimpleNav::computeTrueOutput(uint64_t Clock)
{
    //! - Set output state to truth data
    v3Copy(this->inertialState.r_BN_N, this->trueTransState.r_BN_N);
    v3Copy(this->inertialState.v_BN_N, this->trueTransState.v_BN_N);
    v3Copy(this->inertialState.sigma_BN, this->trueAttState.sigma_BN);
    v3Copy(this->inertialState.omega_BN_B, this->trueAttState.omega_BN_B);
    v3Copy(this->inertialState.TotalAccumDVBdy, this->trueTransState.vehAccumDV);

    //! - For the sun pointing output, compute the spacecraft to sun vector, normalize, and trans 2 body.
    if(this->sunStateInMsg.isLinked()){
        double sc2SunInrtl[3];
        double dcm_BN[3][3];        /* dcm, inertial to body */
        v3Subtract(this->sunState.PositionVector, this->inertialState.r_BN_N, sc2SunInrtl);
        v3Normalize(sc2SunInrtl, sc2SunInrtl);
        MRP2C(this->inertialState.sigma_BN, dcm_BN);
        m33MultV3(dcm_BN, sc2SunInrtl, this->trueAttState.vehSunPntBdy);
    } else {
        v3SetZero(this->trueAttState.vehSunPntBdy);
    }
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.

 @param CurrentSimNanos The clock time associated with the model call
 */
void SimpleNav::computeErrors(uint64_t CurrentSimNanos)
{
    double timeStep;
    Eigen::MatrixXd localProp = this->AMatrix;
    //! - Compute timestep since the last call
    timeStep = (CurrentSimNanos - this->prevTime)*1.0E-9;

    localProp(0,3) *= timeStep; //postion/velocity cross correlation terms
    localProp(1,4) *= timeStep; //postion/velocity cross correlation terms
    localProp(2,5) *= timeStep; //postion/velocity cross correlation terms
    localProp(6,9) *= timeStep; //attitude/attitude rate cross correlation terms
    localProp(7,10) *= timeStep; //attitude/attitude rate cross correlation terms
    localProp(8,11) *= timeStep; //attitude/attitude rate cross correlation terms

    //! - Set the GaussMarkov propagation matrix and compute errors
    if (this->PMatrix.size() != this->numStates*this->numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 18*18. Size is %ld.  Quitting during Simulation", this->PMatrix.size());
        return;
    }
    this->errorModel.setNoiseMatrix(this->PMatrix);
    if (this->walkBounds.size() != this->numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds vector  is not 18 elements. Quitting during Simulation");
    }
    this->errorModel.setUpperBounds(this->walkBounds);
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

/*! This method calls all of the run-time operations for the simple nav model.

    @param CurrentSimNanos The clock time associated with the model call
*/
void SimpleNav::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->computeTrueOutput(CurrentSimNanos);
    this->computeErrors(CurrentSimNanos);
    this->applyErrors();
    this->writeOutputMessages(CurrentSimNanos);
    this->prevTime = CurrentSimNanos;
}
