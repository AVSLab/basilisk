/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/navigation/planetNav/planetNav.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include <cstring>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
PlanetNav::PlanetNav()
{
    this->crossTrans = false;
    this->crossAtt = false;
    this->prevTime = 0;
    this->noisePlanetState = this->ephemerisOutMsg.zeroMsgPayload;
    this->truePlanetState = this->ephemerisOutMsg.zeroMsgPayload;
    this->PMatrix.resize(12,12);
    this->PMatrix.fill(0.0);
    this->walkBounds.resize(12);
    this->walkBounds.fill(0.0);
    this->errorModel =  GaussMarkov(12, this->RNGSeed);
}

/*! Module Destructor */
PlanetNav::~PlanetNav()
{
    return;
}

/*! This method is used to reset the module and checks that required input messages are connect.

    @param CurrentSimNanos The clock time associated with the module call
*/
void PlanetNav::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "PlanetNav.ephemerisInMsg was not linked.");
    }

    int64_t numStates = 12;

    //! - Initialize the propagation matrix to default values for use in update
    this->AMatrix.setIdentity(numStates, numStates);
    this->AMatrix(0,3) = this->AMatrix(1,4) = this->AMatrix(2,5) = this->crossTrans ? 1.0 : 0.0;
    this->AMatrix(6,9) = this->AMatrix(7,10) = this->AMatrix(8, 11) = this->crossAtt ? 1.0 : 0.0;

    //! - Alert the user and stop if the noise matrix is the wrong size.  That'd be bad.
    if (this->PMatrix.size() != numStates*numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your process noise matrix (PMatrix) is not 12*12. Size is %ld.  Quitting", this->PMatrix.size());
        return;
    }
    //! - Set the matrices of the lower level error propagation (GaussMarkov)
    this->errorModel.setNoiseMatrix(this->PMatrix);
    this->errorModel.setRNGSeed(this->RNGSeed);
    if (this->walkBounds.size() != numStates) {
        bskLogger.bskLog(BSK_ERROR, "Your walkbounds vector  is not 12 elements. Quitting");
    }
    this->errorModel.setUpperBounds(this->walkBounds);

}

/*! This method reads the input messages associated with the planet state
 */
void PlanetNav::readInputMessages()
{
    this->truePlanetState = this->ephemerisInMsg();
}

/*! This method writes the aggregate nav information into the output state message.

 @param CurrentSimNanos The clock time associated with the model call
 */
void PlanetNav::writeOutputMessages(uint64_t CurrentSimNanos)
{
    this->noisePlanetState.timeTag = (double) CurrentSimNanos * NANO2SEC;
    this->ephemerisOutMsg.write(&this->noisePlanetState, this->moduleID, CurrentSimNanos);
}

/*! This method applies the errors to the truePlanetState

 */
void PlanetNav::applyErrors()
{
    //! - Add errors
    v3Add(this->truePlanetState.r_BdyZero_N, &(this->navErrors.data()[0]), this->noisePlanetState.r_BdyZero_N);
    v3Add(this->truePlanetState.v_BdyZero_N, &(this->navErrors.data()[3]), this->noisePlanetState.v_BdyZero_N);
    addMRP(this->truePlanetState.sigma_BN, &(this->navErrors.data()[6]), this->noisePlanetState.sigma_BN);
    v3Add(this->truePlanetState.omega_BN_B, &(this->navErrors.data()[9]), this->noisePlanetState.omega_BN_B);
}

/*! This method sets the propagation matrix and requests new random errors from
 its GaussMarkov model.

 @param CurrentSimNanos The clock time associated with the model call
 */
void PlanetNav::computeErrors(uint64_t CurrentSimNanos)
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
    this->errorModel.setPropMatrix(localProp);
    this->errorModel.computeNextState();
    this->navErrors = this->errorModel.getCurrentState();
}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.

    @param CurrentSimNanos The clock time associated with the model call
*/
void PlanetNav::UpdateState(uint64_t CurrentSimNanos)
{
    /* zero the output msg buffer */
    this->noisePlanetState = this->ephemerisOutMsg.zeroMsgPayload;

    this->readInputMessages();
    this->computeErrors(CurrentSimNanos);
    this->applyErrors();
    this->writeOutputMessages(CurrentSimNanos);
    this->prevTime = CurrentSimNanos;
}
