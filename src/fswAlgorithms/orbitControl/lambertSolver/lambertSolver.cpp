/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/orbitControl/lambertSolver/lambertSolver.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSolver::LambertSolver() = default;

/*! Module Destructor */
LambertSolver::~LambertSolver() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSolver.lambertProblemInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated. It computes the solution of Lambert's problem.
    @param CurrentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertSolver::UpdateState(uint64_t CurrentSimNanos)
{
}

/*! This method reads the input messages each call of updateState. It also checks if the message contents are valid for this module.
    @return void
*/
void LambertSolver::readMessages(){
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();

    // check if input parameters are valid
    if (lambertProblemInMsgBuffer.mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: mu must be positive.");
    } else {
        this->mu = lambertProblemInMsgBuffer.mu;
    }
    if (lambertProblemInMsgBuffer.transferTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: transferTime must be positive.");
    } else {
        this->transferTime = lambertProblemInMsgBuffer.transferTime;
    }
    if (lambertProblemInMsgBuffer.numRevolutions < 0){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: numberOfRevolutions must be zero or positive.");
    } else {
        this->numberOfRevolutions = lambertProblemInMsgBuffer.numRevolutions;
    }
    if (!(strcmp(lambertProblemInMsgBuffer.solverName, "Gooding") == 0 || strcmp(lambertProblemInMsgBuffer.solverName, "Izzo") == 0)){
        bskLogger.bskLog(BSK_ERROR, "lambertSolver: solverName must be either 'Gooding' or 'Izzo'.");
    } else {
        this->solverName.assign(lambertProblemInMsgBuffer.solverName);
    }

    this->r1vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r1vec);
    this->r2vec = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2vec);
}
