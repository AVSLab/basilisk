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

#include "fswAlgorithms/orbitControl/lambertValidator/lambertValidator.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/orbitalMotion.h"
#include <cmath>
#include <vector>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertValidator::LambertValidator() = default;

/*! Module Destructor */
LambertValidator::~LambertValidator() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertValidator::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.navTransInMsg was not linked.");
    }
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertProblemInMsg was not linked.");
    }
    if (!this->lambertSolutionInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertSolutionInMsg was not linked.");
    }
    if (!this->lambertPerformanceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertValidator.lambertPerformanceInMsg was not linked.");
    }

    // check that the provided input module parameters are valid
    if (this->finalTime - this->maneuverTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertValidator: Maneuver start time maneuverTime must be before final time finalTime.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertValidator::UpdateState(uint64_t currentSimNanos)
{
}
