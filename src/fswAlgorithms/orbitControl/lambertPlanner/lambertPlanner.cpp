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

#include "fswAlgorithms/orbitControl/lambertPlanner/lambertPlanner.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>
#include <array>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertPlanner::LambertPlanner() = default;

/*! Module Destructor */
LambertPlanner::~LambertPlanner() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertPlanner::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertPlanner.navTransInMsg was not linked.");
    }

    // check that the provided input module parameters are valid
    if (this->mu <= 0.0){
        bskLogger.bskLog(BSK_ERROR, "lambertPlanner: mu must be positive.");
    }
    if (this->finalTime - this->maneuverTime <= 0.0){
        bskLogger.bskLog(BSK_ERROR,
                         "lambertPlanner: Maneuver start time maneuverTime must be before final time finalTime.");
    }
    if (!(this->solverName == "Gooding" || this->solverName == "Izzo")){
        bskLogger.bskLog(BSK_ERROR, "lambertPlanner: solverName must be either 'Gooding' or 'Izzo'.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    @param currentSimNanos current simulation time in nano-seconds
    @return void
*/
void LambertPlanner::UpdateState(uint64_t currentSimNanos)
{
}
