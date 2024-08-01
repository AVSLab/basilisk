/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef LAMBERT_VALIDATOR_MESSAGE_H
#define LAMBERT_VALIDATOR_MESSAGE_H


/*! @brief Structure used to log results of lambert validator, such as constraint violations.
    Specifies why lambertValidator failed to write a non-zero DV command message */
typedef struct {
    int failedValidLambert; //!< [-] no valid lambert solution if 1, otherwise 0
    int failedNumIterationsLambert; //!< [-] Lambert solver root finder iterations for x too high if 1, otherwise 0
    int failedXToleranceLambert; //!< [-] Lambert solver root finder for x not converged if 1, otherwise 0
    int failedXSolutionConvergence; //!< [-] x solution too different from previous time step if 1, otherwise 0
    int failedDvSolutionConvergence; //!< [-] Delta-V solution too different from previous time step if 1, otherwise 0
    int failedDistanceTargetConstraint; //!< [-] violated the maximum distance from target constraint if 1, otherwise 0
    int failedOrbitRadiusConstraint; //!< [-] violated the minimum orbit radius constraint if 1, otherwise 0
    double xSolutionDifference; //!< [-] difference in solution for free variable (iteration variable) to previous time
    double dvSolutionDifference; //!< [m/s] difference in Delta-V solution magnitude to previous time step
    int violationsDistanceTarget; //!< [-] number of violations of the maximum distance from target constraint
    int violationsOrbitRadius; //!< [-] number of violations of the minimum orbit radius constraint
    double dv_N[3]; //!< [m/s] Delta-V vector that would be commanded at maneuver time if all checks passed
}LambertValidatorMsgPayload;

#endif
