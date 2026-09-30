/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef INTEGRATOR_TEST_ACCESS_H
#define INTEGRATOR_TEST_ACCESS_H

#include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"

namespace integrator_step_test {

/** @brief Friend access to the integrator's private stepping entry point for native tests. */
class StateVecIntegratorTestAccess
{
  public:
    /**
     * @brief Prepare and advance an integrator through its normal stepping entry point.
     * @param integrator Integrator to advance.
     * @param currentTime [s] Start time of the integration step.
     * @param timeStep [s] Duration of the integration step.
     */
    static void step(StateVecIntegrator& integrator, double currentTime, double timeStep)
    {
        integrator.integrate(currentTime, timeStep);
    }
};

/**
 * @brief Advance an integrator through its private stepping entry point for testing.
 * @param integrator Integrator to advance.
 * @param currentTime [s] Start time of the integration step.
 * @param timeStep [s] Duration of the integration step.
 */
inline void
stepIntegrator(StateVecIntegrator& integrator, double currentTime, double timeStep)
{
    StateVecIntegratorTestAccess::step(integrator, currentTime, timeStep);
}

/**
 * @brief Advance an integrator supplied by pointer for testing.
 * @param integrator Non-null pointer to the integrator to advance.
 * @param currentTime [s] Start time of the integration step.
 * @param timeStep [s] Duration of the integration step.
 */
inline void
stepIntegrator(StateVecIntegrator* integrator, double currentTime, double timeStep)
{
    StateVecIntegratorTestAccess::step(*integrator, currentTime, timeStep);
}
}

#endif
