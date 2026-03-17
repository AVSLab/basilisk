/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#ifndef ZERO_WIND_MODEL_H
#define ZERO_WIND_MODEL_H

#include <Eigen/Dense>

#include "simulation/environment/_GeneralModuleFiles/windBase.h"

/*! @brief Zero wind model.
 *
 * This model represents a zero wind condition, where the only atmospheric
 * motion comes from the co-rotating atmosphere (handled by WindBase).
 * The wind perturbation component v_wind_N is always zero.
 *
 * The module is a sub-class of `windBase`.  See that class for the nominal
 * messages used and setup instructions.
 */
class ZeroWindModel : public WindBase {
public:
    ZeroWindModel();
    ~ZeroWindModel() = default;

private:
    /*! Evaluates the wind velocity at the spacecraft position.
     *
     *  This sets `v_wind_N` to zero since there are no wind perturbations.
     *  The co-rotating atmosphere component is handled by WindBase.
     *
     *  @param msg          Wind output message to populate.
     *  @param r_BP_N       Spacecraft position relative to planet in inertial frame [m].
     *  @param v_corotatingAir_N  Co-rotating atmosphere velocity [m/s] in N frame.
     *  @param currentTime  Current simulation time (s) - unused in this model.
     */
    void evaluateWindModel(WindMsgPayload *msg, const Eigen::Vector3d& r_BP_N,
                          const Eigen::Vector3d& v_corotatingAir_N, double currentTime) override;

};

#endif /* ZERO_WIND_MODEL_H */
