/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at
 Boulder

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

#ifndef POINT_MASS_GRAVITY_MODEL_H
#define POINT_MASS_GRAVITY_MODEL_H

#include "gravityModel.h"

/**
 * The point mass gravity model
 */
class PointMassGravityModel : public GravityModel {
  public:
    /** Does nothing, as the point-mass gravity model has no parameters other than
     * `muBody`, which must be set separately */
    std::optional<std::string> initializeParameters() override { return {}; };

    /** Reads the only necessary parameter (`muBody`) from the given `GravBodyData`*/
    std::optional<std::string> initializeParameters(const GravBodyData&) override;

    /** Returns the gravity acceleration at a position around this body.
     *
     * The position is given in the body-fixed reference frame.
     * Likewise, the resulting acceleration should be given in the
     * body-fixed reference frame.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed) const override;

    /** Returns the gravitational potential energy at a position around this body.
     *
     * The position is given relative to the body and in the inertial
     * reference frame.
     */
    double computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const override;

  public:
    double muBody = 0; /**< [m^3/s^2] Gravitation parameter for the planet */
};

#endif /* POINT_MASS_GRAVITY_MODEL_H */
