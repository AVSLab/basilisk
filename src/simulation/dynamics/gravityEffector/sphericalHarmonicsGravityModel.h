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

#ifndef SH_GRAVITY_MODEL_H
#define SH_GRAVITY_MODEL_H

#include "architecture/utilities/bskLogging.h"
#include "simulation/dynamics/_GeneralModuleFiles/gravityModel.h"

#include <vector>

/**
 * The Spherical Harmonics gravity model
 */
class SphericalHarmonicsGravityModel : public GravityModel {
  public:
    /** Initialize all parameters necessary for the computation of gravity.
     *
     * The attributes `muBody`and `radEquator` must be set separately.
     *
     * Will return an error message (string) if `cBar` or `sBar` were not set.
     * Otherwise, returns an empty optional.
     */
    std::optional<std::string> initializeParameters() override;

    /** Initialize all parameters necessary for the computation of gravity.
     *
     * The attributes `muBody`and `radEquator` are read from the given `GravBodyData`.
     *
     * Will return an error message (string) if `cBar` or `sBar` were not set.
     * Otherwise, returns an empty optional.
     */
    std::optional<std::string> initializeParameters(const GravBodyData&) override;

    /** Returns the gravity acceleration at a position around this body.
     *
     * The position is given in the body-fixed reference frame.
     * Likewise, the resulting acceleration should be given in the
     * body-fixed reference frame.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed) const override;

    /** Returns the gravity acceleration at a position around this body.
     *
     * The position is given in the body-fixed reference frame.
     * Likewise, the resulting acceleration should be given in the
     * body-fixed reference frame.
     *
     * If the given `degree` is smaller than the maximum degree stored in
     * this class, then the field is computed using only degrees up to `degree`.
     * If the requested degree is higher than the available one, an error is thrown.
     *
     * If include_zero_degree is false the degree that corresponds to the spherical
     * term (point-mass) of the gravity is ignored.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed, size_t degree,
                                 bool include_zero_degree) const;

    /** Returns the gravitational potential energy at a position around this body.
     *
     * The current implementation returns the potential energy of a point-mass
     * (the spherical harmonics coefficients of the body are ignored)
     *
     * The position is given relative to the body and in the inertial
     * reference frame.
     */
    double computePotentialEnergy(const Eigen::Vector3d& positionWrtPlanet_N) const override;

  public:
    double radEquator = 0;  /**< [m] Reference radius for the planet */
    double muBody = 0;      /**< [m^3/s^2] Gravitation parameter for the planet */

    /** The maximum degree of Spherical Harmonics to use
     *
     * A value of maxDeg greater than the size of cBar or sBar will cause an error.
     * A value that is lower will truncate the spherical harmonics, ignoring any
     * parameters in cBar/sBar with degree greater than maxDeg.
     */
    size_t maxDeg = 0;

    /** The normalized "C" spherical harmonics coefficients */
    std::vector<std::vector<double>> cBar;

    /** The normalized "S" spherical harmonics coefficients */
    std::vector<std::vector<double>> sBar;

    mutable BSKLogger bskLogger; //!< BSK Logging

  private:
    /**
     * The following parameters are used internally to compute the gravity.
     *
     * They are coefficients used in the method of Pines for the gravity due to SH.
     * For their definition, see the 'Basilisk-GravityEffector' documentation.
     */
    mutable std::vector<std::vector<double>> aBar;  /**< [-] Eq. 61 */
    std::vector<std::vector<double>> n1;            /**< [-] Eq. 63 */
    std::vector<std::vector<double>> n2;            /**< [-] Eq. 64 */
    std::vector<std::vector<double>> nQuot1;        /**< [-] Eq. 79 */
    std::vector<std::vector<double>> nQuot2;        /**< [-] Eq. 80 */
};

#endif /* SH_GRAVITY_MODEL_H */
