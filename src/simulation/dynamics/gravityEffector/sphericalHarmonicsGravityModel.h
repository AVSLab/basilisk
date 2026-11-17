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

#include <Eigen/Dense>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

class GravBodyData;

/**
 * The Spherical Harmonics gravity model
 */
class SphericalHarmonicsGravityModel : public GravityModel
{
  public:
    /** @brief Validate coefficients and initialize the recurrence tables.
     *
     * The attributes `muBody` and `radEquator` must be set separately.
     * Previous recurrence tables are discarded on every attempt.
     *
     * @return An empty optional on success, or an error message if either coefficient
     * table has fewer than `maxDeg + 1` rows or a retained row `l` has fewer than `l + 1` entries.
     * @note After failure, correct the configuration and initialize successfully before evaluating the field.
     */
    std::optional<std::string> initializeParameters() override;

    /** @brief Initialize the gravity model from the body parameters.
     *
     * The attribute `radEquator` is kept if it is positive, for example when it was
     * loaded from the coefficient file, since the normalized coefficients are only
     * valid with the reference radius of their model. Otherwise, it is read from
     * the `radEquator` of the given `GravBodyData`.
     *
     * The attribute `muBody` is replaced by `body.mu`, which must be finite and
     * positive so the field and other gravity consumers use the same parameter.
     * Previous recurrence tables are discarded on every attempt.
     *
     * @param body Gravity body supplying the gravitational parameter and fallback reference radius.
     * @return An empty optional on success, or an error message if `body.mu` is invalid
     * or the coefficient tables do not contain all entries through `maxDeg`.
     * @note After failure, correct the configuration and initialize successfully before evaluating the field.
     */
    std::optional<std::string> initializeParameters(const GravBodyData& body) override;

    /** @brief Compute gravity acceleration using the current `maxDeg`, including the point-mass term.
     *
     * @param position_planetFixed Position relative to the body in the body-fixed frame [m].
     * @return Acceleration in the body-fixed frame [m/s^2].
     * @throws BasiliskError If initialization has not succeeded, `maxDeg` exceeds the initialized degree,
     * or either coefficient table lacks an entry needed for the evaluation.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed) const override;

    /** @brief Compute gravity acceleration through a requested degree.
     *
     * @param position_planetFixed Position relative to the body in the body-fixed frame [m].
     * @param degree Maximum degree to evaluate; must not exceed `maxDeg` or the last initialized degree.
     * @param include_zero_degree Whether to include the point-mass contribution.
     * @return Acceleration in the body-fixed frame [m/s^2].
     * @throws BasiliskError If initialization has not succeeded, the requested degree exceeds a limit,
     * or either coefficient table lacks an entry needed for the evaluation.
     * @note Coefficient values may be changed without reinitialization. Only rows and orders used by
     * this evaluation are checked; higher degrees and extra columns are ignored.
     */
    Eigen::Vector3d computeField(const Eigen::Vector3d& position_planetFixed,
                                 size_t degree,
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

    /** Returns true if any retained coefficient of order >= 1 (a tesseral or
     * sectoral term) is non-zero. Such terms vary with the body's longitude, so
     * the body orientation must be known to evaluate them correctly. A purely
     * zonal field (only order-0 terms, e.g. a J2-only model) returns false.
     */
    bool dependsOnOrientation() const override;

  public:
    double radEquator = 0; /**< [m] Reference radius of the spherical harmonics coefficients */
    double muBody = 0;     /**< [m^3/s^2] Gravitation parameter for the planet */

    /** @brief Maximum degree of spherical harmonics to use.
     *
     * Initialization requires at least `maxDeg + 1` rows in each coefficient table.
     * Each retained row `l` must contain at least `l + 1` entries. Higher degrees
     * and extra columns are ignored. Reinitialize before evaluating degrees above
     * the degree prepared by the last successful initialization.
     */
    size_t maxDeg = 0;

    /** @brief Normalized cosine coefficients indexed as `cBar[degree][order]` [-].
     * @note See `maxDeg` for the required table dimensions.
     */
    std::vector<std::vector<double>> cBar;

    /** @brief Normalized sine coefficients indexed as `sBar[degree][order]` [-].
     * @note See `maxDeg` for the required table dimensions.
     */
    std::vector<std::vector<double>> sBar;

    mutable BSKLogger bskLogger; //!< BSK Logging

  private:
    /** @brief Discard recurrence tables and invalidate the initialized degree. */
    void clearParameters();

    /** @brief Check the coefficient entries needed through a given degree.
     * @param degree Highest degree whose coefficients must be present.
     * @return An empty optional on success, or an error identifying the missing row or entries.
     */
    std::optional<std::string> validateCoefficients(size_t degree) const;

    std::optional<size_t> initializedDegree; //!< Degree prepared by the last successful initialization.

    /**
     * The following parameters are used internally to compute the gravity.
     *
     * They are coefficients used in the method of Pines for the gravity due to SH.
     * For their definition, see the 'Basilisk-GravityEffector' documentation.
     */
    mutable std::vector<std::vector<double>> aBar; /**< [-] Eq. 61 */
    std::vector<std::vector<double>> n1;           /**< [-] Eq. 63 */
    std::vector<std::vector<double>> n2;           /**< [-] Eq. 64 */
    std::vector<std::vector<double>> nQuot1;       /**< [-] Eq. 79 */
    std::vector<std::vector<double>> nQuot2;       /**< [-] Eq. 80 */
};

#endif /* SH_GRAVITY_MODEL_H */
