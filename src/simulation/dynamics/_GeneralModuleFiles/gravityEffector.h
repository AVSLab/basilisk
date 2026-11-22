/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef GRAVITY_DYN_EFFECTOR_H
#define GRAVITY_DYN_EFFECTOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/stateExtrapolation.h"
#include "dynParamManager.h"
#include "dynamicEffector.h"
#include "gravityModel.h"
#include "pointMassGravityModel.h"
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/** Container for gravitational body data
 *
 * This class is designed to hold all of the information for a gravity
 * body.  The nominal use-case has it initialized at the python level and
 * attached to dynamics using the AddGravityBody method.
 */
class GravBodyData
{
  public:
    friend class GravityEffector;

    /** Initializes `localPlanet` to zero except for `localPlanet.J20002Pfix`, which
     * is initialized to the identity matrix. Additionally, initializes `gravityModel`
     * to a `PointMassGravityModel`.
     */
    GravBodyData();

    /** Initializes the gravityModel, logging and raising an error if
     * the initialization fails.
     */
    void initBody(int64_t moduleID);

    /** @brief Compute gravitational acceleration in inertial coordinates.
     *
     * @param r_I Position relative to this body in inertial coordinates [m].
     * @param simTimeNanos Evaluation time [ns].
     * @return Acceleration in inertial coordinates [m/s^2].
     */
    Eigen::Vector3d computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos);

    /** @brief Compute specific gravitational potential energy from an inertial position.
     *
     * @param r_I Position relative to this body in inertial coordinates [m].
     * @param simTimeNanos Evaluation time [ns].
     * @return Specific potential energy, with zero at infinity [m^2/s^2].
     * @note Uses the same extrapolated body orientation as computeGravityInertial().
     */
    double computePotentialEnergyInertial(const Eigen::Vector3d& r_I, uint64_t simTimeNanos) const;

    /** @brief Load a written planet ephemeris or use a zero state for an unlinked input.
     * @note A linked, unwritten input marks the cached sample unavailable.
     */
    void loadEphemeris();

    /** @brief Register planet properties for standalone calls to computeGravityInertial().
     * @param statesIn State manager that will receive orientation updates and must outlive those calls.
     * @note Position, velocity, and gravitational parameter start at zero. Orientation and its
     * derivative start at the values in localPlanet. Each spacecraft registers its own properties separately.
     */
    void registerProperties(DynParamManager& statesIn);

  public:
    bool isCentralBody = false; /**< Flag indicating that object is center */

    std::shared_ptr<GravityModel> gravityModel; /**< Model used to compute the gravity of the object */

    double mu = 0;          /**< [m3/s^2] central body gravitational parameter, also used by the spherical
                                 harmonics field, which requires a finite, positive value */
    double radEquator = 0;  /**< [m]      Equatorial radius for the body, used as the spherical harmonics
                                 reference radius only if the model has none (e.g. not loaded from a file) */
    double radiusRatio = 1; /**< []       ratio of polar over equatorial radius */
    std::string planetName =
        ""; /**< Gravitational body name, this is used as the Spice name if spiceInterface is used */
    std::string displayName =
        ""; /**< This is the name that is displayed in Vizard.  If not set, Vizard shows planetName */
    std::string modelDictionaryKey =
        ""; /**< "" will result in using the current default for the celestial body's given name, otherwise key will be
               matched if possible to available model in internal model dictionary */

    ReadFunctor<SpicePlanetStateMsgPayload> planetBodyInMsg; /**< planet spice ephemeris input message */
    SpicePlanetStateMsgPayload localPlanet; /**< [-]   Class storage of ephemeris info from scheduled portion */

    BSKLogger bskLogger; /**< -- BSK Logging */

  private:
    struct PropertyHandles
    {
        Eigen::MatrixXd* r_PN_N = nullptr;         //!< [m] Planet inertial position.
        Eigen::MatrixXd* v_PN_N = nullptr;         //!< [m/s] Planet inertial velocity.
        Eigen::MatrixXd* muPlanet = nullptr;       //!< [m^3/s^2] Gravitational parameter.
        Eigen::MatrixXd* J20002Pfix = nullptr;     //!< [-] Planet orientation [PN].
        Eigen::MatrixXd* J20002Pfix_dot = nullptr; //!< [1/s] Planet orientation derivative.
    };
    PropertyHandles createProperties(DynParamManager& statesIn);
    PropertyHandles standaloneProperties; //!< Used only by explicitly registered, standalone body queries.
    double ephemerisOffset(double stageTimeSeconds) const; //!< [s] Stage time relative to the ephemeris message.
    double ephemerisOffset(uint64_t timeNanos) const;      //!< [s] Integer epoch relative to the ephemeris message.
    Eigen::Vector3d positionAtOffset(double dt) const;     //!< [m] Inertial position at offset dt [s] from the message.
    double potentialAtOffset(const Eigen::Vector3d& r_I, double dt) const;
    Eigen::Vector3d gravityAtOffset(const Eigen::Vector3d& r_I, double dt, const PropertyHandles& properties);

    /** @brief Extrapolate the planet orientation and its derivative from the ephemeris message.
     * @param dt Signed time since the ephemeris message [s].
     * @param dcmAdvanced_NPfix Output rotation from body-fixed to inertial coordinates [-].
     * @param dcmAdvanced_NPfix_dot Output time derivative of dcmAdvanced_NPfix [1/s].
     */
    void computePlanetOrientation(double dt,
                                  Eigen::Matrix3d& dcmAdvanced_NPfix,
                                  Eigen::Matrix3d& dcmAdvanced_NPfix_dot) const;

    uint64_t timeWritten = 0;    /**< [ns]     time the input planet state message was written */
    bool ephemerisReady = false; //!< The cached planet state is available to all spacecraft sharing this body.
};

/*! @brief gravity effector class */
class GravityEffector : public SysModel
{
  public:
    /** Initializes every `GravBodyData` associated with this `GravityEffector` */
    void Reset(uint64_t currentSimNanos);

    /** Updates the central body, loads the ephemeris data for every `GravBodyData`,
     * and writes the output messages. */
    void UpdateState(uint64_t currentSimNanos);

    /** Links the correlation between times property */
    void linkInStates(DynParamManager& statesIn);

    /** @brief Register spacecraft gravity properties and the properties of each associated body.
     * @param statesIn This spacecraft's state manager.
     */
    void registerProperties(DynParamManager& statesIn);

    /** Calculate gravitational acceleration of s/c wrt inertial (no central body) or wrt central body
     *
     *   @param r_cF_N is position of center of mass of s/c wrt frame
     *   @param rDot_cF_N is the derivative of above
     */
    void computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N);

    /** @brief Update inertial position and velocity at the current integration stage. */
    void updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N);

    /** @brief Update inertial position and velocity at the spacecraft output epoch.
     * @param r_BF_N Spacecraft position relative to the integration origin, in inertial components [m].
     * @param rDot_BF_N Spacecraft velocity relative to the integration origin, in inertial components [m/s].
     * @param outputTimeNanos Accepted spacecraft epoch [ns].
     */
    void updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N, uint64_t outputTimeNanos);

    /** @brief Accumulate specific potential energy from the associated gravity bodies.
     * @param r_cF_N Spacecraft center-of-mass position relative to the integration origin, in inertial coordinates [m].
     * @param orbPotEnergyContr Specific potential energy accumulator [m^2/s^2].
     * @param simTimeNanos Evaluation time for planet positions and orientations [ns].
     */
    void updateEnergyContributions(Eigen::Vector3d r_cF_N, double& orbPotEnergyContr, uint64_t simTimeNanos);

    /** Sets the `GravBodyData` associated with this effector */
    void setGravBodies(std::vector<std::shared_ptr<GravBodyData>> gravBodies);

    /** Adds a `GravBodyData` associated with this effector */
    void addGravBody(std::shared_ptr<GravBodyData> gravBody);

  private:
    friend class Spacecraft;

    double currentEphemerisOffset(const GravBodyData& body) const;

    /** @brief Update the inertial state using a signed ephemeris offset.
     */
    bool ephemeridesReady() const; //!< Check the current samples, including bodies refreshed by another spacecraft.
    bool bodiesResolved = false;   //!< The last scan completed and identified the integration origin.
    bool originReady = false;      //!< The last gravity sample included the integration origin.
    void loadEphemerides(bool atReset);
    void updateInertialPosAndVelAtOffset(const Eigen::Vector3d& r_BF_N, const Eigen::Vector3d& rDot_BF_N, double dt);

    /** Writes to centralBodyOutMsg if it is linked and there is a central body */
    void writeOutputMessages(uint64_t currentSimNanos);

  public:
    std::vector<std::shared_ptr<GravBodyData>> gravBodies; /**< [-] Vector of bodies we feel gravity from */
    std::shared_ptr<GravBodyData> centralBody;             /**<  Central body */

    std::string vehicleGravityPropName = "g_N";        /**< [-] Name of the vehicle gravity acceleration property */
    std::string systemTimeCorrPropName = "systemTime"; /**< [-] Name of the correlation between times */
    std::string inertialPositionPropName = "r_BN_N";   /**< [-] Name of the inertial position property */
    std::string inertialVelocityPropName = "v_BN_N";   /**< [-] Name of the inertial velocity property */

    Message<SpicePlanetStateMsgPayload> centralBodyOutMsg; /**< central planet body state output message */

    BSKLogger bskLogger; /**< -- BSK Logging */

  private:
    std::unordered_map<std::shared_ptr<GravBodyData>, GravBodyData::PropertyHandles> bodyProperties;
    DynParamManager::InertialFrame* inertialFrame = nullptr; //!< Frame of the registered spacecraft.
    Eigen::MatrixXd* gravProperty;                           /**< [-] g_N property for output */
    Eigen::MatrixXd* timeCorr;                               /**< [-] Time correlation property */
    Eigen::MatrixXd* inertialPositionProperty; /**< [m] r_N inertial position relative to system spice zeroBase/refBase
                                                  coordinate frame, property for output. */
    Eigen::MatrixXd* inertialVelocityProperty; /**< [m/s] v_N inertial velocity relative to system spice
                                                  zeroBase/refBase coordinate frame, property for output. */
};

#endif /* GRAVITY_EFFECTOR_H */
