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

#include "dynamicEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <Eigen/Dense>
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <memory>
#include "gravityModel.h"
#include "pointMassGravityModel.h"
#include "architecture/utilities/avsEigenSupport.h"

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

    /** Computes the gravitational acceleration
     *
     * @param r_I inertial position vector
     * @param simTimeNanos simulation time (ns)
     */
    Eigen::Vector3d computeGravityInertial(Eigen::Vector3d r_I, uint64_t simTimeNanos);

    /** Read the ephemeris data from planetBodyInMsg if it's linked.
     * Otherwise, zeros `this->localPlanet`.
     */
    void loadEphemeris();

    /** Creates the following properies in the given statesIn object.
     * 
     *      - [planetName].r_PN_N
     *      - [planetName].v_PN_N
     *      - [planetName].mu
     *      - [planetName].J20002Pfix
     *      - [planetName].J20002Pfix_dot
     * 
     * vr_PN_N`, `v_PN_N`, and `mu` are initialized to zero, while `J20002Pfix` and `J20002Pfix_dot`
     * are initialized to the values stored in `this->localPlanet`. This usually means that
     * `J20002Pfix` is initialized to the identity matrix and `J20002Pfix_dot` to zero.
     */
    void registerProperties(DynParamManager &statesIn);

public:
    bool isCentralBody = false; /**< Flag indicating that object is center */

    std::shared_ptr<GravityModel> gravityModel; /**< Model used to compute the gravity of the object */

    double mu = 0;                       /**< [m3/s^2] central body gravitational param */
    double radEquator = 0;               /**< [m]      Equatorial radius for the body */
    double radiusRatio = 1;              /**< []       ratio of polar over equatorial radius */
    std::string planetName = "";         /**< Gravitational body name, this is used as the Spice name if spiceInterface is used */
    std::string displayName = "";        /**< This is the name that is displayed in Vizard.  If not set, Vizard shows planetName */
    std::string modelDictionaryKey = ""; /**< "" will result in using the current default for the celestial body's given name, otherwise key will be matched if possible to available model in internal model dictionary */

    ReadFunctor<SpicePlanetStateMsgPayload> planetBodyInMsg; /**< planet spice ephemeris input message */
    SpicePlanetStateMsgPayload localPlanet;             /**< [-]   Class storage of ephemeris info from scheduled portion */

    BSKLogger bskLogger; /**< -- BSK Logging */

private:
    Eigen::MatrixXd *r_PN_N;         /**< [m]      (state engine property) planet inertial position vector */
    Eigen::MatrixXd *v_PN_N;         /**< [m/s]    (state engine property) planet inertial velocity vector */
    Eigen::MatrixXd *muPlanet;       /**< [m/s]    (state engine property) planet inertial velocity vector */
    Eigen::MatrixXd *J20002Pfix;     /**< [m/s]    (state engine property) planet attitude [PN] */
    Eigen::MatrixXd *J20002Pfix_dot; /**< [m/s]    (state engine property) planet attitude rate [PN_dot] */

    uint64_t timeWritten = 0; /**< [ns]     time the input planet state message was written */
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
    void linkInStates(DynParamManager &statesIn);

    /** Registers the gravity, inertial position, and inertial velocity properties.
     * It also calls `registerProperties` for every associated `GravBodyData`.
     */
    void registerProperties(DynParamManager &statesIn);

    /** Calculate gravitational acceleration of s/c wrt inertial (no central body) or wrt central body
     *  
     *   @param r_cF_N is position of center of mass of s/c wrt frame
     *   @param rDot_cF_N is the derivative of above
     */
    void computeGravityField(Eigen::Vector3d r_cF_N, Eigen::Vector3d rDot_cF_N);

    /** Updates the inertial position and velocity properties */
    void updateInertialPosAndVel(Eigen::Vector3d r_BF_N, Eigen::Vector3d rDot_BF_N);

    /** Computes the Potential Energy Contributions from every associated `GravBodyData` */
    void updateEnergyContributions(Eigen::Vector3d r_CN_N, double &orbPotEnergyContr);

    /** Sets the `GravBodyData` associated with this effector */
    void setGravBodies(std::vector<std::shared_ptr<GravBodyData>> gravBodies);

    /** Adds a `GravBodyData` associated with this effector */
    void addGravBody(std::shared_ptr<GravBodyData> gravBody);

    /** Called to modify property names to prepend them by the string stored in nameOfSpacecraftAttachedTo 
     * 
     * This can be used to make property names unique between different `GravityEffector` in a simulation
     * with multiple dynamic objects.
    */
    void prependSpacecraftNameToStates();

private:
    /**
        Compute planet position with Euler integration
        @param bodyData planet data
    */
    Eigen::Vector3d getEulerSteppedGravBodyPosition(std::shared_ptr<GravBodyData> bodyData);

    /** Writes to centralBodyOutMsg if it is linked and there is a central body */
    void writeOutputMessages(uint64_t currentSimNanos);

public:
    std::vector<std::shared_ptr<GravBodyData>> gravBodies; /**< [-] Vector of bodies we feel gravity from */
    std::shared_ptr<GravBodyData> centralBody;             /**<  Central body */

    std::string vehicleGravityPropName = "g_N";        /**< [-] Name of the vehicle gravity acceleration property */
    std::string systemTimeCorrPropName = "systemTime"; /**< [-] Name of the correlation between times */
    std::string inertialPositionPropName = "r_BN_N";   /**< [-] Name of the inertial position property */
    std::string inertialVelocityPropName = "v_BN_N";   /**< [-] Name of the inertial velocity property */
    std::string nameOfSpacecraftAttachedTo = "";       /**< [-] Name of the s/c this gravity model is attached to */

    Message<SpicePlanetStateMsgPayload> centralBodyOutMsg; /**< central planet body state output message */

    BSKLogger bskLogger; /**< -- BSK Logging */

private:
    Eigen::MatrixXd *gravProperty;             /**< [-] g_N property for output */
    Eigen::MatrixXd *timeCorr;                 /**< [-] Time correlation property */
    Eigen::MatrixXd *inertialPositionProperty; /**< [m] r_N inertial position relative to system spice zeroBase/refBase coordinate frame, property for output. */
    Eigen::MatrixXd *inertialVelocityProperty; /**< [m/s] v_N inertial velocity relative to system spice zeroBase/refBase coordinate frame, property for output. */
};

#endif /* GRAVITY_EFFECTOR_H */
