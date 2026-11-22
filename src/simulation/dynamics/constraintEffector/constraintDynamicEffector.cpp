/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "constraintDynamicEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <array>
#include <cmath>
#include <iostream>

/*! The Constructor */
ConstraintDynamicEffector::ConstraintDynamicEffector()
{
    /* this effector can be attached onto a state effector */
    this->isAttachableToStateEffector = true;
}

/*! This is the destructor, nothing to report here */
ConstraintDynamicEffector::~ConstraintDynamicEffector() {}

/*! Validate and initialize the constraint gains when the scheduler resets the model.

 @param CurrentSimNanos [ns] Time at which the reset occurs
 */
void ConstraintDynamicEffector::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    this->validateConfiguration();
    this->initializeGains();
}

/*! Validate that either the tuning parameters or individual gains were configured. */
void ConstraintDynamicEffector::validateConfiguration()
{
    // Preserve support for partial explicit configurations, including zero unspecified gains.
    const bool gainset = this->k_dExplicit || this->c_dExplicit || this->k_aExplicit || this->c_aExplicit;
    if (this->alpha <= 0 && !gainset) {
        bskLogger.bskError("Alpha must be set to a positive nonzero value prior to initialization");
    }
    if (this->beta <= 0 && !gainset) {
        bskLogger.bskError("Beta must be set to a positive nonzero value prior to initialization");
    }
}

/*! @brief Recompute derived gains without replacing explicitly configured individual gains. */
void ConstraintDynamicEffector::initializeGains()
{
    const double k_d = this->k_dExplicit ? this->k_d : this->alpha * this->alpha; // [N/m]
    const double c_d = this->c_dExplicit ? this->c_d : 2.0 * this->beta;          // [N*s/m]
    const double k_a = this->k_aExplicit ? this->k_a : this->alpha * this->alpha; // [N*m]
    const double c_a = this->c_aExplicit ? this->c_a : 2.0 * this->beta;          // [N*m*s]
    if (!std::isfinite(k_d) || !std::isfinite(c_d) || !std::isfinite(k_a) || !std::isfinite(c_a)) {
        bskLogger.bskError("ConstraintDynamicEffector: alpha and beta must produce finite derived gains.");
    }
    this->k_d = k_d;
    this->c_d = c_d;
    this->k_a = k_a;
    this->c_a = c_a;
}

/*! @brief Set the initial separation between connection points.
 *
 * @param[in] r_P2P1_B1Init [m] Initial separation from connection point P1 to P2.
 */
void ConstraintDynamicEffector::setR_P2P1_B1Init(Eigen::Vector3d r_P2P1_B1Init)
{
    this->r_P2P1_B1Init = r_P2P1_B1Init;
}

/*! @brief Set the first body's connection-point position.
 *
 * @param[in] r_P1B1_B1 [m] Position of connection point P1 relative to hub B1.
 */
void ConstraintDynamicEffector::setR_P1B1_B1(Eigen::Vector3d r_P1B1_B1)
{
    this->r_P1B1_B1 = r_P1B1_B1;
}

/*! @brief Set the second body's connection-point position.
 *
 * @param[in] r_P2B2_B2 [m] Position of connection point P2 relative to hub B2.
 */
void ConstraintDynamicEffector::setR_P2B2_B2(Eigen::Vector3d r_P2B2_B2)
{
    this->r_P2B2_B2 = r_P2B2_B2;
}

/*! @brief Set the initial relative attitude constraint.
 *
 * @param[in] sigma_B2B1Init Initial attitude of body B2 relative to body B1.
 */
void ConstraintDynamicEffector::setSigma_B2B1Init(Eigen::MRPd sigma_B2B1Init)
{
    this->dcm_B2B1Init = sigma_B2B1Init.toRotationMatrix().transpose();
}

/*! @brief Set the proportional Baumgarte tuning parameter.
 *
 * @param[in] alpha [-] Numerical proportional Baumgarte tuning parameter.
 * @note Its square supplies the numerical SI values of derived k_d [N/m] and k_a [N*m].
 */
void ConstraintDynamicEffector::setAlpha(double alpha)
{
    if (std::isfinite(alpha) && alpha > 0.0)
        this->alpha = alpha;
    else {
        bskLogger.bskError("Proportional gain tuning variable alpha must be finite and greater than 0.");
    }
}

/*! @brief Set the derivative Baumgarte tuning parameter.
 *
 * @param[in] beta [-] Numerical derivative Baumgarte tuning parameter.
 * @note Twice its value supplies the numerical SI values of derived c_d [N*s/m] and c_a [N*m*s].
 */
void ConstraintDynamicEffector::setBeta(double beta)
{
    if (std::isfinite(beta) && beta > 0.0)
        this->beta = beta;
    else {
        bskLogger.bskError("Derivative gain tuning parameter beta must be finite and greater than 0.");
    }
}

/*! @brief Set the direction-constraint proportional gain.
 *
 * @param[in] k_d [N/m] Direction-constraint proportional gain.
 */
void ConstraintDynamicEffector::setK_d(double k_d)
{
    if (std::isfinite(k_d) && k_d > 0.0) {
        this->k_d = k_d;
        this->k_dExplicit = true;
    } else {
        bskLogger.bskError("Direction constraint proportional gain k_d must be finite and greater than 0.");
    }
}

/*! @brief Set the direction-constraint derivative gain.
 *
 * @param[in] c_d [N*s/m] Direction-constraint derivative gain.
 */
void ConstraintDynamicEffector::setC_d(double c_d)
{
    if (std::isfinite(c_d) && c_d > 0.0) {
        this->c_d = c_d;
        this->c_dExplicit = true;
    } else {
        bskLogger.bskError("Direction constraint derivative gain c_d must be finite and greater than 0.");
    }
}

/*! @brief Set the attitude-constraint proportional gain.
 *
 * @param[in] k_a [N*m] Attitude-constraint proportional gain multiplying dimensionless MRP error.
 */
void ConstraintDynamicEffector::setK_a(double k_a)
{
    if (std::isfinite(k_a) && k_a > 0.0) {
        this->k_a = k_a;
        this->k_aExplicit = true;
    } else {
        bskLogger.bskError("Attitude constraint proportional gain k_a must be finite and greater than 0.");
    }
}

/*! @brief Set the attitude-constraint derivative gain.
 *
 * @param[in] c_a [N*m*s] Attitude-constraint derivative gain multiplying the MRP error rate.
 */
void ConstraintDynamicEffector::setC_a(double c_a)
{
    if (std::isfinite(c_a) && c_a > 0.0) {
        this->c_a = c_a;
        this->c_aExplicit = true;
    } else {
        bskLogger.bskError("Attitude constraint derivative gain c_a must be finite and greater than 0.");
    }
}

/*! @brief Add a spacecraft attitude-state name.
 *
 * @param[in] value Attitude-state name to add.
 */
void ConstraintDynamicEffector::setStateNameOfSigma(std::string value)
{
    if (!value.empty()) {
        this->stateNameOfSigma.push_back(value);
    } else {
        bskLogger.bskError("ConstraintDynamicEffector: stateNameOfSigma variable must be a non-empty string");
    }
}

/*! @brief Add a spacecraft angular-velocity-state name.
 *
 * @param[in] value Angular-velocity-state name to add.
 */
void ConstraintDynamicEffector::setStateNameOfOmega(std::string value)
{
    if (!value.empty()) {
        this->stateNameOfOmega.push_back(value);
    } else {
        bskLogger.bskError("ConstraintDynamicEffector: stateNameOfOmega variable must be a non-empty string");
    }
}

/*! @brief Add an inertial-position property name.
 *
 * @param[in] value Inertial-position property name to add.
 */
void ConstraintDynamicEffector::setPropName_inertialPosition(std::string value)
{
    if (!value.empty()) {
        this->propName_inertialPosition.push_back(value);
    } else {
        bskLogger.bskError("ConstraintDynamicEffector: propName_inertialPosition variable must be a non-empty string");
    }
}

/*! @brief Add an inertial-velocity property name.
 *
 * @param[in] value Inertial-velocity property name to add.
 */
void ConstraintDynamicEffector::setPropName_inertialVelocity(std::string value)
{
    if (!value.empty()) {
        this->propName_inertialVelocity.push_back(value);
    } else {
        bskLogger.bskError("ConstraintDynamicEffector: propName_inertialVelocity variable must be a non-empty string");
    }
}

/*! @brief Add an inertial-attitude property name.
 *
 * @param[in] value Inertial-attitude property name to add.
 */
void ConstraintDynamicEffector::setPropName_inertialAttitude(std::string value)
{
    if (!value.empty()) {
        this->propName_inertialAttitude.push_back(value);
    } else {
        bskLogger.bskError("ConstraintDynamicEffector: propName_inertialAttitude variable must be a non-empty string");
    }
}

/*! @brief Add an inertial-angular-velocity property name.
 *
 * @param[in] value Inertial-angular-velocity property name to add.
 */
void ConstraintDynamicEffector::setPropName_inertialAngVelocity(std::string value)
{
    if (!value.empty()) {
        this->propName_inertialAngVelocity.push_back(value);
    } else {
        bskLogger.bskError(
            "ConstraintDynamicEffector: propName_inertialAngVelocity variable must be a non-empty string");
    }
}

/*! This method allows the user to set the cut-off frequency of the low pass filter which is then
    used to calculate the coefficients for numerical low pass filtering based on a second-order low
    pass filter design.
 @param wc The cut-off frequency of the low pass filter.
 @param h The constant digital time step.
 @param k The damping coefficient
*/
void ConstraintDynamicEffector::setFilter_Data(double wc, double h, double k)
{
    if (wc > 0) {
        std::array<double, 3> num_coeffs = { pow(wc * h, 2), 2 * pow(wc * h, 2), pow(wc * h, 2) };
        std::array<double, 3> denom_coeffs = { -4 + 4 * k * h - pow(wc * h, 2),
                                               8 - 2 * pow(wc * h, 2),
                                               4 + 4 * k * h + pow(wc * h, 2) };
        this->a = denom_coeffs[1] / denom_coeffs[2];
        this->b = denom_coeffs[0] / denom_coeffs[2];
        this->c = num_coeffs[2] / denom_coeffs[2];
        this->d = num_coeffs[1] / denom_coeffs[2];
        this->e = num_coeffs[0] / denom_coeffs[2];
    } else {
        bskLogger.bskError("Cut off frequency of low pass filter w_c must be greater than 0.");
    }
}

/*! This method allows the user to set the status of the constraint dynamic effector */
void ConstraintDynamicEffector::readInputMessage()
{
    if (this->effectorStatusInMsg.isLinked()) {
        DeviceStatusMsgPayload statusMsg;
        statusMsg = this->effectorStatusInMsg();
        this->effectorStatus = statusMsg.deviceStatus;
    } else { // default to active if no message is linked
        this->effectorStatus = 1;
    }
}

DynamicEffector* ConstraintDynamicEffector::forAttachment(bool hub)
{
    if (this->attachmentCount >= this->endpoints.size()) {
        this->bskLogger.bskError("constraintDynamicEffector: tried to attach more than 2 parents");
    }
    const auto index = this->attachmentCount;
    auto& endpoint = this->endpoints[index];
    endpoint.setPropName_inertialPosition(this->propName_inertialPosition.at(index));
    endpoint.setPropName_inertialVelocity(this->propName_inertialVelocity.at(index));
    // Hub attitude and rate names are stored separately from articulated-body names.
    // For a mixed pair, each parent therefore uses the first entry of its kind.
    const bool firstIsHub = !this->endpoints[0].getStateNameOfSigma().empty();
    const std::size_t typeIndex = index == 1 && firstIsHub == hub ? 1 : 0;
    if (hub) {
        endpoint.setStateNameOfPosition(this->stateNameOfPosition);
        endpoint.setStateNameOfVelocity(this->stateNameOfVelocity);
        endpoint.setStateNameOfSigma(this->stateNameOfSigma.at(typeIndex));
        endpoint.setStateNameOfOmega(this->stateNameOfOmega.at(typeIndex));
    } else {
        endpoint.setPropName_inertialAttitude(this->propName_inertialAttitude.at(typeIndex));
        endpoint.setPropName_inertialAngVelocity(this->propName_inertialAngVelocity.at(typeIndex));
    }
    ++this->attachmentCount;
    return &endpoint;
}

void ConstraintDynamicEffector::Endpoint::linkInStates(DynParamManager& states)
{
    this->hubPosition = states.getStateObject(this->stateNameOfPosition);
    this->hubVelocity = states.getStateObject(this->stateNameOfVelocity);
    this->hubAttitude = states.getStateObject(this->stateNameOfSigma);
    this->hubRate = states.getStateObject(this->stateNameOfOmega);
    this->linkTranslation(states);
}

void ConstraintDynamicEffector::Endpoint::linkInProperties(DynParamManager& states)
{
    this->branchAttitude = states.getPropertyReference(this->propName_inertialAttitude);
    this->branchRate = states.getPropertyReference(this->propName_inertialAngVelocity);
    this->linkTranslation(states);
}

void ConstraintDynamicEffector::Endpoint::linkTranslation(DynParamManager& states)
{
    this->owner.validateConfiguration();
    this->owner.initializeGains();
    this->position = states.getPropertyReference(this->propName_inertialPosition);
    this->velocity = states.getPropertyReference(this->propName_inertialVelocity);
    states.hasConstraint = true;
    this->frame = &states.inertialFrame;
    if (this->owner.bothLinked() && this->owner.endpoints[0].hubAttitude && this->owner.endpoints[1].branchAttitude &&
        this->owner.propName_inertialPosition.size() == 2) {
        // Preserve the public property lists for a hub followed by an articulated parent.
        this->owner.propName_inertialPosition.erase(this->owner.propName_inertialPosition.begin());
        this->owner.propName_inertialVelocity.erase(this->owner.propName_inertialVelocity.begin());
    }
}

Eigen::MRPd ConstraintDynamicEffector::Endpoint::attitude() const
{
    return Eigen::MRPd(this->hubAttitude ? this->hubAttitude->stateView().data() : this->branchAttitude->data());
}

Eigen::Vector3d ConstraintDynamicEffector::Endpoint::angularVelocity() const
{
    if (this->hubRate) {
        return this->hubRate->stateView();
    }
    return *this->branchRate;
}

void ConstraintDynamicEffector::linkInStates(DynParamManager& states)
{
    this->forAttachment(true)->linkInStates(states);
}

void ConstraintDynamicEffector::linkInProperties(DynParamManager& states)
{
    this->forAttachment(false)->linkInProperties(states);
}

bool ConstraintDynamicEffector::bothLinked() const
{
    return this->endpoints[0].frame && this->endpoints[1].frame;
}

void ConstraintDynamicEffector::Endpoint::computeForceTorque(double time [[maybe_unused]],
                                                             double timeStep [[maybe_unused]])
{
    this->owner.evaluateConstraint();
    this->forceExternal_N = this->index == 0 ? this->owner.Fc_N : -this->owner.Fc_N;
    this->torqueExternalPntB_B = this->index == 0 ? this->owner.T_B1 : this->owner.T_B2;
    this->owner.forceExternal_N = this->forceExternal_N;
    this->owner.torqueExternalPntB_B = this->torqueExternalPntB_B;
    this->owner.directEndpoint = 1 - this->index;
}

void ConstraintDynamicEffector::computeForceTorque(double time, double timeStep)
{
    if (this->bothLinked()) {
        this->endpoints[this->directEndpoint].computeForceTorque(time, timeStep);
    }
}

void ConstraintDynamicEffector::evaluateConstraint()
{
    if (!this->bothLinked()) {
        return;
    }
    const bool unknownOrigin = !this->endpoints[0].frame->originReady || !this->endpoints[1].frame->originReady;
    if (this->effectorStatus == 0 || unknownOrigin) {
        this->Fc_N.setZero();
        this->T_B1.setZero();
        this->T_B2.setZero();
    }
    if (unknownOrigin) {
        return;
    }
    const auto& first = this->endpoints[0];
    const auto& second = this->endpoints[1];
    Eigen::Vector3d r_B2B1_N;
    Eigen::Vector3d rDot_B2B1_N;
    if (first.hubPosition && second.hubPosition && first.frame->originPosition == second.frame->originPosition &&
        first.frame->originVelocity == second.frame->originVelocity) {
        // The same translation and boost cancel exactly in the integrated hub states.
        r_B2B1_N = second.hubPosition->stateView() - first.hubPosition->stateView();
        rDot_B2B1_N = second.hubVelocity->stateView() - first.hubVelocity->stateView();
    } else {
        r_B2B1_N = *second.position - *first.position;
        rDot_B2B1_N = *second.velocity - *first.velocity;
    }
    const Eigen::Vector3d omega_B1N_B1 = first.angularVelocity();
    const Eigen::MRPd sigma_B1N = first.attitude();
    const Eigen::Vector3d omega_B2N_B2 = second.angularVelocity();
    const Eigen::MRPd sigma_B2N = second.attitude();

    // computing direction constraint psi in the N frame
    Eigen::Matrix3d dcm_B1N = (sigma_B1N.toRotationMatrix()).transpose();
    Eigen::Matrix3d dcm_B2N = (sigma_B2N.toRotationMatrix()).transpose();
    Eigen::Vector3d r_P1B1_N = dcm_B1N.transpose() * this->r_P1B1_B1;
    Eigen::Vector3d r_P2B2_N = dcm_B2N.transpose() * this->r_P2B2_B2;
    // Difference the parent states before adding local offsets to preserve small attachment motions.
    Eigen::Vector3d r_P2P1_N = r_B2B1_N + (r_P2B2_N - r_P1B1_N);

    // computing length constraint rate of change psiPrime in the N frame
    Eigen::Vector3d rDot_P1B1_B1 = omega_B1N_B1.cross(this->r_P1B1_B1);
    Eigen::Vector3d rDot_P2B2_B2 = omega_B2N_B2.cross(this->r_P2B2_B2);
    Eigen::Vector3d rDot_P2P1_N =
        rDot_B2B1_N + (dcm_B2N.transpose() * rDot_P2B2_B2 - dcm_B1N.transpose() * rDot_P1B1_B1);
    Eigen::Vector3d omega_B1N_N = dcm_B1N.transpose() * omega_B1N_B1;

    // calculate the direction constraint violations
    this->psi_N = r_P2P1_N - dcm_B1N.transpose() * this->r_P2P1_B1Init;
    this->psiPrime_N = rDot_P2P1_N - omega_B1N_N.cross(r_P2P1_N);

    // calculate the attitude constraint violations
    this->phi = eigenC2MRP(dcm_B2N * dcm_B1N.transpose() *
                           this->dcm_B2B1Init.transpose()); // calculate the difference in attitude
    Eigen::Vector3d omega_B1N_B2 = dcm_B2N * dcm_B1N.transpose() * omega_B1N_B1;
    this->omega_B2B1_B2 = omega_B2N_B2 - omega_B1N_B2; // difference in angular rate

    if (this->effectorStatus == 1) {
        // calculate the constraint force
        this->Fc_N = this->k_d * this->psi_N + this->c_d * this->psiPrime_N; // force on endpoint 1

        // calculate the torque on each spacecraft from the direction constraint
        Eigen::Vector3d Fc_B1 = dcm_B1N * this->Fc_N;
        Eigen::Vector3d L_B1_len = (this->r_P1B1_B1).cross(Fc_B1);
        Eigen::Vector3d Fc_B2 = dcm_B2N * this->Fc_N;
        Eigen::Vector3d L_B2_len = -this->r_P2B2_B2.cross(Fc_B2);

        // calculate the constraint torque imparted on each spacecraft from the attitude constraint
        Eigen::Matrix3d dcm_B1B2 = dcm_B1N * dcm_B2N.transpose();
        Eigen::Vector3d L_B2_att =
            -this->k_a * this->phi.coeffs() - this->c_a * 0.25 * this->phi.Bmat() * omega_B2B1_B2;
        Eigen::Vector3d L_B1_att = -dcm_B1B2 * L_B2_att;
        this->T_B2 = L_B2_len + L_B2_att; // store the constraint torque for spacecraft 2

        // assign forces and torques for spacecraft 1
        this->T_B1 = L_B1_len + L_B1_att;
    }
}

/*! This method takes the computed constraint force and torque states and outputs them to the
 messaging system.
 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::writeOutputStateMessage(uint64_t CurrentClock)
{
    ConstDynEffectorMsgPayload outputForces;
    outputForces = this->constraintElements.zeroMsgPayload;
    eigenVector3d2CArray(this->forceExternal_N, outputForces.Fc_N);
    eigenVector3d2CArray(this->T_B1, outputForces.L1_B1);
    eigenVector3d2CArray(this->T_B2, outputForces.L2_B2);
    eigenVector3d2CArray(this->psi_N, outputForces.psi_N);
    outputForces.Fc_mag_filtered = this->F_filtered_mag_t;
    outputForces.L1_mag_filtered = this->T1_filtered_mag_t;
    outputForces.L2_mag_filtered = this->T2_filtered_mag_t;
    this->constraintElements.write(&outputForces, this->moduleID, CurrentClock);
}

/*! Update state method
 @param CurrentSimNanos The current simulation time
 */
void ConstraintDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessage();
    this->computeFilteredForce(CurrentSimNanos);
    this->computeFilteredTorque(CurrentSimNanos);
    this->writeOutputStateMessage(CurrentSimNanos);
}

/*! Filtering method to calculate filtered Constraint Force
 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::computeFilteredForce(uint64_t CurrentClock [[maybe_unused]])
{
    double F_t[3];
    eigenVector3d2CArray(this->Fc_N, F_t);
    this->F_mag_t = std::sqrt(pow(F_t[0], 2) + pow(F_t[1], 2) + pow(F_t[2], 2));
    this->F_filtered_mag_t = this->a * this->F_filtered_mag_tminus1 + this->b * this->F_filtered_mag_tminus2 +
                             this->c * this->F_mag_t + this->d * this->F_mag_tminus1 + this->e * this->F_mag_tminus2;
    this->F_filtered_mag_tminus2 = this->F_filtered_mag_tminus1;
    this->F_filtered_mag_tminus1 = this->F_filtered_mag_t;
    this->F_mag_tminus2 = this->F_mag_tminus1;
    this->F_mag_tminus1 = this->F_mag_t;
}

/*! Filtering method to calculate filtered Constraint Torque
 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::computeFilteredTorque(uint64_t CurrentClock [[maybe_unused]])
{
    double T_t1[3];
    eigenVector3d2CArray(this->T_B1, T_t1);
    this->T1_mag_t = std::sqrt(pow(T_t1[0], 2) + pow(T_t1[1], 2) + pow(T_t1[2], 2));
    this->T1_filtered_mag_t = this->a * this->T1_filtered_mag_tminus1 + this->b * this->T1_filtered_mag_tminus2 +
                              this->c * this->T1_mag_t + this->d * this->T1_mag_tminus1 +
                              this->e * this->T1_mag_tminus2;
    this->T1_filtered_mag_tminus2 = this->T1_filtered_mag_tminus1;
    this->T1_filtered_mag_tminus1 = this->T1_filtered_mag_t;
    this->T1_mag_tminus2 = this->T1_mag_tminus1;
    this->T1_mag_tminus1 = this->T1_mag_t;
    double T_t2[3];
    eigenVector3d2CArray(this->T_B2, T_t2);
    this->T2_mag_t = std::sqrt(pow(T_t2[0], 2) + pow(T_t2[1], 2) + pow(T_t2[2], 2));
    this->T2_filtered_mag_t = this->a * this->T2_filtered_mag_tminus1 + this->b * this->T2_filtered_mag_tminus2 +
                              this->c * this->T2_mag_t + this->d * this->T2_mag_tminus1 +
                              this->e * this->T2_mag_tminus2;
    this->T2_filtered_mag_tminus2 = this->T2_filtered_mag_tminus1;
    this->T2_filtered_mag_tminus1 = this->T2_filtered_mag_t;
    this->T2_mag_tminus2 = this->T2_mag_tminus1;
    this->T2_mag_tminus1 = this->T2_mag_t;
}
