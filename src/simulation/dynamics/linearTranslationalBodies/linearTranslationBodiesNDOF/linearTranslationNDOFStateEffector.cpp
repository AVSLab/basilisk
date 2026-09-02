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

#include "linearTranslationNDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

/*! This is the constructor, setting variables to default values */
LinearTranslationNDOFStateEffector::LinearTranslationNDOFStateEffector()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    this->nameOfRhoState = "translatingBodyRho" + std::to_string(LinearTranslationNDOFStateEffector::effectorID);
    this->nameOfRhoDotState = "translatingBodyRhoDot" + std::to_string(LinearTranslationNDOFStateEffector::effectorID);
    // preserves the effectorID for bodies added later
    this->propertyNameIndex = std::to_string(LinearTranslationNDOFStateEffector::effectorID);
    LinearTranslationNDOFStateEffector::effectorID++;
}

uint64_t LinearTranslationNDOFStateEffector::effectorID = 1;

/*! This is the destructor, releasing the per body output messages */
LinearTranslationNDOFStateEffector::~LinearTranslationNDOFStateEffector()
{
    for (size_t c = 0; c < this->translatingBodyOutMsgs.size(); c++) {
        delete this->translatingBodyOutMsgs.at(c);
        delete this->translatingBodyConfigLogOutMsgs.at(c);
    }
}

void TranslatingBody::setMass(double mass) {
    if (mass >= 0.0)
        this->mass = mass;
    else {
        this->bskLogger.bskError("Mass must be greater than or equal to 0.");
    }
}

void TranslatingBody::setFHat_P(Eigen::Vector3d fHat_P) {
    if (fHat_P.norm() > 0.01) {
        this->fHat_P = fHat_P.normalized();
    }
    else {
        this->bskLogger.bskError("Norm of fHat must be greater than 0.");
    }
}

void TranslatingBody::setK(double k) {
    if (k >= 0.0)
        this->k = k;
    else {
        this->bskLogger.bskError("k must be greater than or equal to 0.");
    }
}

void TranslatingBody::setC(double c) {
    if (c >= 0.0)
        this->c = c;
    else {
        this->bskLogger.bskError("c must be greater than or equal to 0.");
    }
}

/*! This method is used to reset the module. */
void LinearTranslationNDOFStateEffector::Reset(uint64_t CurrentClock [[maybe_unused]])
{
    this->validateConfiguration();
}

/*! This method is used to add a translating body. */
void LinearTranslationNDOFStateEffector::addTranslatingBody(const std::shared_ptr<TranslatingBody> newBody) {
    // Pushback new body
    translatingBodyVec.push_back(newBody);
    this->N++;

    // Create the output vectors
    this->translatingBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);
    this->translatingBodyRefInMsgs.push_back(ReadFunctor<LinearTranslationRigidBodyMsgPayload>());

    // resize A B and C
    this->ARho.conservativeResize(this->ARho.rows()+1, 3);
    this->BRho.conservativeResize(this->BRho.rows()+1, 3);
    this->CRho.conservativeResize(this->CRho.rows()+1);

    const std::string bodySuffix = this->propertyNameIndex + "_" + std::to_string(this->N);
    newBody->nameOfInertialPositionProperty = "linearTranslationInertialPosition" + bodySuffix;
    newBody->nameOfInertialVelocityProperty = "linearTranslationInertialVelocity" + bodySuffix;
    newBody->nameOfInertialAttitudeProperty = "linearTranslationInertialAttitude" + bodySuffix;
    newBody->nameOfInertialAngVelocityProperty = "linearTranslationInertialAngVelocity" + bodySuffix;
}

/*! This method is used to get a translating body. */
std::shared_ptr<TranslatingBody> LinearTranslationNDOFStateEffector::getTranslatingBody(uint64_t index) {
    assert(("Index must be less than the number of translating body axes", index < static_cast<uint64_t>(this->N)));

    return this->translatingBodyVec.at(index);
}

/*! This method reads motor force, lock flag, and reference state messages. */
void LinearTranslationNDOFStateEffector::readInputMessages()
{
    //! - Read the incoming command array
    if (this->motorForceInMsg.isLinked() && this->motorForceInMsg.isWritten()) {
        ArrayMotorForceMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorForceInMsg();
        size_t i = 0;
        for(auto& translatingBody: this->translatingBodyVec) {
            translatingBody->u = incomingCmdBuffer.motorForce[i];
            i++;
        }
    }

    //! - Zero the command buffer and read the incoming command array
    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        size_t i = 0;
        for(auto& translatingBody: this->translatingBodyVec) {
            translatingBody->isAxisLocked = incomingLockBuffer.effectorLockFlag[i];
            i++;
        }
    }

    size_t translatingBodyIndex = 0;
    for(auto& translatingBody: this->translatingBodyVec) {
        if (this->translatingBodyRefInMsgs[translatingBodyIndex].isLinked() && this->translatingBodyRefInMsgs[translatingBodyIndex].isWritten()) {
            LinearTranslationRigidBodyMsgPayload incomingRefBuffer;
            incomingRefBuffer = this->translatingBodyRefInMsgs[translatingBodyIndex]();
            translatingBody->rhoRef = incomingRefBuffer.rho;
            translatingBody->rhoDotRef = incomingRefBuffer.rhoDot;
        }
        translatingBodyIndex++;
    }
}

/*! This method takes the computed rho states and outputs them to the messaging system. */
void LinearTranslationNDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    this->computeTranslatingBodyInertialStates();

    // Write out the translating body output messages
    size_t i = 0;
    LinearTranslationRigidBodyMsgPayload translatingBodyBuffer;
    SCStatesMsgPayload configLogMsg;
    for(auto& translatingBody: this->translatingBodyVec) {
        if (this->translatingBodyOutMsgs[i]->isLinked()) {
            translatingBodyBuffer = this->translatingBodyOutMsgs[i]->zeroMsgPayload;
            translatingBodyBuffer.rho = translatingBody->rho;
            translatingBodyBuffer.rhoDot = translatingBody->rhoDot;
            this->translatingBodyOutMsgs[i]->write(&translatingBodyBuffer, this->moduleID, CurrentClock);
        }

        if (this->translatingBodyConfigLogOutMsgs[i]->isLinked()) {
            configLogMsg = this->translatingBodyConfigLogOutMsgs[i]->zeroMsgPayload;

            // Logging the F frame is the body frame B of that object
            eigenVector3d2CArray(translatingBody->r_FcN_N, configLogMsg.r_BN_N);
            eigenVector3d2CArray(translatingBody->v_FcN_N, configLogMsg.v_BN_N);
            eigenMatrixXd2CArray(*translatingBody->sigma_FN, configLogMsg.sigma_BN);
            eigenMatrixXd2CArray(*translatingBody->omega_FN_F, configLogMsg.omega_BN_B);
            this->translatingBodyConfigLogOutMsgs[i]->write(&configLogMsg, this->moduleID, CurrentClock);
        }

        i++;
    }
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void LinearTranslationNDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfRhoState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoState;
    this->nameOfRhoDotState = this->nameOfSpacecraftAttachedTo + this->nameOfRhoDotState;
}

/*! This method allows the TB state effector to have access to the hub states and gravity*/
void LinearTranslationNDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
    this->hubSigmaState = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + this->stateNameOfSigma);
}

/*! This method runs every configuration check. Spacecraft initialization always reaches it through
 registerStates(), whereas Reset() runs only when the effector is also added to a task */
void
LinearTranslationNDOFStateEffector::validateConfiguration()
{
    for (auto& translatingBody : this->translatingBodyVec) {
        if (translatingBody->fHat_P.norm() > 0.0) {
            translatingBody->fHat_P.normalize();
        }
        else {
            bskLogger.bskError("Norm of fHat must be greater than 0. fHat may not have been set by the user.");
        }
    }

    this->checkBodyConfiguration();
    this->checkJointMassMatrix();
}

/*! This method checks each translating body's user set frame and inertia */
void
LinearTranslationNDOFStateEffector::checkBodyConfiguration()
{
    if (this->translatingBodyVec.empty()) {
        bskLogger.bskError("LinearTranslationNDOFStateEffector: at least one translating body is required.");
        return;
    }

    for (const auto& translatingBody : this->translatingBodyVec) {
        if (!eigenIsRotationMatrix(translatingBody->dcm_FP)) {
            bskLogger.bskError(
              "LinearTranslationNDOFStateEffector: a translating body's dcm_FP is not a valid rotation matrix; it must "
              "be orthogonal and right-handed. It may not have been set properly by the user.");
        }
        // a massless body legitimately carries a zero inertia tensor, so only check when its mass > 0
        if (translatingBody->mass > 0.0 && !eigenIsValidInertiaMatrix(translatingBody->IPntFc_F)) {
            bskLogger.bskError("LinearTranslationNDOFStateEffector: a translating body's IPntFc_F is not a valid "
                               "inertia tensor. It may not have been set properly by the user.");
        }
    }
}

/*! This method checks that the joint mass matrix the equations of motion invert is not singular */
void
LinearTranslationNDOFStateEffector::checkJointMassMatrix()
{
    // the axes are fixed in their parents and a translating body does not rotate, so the matrix
    // never changes during the integration and the equations of motion can build it here
    Eigen::Matrix3d dcm_FB = Eigen::Matrix3d::Identity();
    for (auto& translatingBody : this->translatingBodyVec) {
        translatingBody->fHat_B = dcm_FB.transpose() * translatingBody->fHat_P;
        dcm_FB = translatingBody->dcm_FP * dcm_FB;
    }

    Eigen::MatrixXd MRho = Eigen::MatrixXd::Zero(this->N, this->N);
    this->computeMRho(MRho);
    if (Eigen::FullPivLU<Eigen::MatrixXd>(MRho).rank() < this->N) {
        bskLogger.bskError("LinearTranslationNDOFStateEffector: the translating body masses and axes leave the joint "
                           "mass matrix singular because at least one nonzero combination of joint rates leaves every "
                           "mass-bearing body stationary.");
    }
}

/*! This method allows the TB state effector to register its states: rho and rhoDot with the dynamic parameter manager */
void LinearTranslationNDOFStateEffector::registerStates(DynParamManager& states)
{
    // Register the rho states
    this->rhoState = states.registerState(static_cast<uint32_t>(N), 1, this->nameOfRhoState);
    this->rhoDotState = states.registerState(static_cast<uint32_t>(N), 1, this->nameOfRhoDotState);
    Eigen::MatrixXd RhoInitMatrix(N,1);
    Eigen::MatrixXd RhoDotInitMatrix(N,1);
    int i = 0;
    for(const auto& translatingBody: this->translatingBodyVec) {
        RhoInitMatrix(i,0) = translatingBody->rhoInit;
        RhoDotInitMatrix(i,0) = translatingBody->rhoDotInit;
        i++;
    }
    this->rhoState->setState(RhoInitMatrix);
    this->rhoDotState->setState(RhoDotInitMatrix);

    this->registerProperties(states);
    this->validateConfiguration();
}

/*! This method attaches a dynamicEffector to one of the translating bodies

 @param newDynamicEffector the dynamic effector to be attached
 @param segment the translating body to attach to, counting outward from the hub starting at 1 */
void
LinearTranslationNDOFStateEffector::addDynamicEffector(DynamicEffector* newDynamicEffector, int segment)
{
    if (segment <= 0 || segment > this->N) {
        bskLogger.bskError("LinearTranslationNDOFStateEffector: specifying attachment to a "
                           "non-existent translating body.");
        return;
    }

    auto& translatingBody = this->translatingBodyVec[static_cast<size_t>(segment - 1)];
    translatingBody->assignStateParamNames<DynamicEffector*>(newDynamicEffector);
    translatingBody->dynEffectors.push_back(newDynamicEffector);
}

/*! This method registers each translating body's inertial properties with the dynamic parameter
 manager and links them into dependent dynamic effectors

 @param states the dynamic parameter manager holding the published properties */
void
LinearTranslationNDOFStateEffector::registerProperties(DynParamManager& states)
{
    const Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    for (auto& translatingBody : this->translatingBodyVec) {
        translatingBody->r_FN_N = states.createProperty(translatingBody->nameOfInertialPositionProperty, stateInit);
        translatingBody->v_FN_N = states.createProperty(translatingBody->nameOfInertialVelocityProperty, stateInit);
        translatingBody->sigma_FN = states.createProperty(translatingBody->nameOfInertialAttitudeProperty, stateInit);
        translatingBody->omega_FN_F =
          states.createProperty(translatingBody->nameOfInertialAngVelocityProperty, stateInit);

        for (auto& dynEffector : translatingBody->dynEffectors) {
            dynEffector->linkInProperties(states);
        }
    }
}

/*! This method allows the TB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void LinearTranslationNDOFStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]])
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B = Eigen::Vector3d::Zero();
    this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();
    this->effProps.IEffPntB_B = Eigen::Matrix3d::Zero();
    this->effProps.IEffPrimePntB_B = Eigen::Matrix3d::Zero();

    int i = 0;
    for(auto& translatingBody: this->translatingBodyVec) {
        const size_t bodyIndex = static_cast<size_t>(i);
        if (translatingBody->isAxisLocked) {
            auto rhoDotVector = this->rhoDotState->getState();
            rhoDotVector(i) = 0.0;
            this->rhoDotState->setState(rhoDotVector);
        }
        // Give the mass of the translating body to the effProps mass
        this->effProps.mEff += translatingBody->mass;

        // Grab current states
        translatingBody->rho = this->rhoState->getStateReference()(i, 0);
        translatingBody->rhoDot = this->rhoDotState->getStateReference()(i, 0);

        // Write the translating axis in B frame
        if (i == 0) {
            translatingBody->dcm_FB = translatingBody->dcm_FP;
            translatingBody->fHat_B = translatingBody->fHat_P;
        } else {
            const size_t previousBodyIndex = bodyIndex - 1;
            translatingBody->dcm_FB = translatingBody->dcm_FP
                * this->translatingBodyVec[previousBodyIndex]->dcm_FB;
            translatingBody->fHat_B = this->translatingBodyVec[previousBodyIndex]->dcm_FB.transpose()
                * translatingBody->fHat_P;
        }
        translatingBody->r_FF0_B = translatingBody->rho * translatingBody->fHat_B;

        // Compute the effector's CoM with respect to point B
        translatingBody->r_FcF_B = translatingBody->dcm_FB.transpose() * translatingBody->r_FcF_F;
        if (i == 0) {
            // The parent frame of first body is the B frame
            translatingBody->r_F0P_B = translatingBody->r_F0P_P;
            translatingBody->r_FP_B = translatingBody->r_F0P_B + translatingBody->r_FF0_B;
            translatingBody->r_FB_B = translatingBody->r_FP_B;
        } else {
            const size_t previousBodyIndex = bodyIndex - 1;
            translatingBody->r_F0P_B = this->translatingBodyVec[previousBodyIndex]->dcm_FB.transpose()
                * translatingBody->r_F0P_P;
            translatingBody->r_FP_B = translatingBody->r_F0P_B + translatingBody->r_FF0_B;
            translatingBody->r_FB_B = translatingBody->r_FP_B
                + this->translatingBodyVec[previousBodyIndex]->r_FB_B;
        }
        translatingBody->r_FcB_B = translatingBody->r_FcF_B + translatingBody->r_FB_B;
        this->effProps.rEff_CB_B += translatingBody->mass * translatingBody->r_FcB_B;

        // Find the inertia of the bodies about point B
        translatingBody->rTilde_FcB_B = eigenTilde(translatingBody->r_FcB_B);
        translatingBody->IPntFc_B = translatingBody->dcm_FB.transpose() * translatingBody->IPntFc_F * translatingBody->dcm_FB;
        this->effProps.IEffPntB_B += translatingBody->IPntFc_B - translatingBody->mass * translatingBody->rTilde_FcB_B * translatingBody->rTilde_FcB_B;

        // Find rPrime_FcB_B
        translatingBody->rPrime_FcF_B = Eigen::Vector3d::Zero();
        translatingBody->rPrime_FF0_B = translatingBody->rhoDot * translatingBody->fHat_B;
        translatingBody->rPrime_FP_B = translatingBody->rPrime_FF0_B;
        if (i == 0) {
            translatingBody->rPrime_FB_B = translatingBody->rPrime_FP_B;
        } else {
            translatingBody->rPrime_FB_B = translatingBody->rPrime_FP_B
                + this->translatingBodyVec[bodyIndex - 1]->rPrime_FB_B;
        }
        translatingBody->rPrime_FcB_B = translatingBody->rPrime_FcF_B + translatingBody->rPrime_FB_B;
        this->effProps.rEffPrime_CB_B += translatingBody->mass * translatingBody->rPrime_FcB_B;

        // Find the body-frame time derivative of the inertia of each arm and the entire spacecraft
        translatingBody->IPrimePntFc_B = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d rPrimeTilde_FcB_B = eigenTilde(translatingBody->rPrime_FcB_B);
        this->effProps.IEffPrimePntB_B += translatingBody->IPrimePntFc_B - translatingBody->mass * (rPrimeTilde_FcB_B * translatingBody->rTilde_FcB_B + translatingBody->rTilde_FcB_B * rPrimeTilde_FcB_B);

        i++;
    }
    this->effProps.rEff_CB_B /= this->effProps.mEff;
    this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
}

/*! This method allows the TB state effector to give its contributions to the matrices needed for the back-sub
 method */
void LinearTranslationNDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // Find the DCM from N to B frames
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
    this->omega_BN_B = omega_BN_B;

    Eigen::MatrixXd MRho = Eigen::MatrixXd::Zero(this->N, this->N);
    Eigen::MatrixX3d ARhoStar = Eigen::MatrixX3d::Zero(this->N, 3);
    Eigen::MatrixX3d BRhoStar = Eigen::MatrixX3d::Zero(this->N, 3);
    Eigen::VectorXd CRhoStar = Eigen::VectorXd::Zero(this->N);

    bool hasAttachedEffectors = false;
    for (const auto& translatingBody : this->translatingBodyVec) {
        if (!translatingBody->dynEffectors.empty()) {
            hasAttachedEffectors = true;
            break;
        }
    }
    if (hasAttachedEffectors) {
        this->computeTranslatingBodyInertialStates();
    }
    this->computeDependentEffectors(backSubContr, integTime);

    this->computeMRho(MRho);
    this->computeARhoStar(ARhoStar);
    this->computeBRhoStar(BRhoStar);
    this->computeCRhoStar(CRhoStar, g_N);

    Eigen::MatrixXd MRhoInv = MRho.inverse();
    this->ARho = MRhoInv * ARhoStar;
    this->BRho = MRhoInv * BRhoStar;
    this->CRho = MRhoInv * CRhoStar;

    this->computeBackSubContributions(backSubContr);
}

/*! This method collects the loads from any attached dynamic effectors and applies them to the hub

 @param backSubContr the Backsubstitution contributions this effector adds to the hub
 @param integTime the integration time the attached effectors evaluate their loads at */
void
LinearTranslationNDOFStateEffector::computeDependentEffectors(BackSubMatrices& backSubContr, double integTime)
{
    for (auto& translatingBody : this->translatingBodyVec) {
        translatingBody->extForce_B.setZero();
        translatingBody->extTorquePntF_B.setZero();
        for (auto& dynEffector : translatingBody->dynEffectors) {
            dynEffector->computeForceTorque(integTime, double(0.0));
            // a child's "_B" loads are already in this body's F frame, so only "_N" is rotated
            translatingBody->extForce_B += translatingBody->dcm_FB.transpose() * dynEffector->forceExternal_B +
                                           this->dcm_BN * dynEffector->forceExternal_N;
            translatingBody->extTorquePntF_B += translatingBody->dcm_FB.transpose() * dynEffector->torqueExternalPntB_B;
        }

        backSubContr.vecTrans += translatingBody->extForce_B;
        backSubContr.vecRot +=
          translatingBody->extTorquePntF_B + translatingBody->r_FB_B.cross(translatingBody->extForce_B);
    }
}

/*! This method compute MRho for back-sub */
void LinearTranslationNDOFStateEffector::computeMRho(Eigen::MatrixXd& MRho)
{
    for (int n = 0; n<this->N; n++) {
        const size_t nIndex = static_cast<size_t>(n);
        for (int i = 0; i<this->N; i++) {
            const size_t iIndex = static_cast<size_t>(i);
            MRho(n,i) = 0.0;
            if ((this->translatingBodyVec[nIndex]->isAxisLocked
                 || this->translatingBodyVec[iIndex]->isAxisLocked) && n != i)
                continue;
            for (int j = (i<=n) ? n : i; j<this->N; j++) {
                MRho(n,i) += this->translatingBodyVec[nIndex]->fHat_B.transpose()
                    * this->translatingBodyVec[static_cast<size_t>(j)]->mass
                    * this->translatingBodyVec[iIndex]->fHat_B;
            }
        }
    }
}

/*! This method compute ARhoStar for back-sub */
void
LinearTranslationNDOFStateEffector::computeARhoStar(Eigen::MatrixX3d& ARhoStar)
{
    for (int n = 0; n<this->N; n++) {
        const size_t nIndex = static_cast<size_t>(n);
        if (this->translatingBodyVec[nIndex]->isAxisLocked)
            continue;
        for (int i = n; i<this->N; i++) {
            ARhoStar.row(n) -= this->translatingBodyVec[nIndex]->fHat_B.transpose()
                * this->translatingBodyVec[static_cast<size_t>(i)]->mass;
        }
    }
}

/*! This method compute BRhoStar for back-sub */
void
LinearTranslationNDOFStateEffector::computeBRhoStar(Eigen::MatrixX3d& BRhoStar)
{
    for (int n = 0; n<this->N; n++) {
        const size_t nIndex = static_cast<size_t>(n);
        if (this->translatingBodyVec[nIndex]->isAxisLocked)
            continue;
        for (int i = n; i<this->N; i++) {
            const size_t iIndex = static_cast<size_t>(i);
            Eigen::Vector3d r_FciB_B = this->translatingBodyVec[iIndex]->r_FcB_B;

            BRhoStar.row(n) += this->translatingBodyVec[iIndex]->mass
                                * this->translatingBodyVec[nIndex]->fHat_B.cross(r_FciB_B).transpose();
        }
    }
}

/*! This method compute CRhoStar for back-sub */
void LinearTranslationNDOFStateEffector::computeCRhoStar(Eigen::VectorXd& CRhoStar,
                                                      const Eigen::Vector3d& g_N)
{
    // Map gravity to body frame
    Eigen::Vector3d g_B;
    g_B = this->dcm_BN * g_N;
    Eigen::Vector3d F_g = Eigen::Vector3d::Zero().transpose();

    for (int n = 0; n<this->N; n++) {
        const size_t nIndex = static_cast<size_t>(n);
        if (this->translatingBodyVec[nIndex]->isAxisLocked)
            continue;
        CRhoStar(n, 0) = this->translatingBodyVec[nIndex]->u
                           - this->translatingBodyVec[nIndex]->k * (this->translatingBodyVec[nIndex]->rho -
                           this->translatingBodyVec[nIndex]->rhoRef) - this->translatingBodyVec[nIndex]->c *
                           (this->translatingBodyVec[nIndex]->rhoDot - this->translatingBodyVec[nIndex]->rhoDotRef);
        for (int i = n; i<this->N; i++) {
            const size_t iIndex = static_cast<size_t>(i);
            Eigen::Vector3d r_FciB_B = this->translatingBodyVec[iIndex]->r_FcB_B;
            Eigen::Vector3d rPrime_FciB_B = this->translatingBodyVec[iIndex]->rPrime_FcB_B;

            F_g = this->translatingBodyVec[iIndex]->mass * g_B;
            // a torque does no work through a prismatic joint, so only the force reaches this equation
            CRhoStar(n, 0) += this->translatingBodyVec[nIndex]->fHat_B.transpose()
                * (F_g + this->translatingBodyVec[iIndex]->extForce_B
                       - this->translatingBodyVec[iIndex]->mass *
                              (this->omega_BN_B.cross(this->omega_BN_B.cross(r_FciB_B))
                               + 2 * this->omega_BN_B.cross(rPrime_FciB_B)));
        }
    }
}

/*! This method computes the back-sub contributions of the system */
void LinearTranslationNDOFStateEffector::computeBackSubContributions(BackSubMatrices& backSubContr) const
{
    for (int i = 0; i<this->N; i++) {
        const size_t iIndex = static_cast<size_t>(i);
        Eigen::Vector3d r_FciB_B = this->translatingBodyVec[iIndex]->r_FcB_B;
        Eigen::Vector3d rPrime_FciB_B = this->translatingBodyVec[iIndex]->rPrime_FcB_B;
        backSubContr.vecRot -= this->translatingBodyVec[iIndex]->mass
                               * this->omega_BN_B.cross(r_FciB_B.cross(rPrime_FciB_B));
        for (int j = i; j < this->N; j++) {
            const size_t jIndex = static_cast<size_t>(j);
            Eigen::Vector3d rCrossFHat_B = this->translatingBodyVec[jIndex]->r_FcB_B.cross(
                this->translatingBodyVec[iIndex]->fHat_B);

            // Translation contributions
            backSubContr.matrixA += this->translatingBodyVec[jIndex]->mass *  this->translatingBodyVec[iIndex]->fHat_B * this->ARho.row(i);
            backSubContr.matrixB += this->translatingBodyVec[jIndex]->mass *  this->translatingBodyVec[iIndex]->fHat_B * this->BRho.row(i);
            backSubContr.vecTrans -= this->translatingBodyVec[jIndex]->mass * this->translatingBodyVec[iIndex]->fHat_B * this->CRho.row(i);

            // Rotation contributions
            backSubContr.matrixC += this->translatingBodyVec[jIndex]->mass * rCrossFHat_B * this->ARho.row(i);
            backSubContr.matrixD += this->translatingBodyVec[jIndex]->mass * rCrossFHat_B * this->BRho.row(i);
            backSubContr.vecRot -= this->translatingBodyVec[jIndex]->mass * rCrossFHat_B * this->CRho.row(i);
        }
    }
}

/*! This method is used to find the derivatives for the TB stateEffector: rhoDDot and the kinematic derivative */
void LinearTranslationNDOFStateEffector::computeDerivatives(double integTime [[maybe_unused]], Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN [[maybe_unused]])
{
    // Find rDDotLoc_BN_B
    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
    Eigen::Vector3d rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;

    // Compute rho and rhoDot derivatives
    Eigen::VectorXd rhoDDot = this->ARho * rDDotLocal_BN_B + this->BRho * omegaDot_BN_B + this->CRho;
    this->rhoState->setDerivative(this->rhoDotState->getStateReference());
    this->rhoDotState->setDerivative(rhoDDot);
}

/*! This method is for calculating the contributions of the TB state effector to the energy and momentum of the spacecraft */
void LinearTranslationNDOFStateEffector::updateEnergyMomContributions(double integTime [[maybe_unused]],
                                                                      Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                      double & rotEnergyContr,
                                                                      Eigen::Vector3d omega_BN_B)
{
    this->omega_BN_B = omega_BN_B;

    rotAngMomPntCContr_B = Eigen::Vector3d::Zero();
    rotEnergyContr = 0.0;

    for(auto& translatingBody: this->translatingBodyVec) {
        // Compute rDot_FcB_B, noting that the F frame shares the hub angular velocity
        translatingBody->rDot_FcB_B = translatingBody->rPrime_FcB_B
                                      + this->omega_BN_B.cross(translatingBody->r_FcB_B);

        // Find rotational angular momentum contribution from hub
        rotAngMomPntCContr_B += translatingBody->IPntFc_B * this->omega_BN_B
                                + translatingBody->mass * translatingBody->r_FcB_B.cross(translatingBody->rDot_FcB_B);

        // Find rotational energy contribution from the hub
        rotEnergyContr += 1.0 / 2.0 * this->omega_BN_B.dot(translatingBody->IPntFc_B * this->omega_BN_B)
                        + 1.0 / 2.0 * translatingBody->mass * translatingBody->rDot_FcB_B.dot(translatingBody->rDot_FcB_B)
                        + 1.0 / 2.0 * translatingBody->k * (translatingBody->rho - translatingBody->rhoRef) *
                        (translatingBody->rho - translatingBody->rhoRef);
    }
}

/*! This method computes the translating body states relative to the inertial frame */
void LinearTranslationNDOFStateEffector::computeTranslatingBodyInertialStates()
{
    // - read live: the cached copy lags an integrator substep at write time
    const Eigen::MRPd sigmaHub_BN(this->hubSigmaState->getStateReference().data());
    this->dcm_BN = sigmaHub_BN.toRotationMatrix().transpose();

    const Eigen::Vector3d r_BN_N = (Eigen::Vector3d)*this->inertialPositionProperty;
    const Eigen::Vector3d v_BN_N = (Eigen::Vector3d)*this->inertialVelocityProperty;

    for(auto& translatingBody: this->translatingBodyVec) {
        // Compute the rotational properties, noting that F does not rotate relative to B
        const Eigen::Matrix3d dcm_FN = translatingBody->dcm_FB * this->dcm_BN;
        *translatingBody->sigma_FN = eigenC2MRP(dcm_FN).coeffs();
        *translatingBody->omega_FN_F = translatingBody->dcm_FB * this->omega_BN_B;

        // Compute the translation properties of the center of mass Fc and of the frame origin F
        translatingBody->rDot_FcB_B = translatingBody->rPrime_FcB_B + this->omega_BN_B.cross(translatingBody->r_FcB_B);
        translatingBody->r_FcN_N = r_BN_N + this->dcm_BN.transpose() * translatingBody->r_FcB_B;
        translatingBody->v_FcN_N = v_BN_N + this->dcm_BN.transpose() * translatingBody->rDot_FcB_B;

        *translatingBody->r_FN_N = r_BN_N + this->dcm_BN.transpose() * translatingBody->r_FB_B;
        *translatingBody->v_FN_N =
          v_BN_N +
          this->dcm_BN.transpose() * (translatingBody->rPrime_FB_B + this->omega_BN_B.cross(translatingBody->r_FB_B));
    }
}

/*! This method is used so that the simulation will ask TB to update messages */
void LinearTranslationNDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessages();
    this->writeOutputStateMessages(CurrentSimNanos);
}
