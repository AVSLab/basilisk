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

#include "nHingedRigidBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

/*! This is the constructor, setting variables to default values */
NHingedRigidBodyStateEffector::NHingedRigidBodyStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    this->r_HB_B.setZero();
    this->dcm_HB.setIdentity();
    this->nameOfThetaState ="nHingedRigidBody" + std::to_string(this->effectorID) + "Theta";
    this->nameOfThetaDotState = "nHingedRigidBody" + std::to_string(this->effectorID) + "ThetaDot";
    this->propertyNameIndex = std::to_string(this->effectorID); // preserves effectorID for later adding panels
    this->effectorID++;

    return;
}

uint64_t NHingedRigidBodyStateEffector::effectorID = 1;

/*! This is the destructor, releasing the per panel output messages */
NHingedRigidBodyStateEffector::~NHingedRigidBodyStateEffector()
{
    for (size_t c = 0; c < this->nHingedRigidBodyOutMsgs.size(); c++) {
        delete this->nHingedRigidBodyOutMsgs.at(c);
        delete this->nHingedRigidBodyConfigLogOutMsgs.at(c);
    }

    return;
}

/*! This method appends a panel to the chain along with its output messages

 @param NewPanel the panel to append to the chain
 */
void
NHingedRigidBodyStateEffector::addHingedPanel(HingedPanel NewPanel)
{
    this->PanelVec.push_back(NewPanel);
    this->nHingedRigidBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);
    this->nHingedRigidBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);

    const std::string panelSuffix = this->propertyNameIndex + "_" + std::to_string(this->PanelVec.size());
    HingedPanel& panel = this->PanelVec.back();
    panel.nameOfInertialPositionProperty = "nHingedRigidBodyInertialPosition" + panelSuffix;
    panel.nameOfInertialVelocityProperty = "nHingedRigidBodyInertialVelocity" + panelSuffix;
    panel.nameOfInertialAttitudeProperty = "nHingedRigidBodyInertialAttitude" + panelSuffix;
    panel.nameOfInertialAngVelocityProperty = "nHingedRigidBodyInertialAngVelocity" + panelSuffix;

    return;
}

/*! This method takes the computed theta states and outputs them to the messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void
NHingedRigidBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    this->computePanelInertialStates();

    size_t panelIndex = 0;
    std::vector<HingedPanel>::iterator PanelIt;
    for (PanelIt = this->PanelVec.begin(); PanelIt != this->PanelVec.end(); PanelIt++) {
        if (this->nHingedRigidBodyOutMsgs[panelIndex]->isLinked()) {
            HingedRigidBodyMsgPayload panelBuffer = this->nHingedRigidBodyOutMsgs[panelIndex]->zeroMsgPayload;
            panelBuffer.theta = PanelIt->theta;
            panelBuffer.thetaDot = PanelIt->thetaDot;
            this->nHingedRigidBodyOutMsgs[panelIndex]->write(&panelBuffer, this->moduleID, CurrentClock);
        }

        if (this->nHingedRigidBodyConfigLogOutMsgs[panelIndex]->isLinked()) {
            SCStatesMsgPayload configLogMsg = this->nHingedRigidBodyConfigLogOutMsgs[panelIndex]->zeroMsgPayload;
            // Note, logging the panel frame S is the body frame B of that object
            eigenVector3d2CArray(PanelIt->r_ScN_N, configLogMsg.r_BN_N);
            eigenVector3d2CArray(PanelIt->v_ScN_N, configLogMsg.v_BN_N);
            eigenMatrixXd2CArray(*PanelIt->sigma_SN, configLogMsg.sigma_BN);
            eigenMatrixXd2CArray(*PanelIt->omega_SN_S, configLogMsg.omega_BN_B);
            this->nHingedRigidBodyConfigLogOutMsgs[panelIndex]->write(&configLogMsg, this->moduleID, CurrentClock);
        }
        panelIndex++;
    }

    return;
}

/*! This method computes the panel states relative to the inertial frame */
void
NHingedRigidBodyStateEffector::computePanelInertialStates()
{
    // - read live: the cached copy lags half a step at write time, unless a prescribed body set it
    if (this->prescribedAttitudeProperty == nullptr) {
        this->sigma_BN = Eigen::MRPd(this->hubSigmaState->getStateReference().data());
    }
    Eigen::MRPd sigmaLocal_BN = this->sigma_BN;
    Eigen::Matrix3d dcm_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Vector3d r_BN_N = (Eigen::Vector3d)(*this->inertialPositionProperty);
    Eigen::Vector3d v_BN_N = (Eigen::Vector3d)(*this->inertialVelocityProperty);

    std::vector<HingedPanel>::iterator PanelIt;
    for (PanelIt = this->PanelVec.begin(); PanelIt != this->PanelVec.end(); PanelIt++) {
        const Eigen::MRPd sigma_SN = eigenC2MRP(PanelIt->dcm_SB * dcm_NB.transpose());
        *PanelIt->sigma_SN = sigma_SN.coeffs();
        *PanelIt->omega_SN_S = PanelIt->dcm_SB * (this->omegaLoc_BN_B + PanelIt->omega_SB_B);

        PanelIt->r_ScN_N = r_BN_N + dcm_NB * PanelIt->r_SB_B;
        PanelIt->v_ScN_N = v_BN_N + dcm_NB * (PanelIt->rPrime_SB_B + this->omegaLoc_BN_B.cross(PanelIt->r_SB_B));

        *PanelIt->r_HN_N = r_BN_N + dcm_NB * PanelIt->r_HB_B;
        *PanelIt->v_HN_N = v_BN_N + dcm_NB * (PanelIt->rPrime_HB_B + this->omegaLoc_BN_B.cross(PanelIt->r_HB_B));
    }

    return;
}

/*! This method allows the HRB state effector to have access to the hub states and gravity
 *
 * @param[in] states Dynamic parameter manager containing the required states.
 */
void NHingedRigidBodyStateEffector::linkInStates(DynParamManager& states)
{
    // - Get access to the hub states
    this->g_N = states.getPropertyReference(this->propName_vehicleGravity);

    this->inertialPositionProperty =
      states.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty =
      states.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);
    this->hubSigmaState = states.getStateObject(this->nameOfSpacecraftAttachedTo + this->stateNameOfSigma);

    return;
}

/*! This method checks that the panel chain is one the equations of motion can represent */
void
NHingedRigidBodyStateEffector::checkPanelUniformity()
{
    if (this->PanelVec.empty()) {
        this->bskLogger.bskError("NHingedRigidBodyStateEffector: at least one hinged panel is required.");
    }

    // the equations of motion factor one panel mass and one half-length out of the sums over the
    // chain, so an uneven chain integrates silently wrong dynamics rather than failing
    const double relativeTolerance = 1e-9; // [-]
    const double mass = this->PanelVec.front().mass;
    const double d = this->PanelVec.front().d;
    std::vector<HingedPanel>::iterator PanelIt;
    for (PanelIt = this->PanelVec.begin(); PanelIt != this->PanelVec.end(); PanelIt++) {
        if (PanelIt->mass <= 0.0) {
            this->bskLogger.bskError("NHingedRigidBodyStateEffector: every panel mass must be greater than 0.");
        }
        if (std::abs(PanelIt->mass - mass) > relativeTolerance * std::abs(mass) ||
            std::abs(PanelIt->d - d) > relativeTolerance * std::abs(d)) {
            this->bskLogger.bskError("NHingedRigidBodyStateEffector: every panel must carry the same "
                                     "mass and the same hinge to center of mass distance d. Model an "
                                     "uneven chain with dualHingedRigidBodyStateEffector or "
                                     "spinningBodyNDOFStateEffector instead.");
        }
    }
}

/*! This method allows the HRB state effector to register its states: theta and thetaDot with the dyn param manager
 *
 * @param[in,out] statesIn Dynamic parameter manager used to register states or properties.
 */
void NHingedRigidBodyStateEffector::registerStates(DynParamManager& statesIn)
{
    this->checkPanelUniformity();

    // - Register the states associated with hinged rigid bodies - theta and thetaDot
    Eigen::MatrixXd thetaInitMatrix(this->PanelVec.size(),1);
    Eigen::MatrixXd thetaDotInitMatrix(this->PanelVec.size(),1);
    std::vector<HingedPanel>::iterator PanelIt;
    int it = 0;
    this->totalMass = 0;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        thetaInitMatrix(it,0) = PanelIt->thetaInit;
        thetaDotInitMatrix(it,0) = PanelIt->thetaDotInit;
        // - Looping over hinged rigid bodies to find total mass
        this->totalMass += PanelIt->mass;
        it += 1;
    }
    this->thetaState = statesIn.registerState((uint32_t) this->PanelVec.size(), 1, this->nameOfThetaState);
    this->thetaState->setState(thetaInitMatrix);
    this->thetaDotState = statesIn.registerState((uint32_t) this->PanelVec.size(), 1, this->nameOfThetaDotState);
    this->thetaDotState->setState(thetaDotInitMatrix);

    registerProperties(statesIn);

    return;
}

/*! This method attaches a dynamicEffector to one of the panels

 @param newDynamicEffector the dynamic effector to be attached
 @param segment the panel to attach to, counting outward from the hub starting at 1 */
void
NHingedRigidBodyStateEffector::addDynamicEffector(DynamicEffector* newDynamicEffector, int segment)
{
    if (segment <= 0 || segment > (int)this->PanelVec.size()) {
        this->bskLogger.bskError("NHingedRigidBodyStateEffector: specifying attachment to a non-existent panel.");
    }

    HingedPanel& panel = this->PanelVec[(size_t)(segment - 1)];
    panel.assignStateParamNames<DynamicEffector*>(newDynamicEffector);
    panel.dynEffectors.push_back(newDynamicEffector);
    this->hasAttachedEffectors = true;
}

/*! This method registers the panel inertial properties with the dynamic parameter manager and links
 them into dependent dynamic effectors
 *
 * @param[in,out] states Dynamic parameter manager used to register states or properties.
 */
void
NHingedRigidBodyStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    std::vector<HingedPanel>::iterator PanelIt;
    for (PanelIt = this->PanelVec.begin(); PanelIt != this->PanelVec.end(); PanelIt++) {
        PanelIt->r_HN_N = states.createProperty(PanelIt->nameOfInertialPositionProperty, stateInit);
        PanelIt->v_HN_N = states.createProperty(PanelIt->nameOfInertialVelocityProperty, stateInit);
        PanelIt->sigma_SN = states.createProperty(PanelIt->nameOfInertialAttitudeProperty, stateInit);
        PanelIt->omega_SN_S = states.createProperty(PanelIt->nameOfInertialAngVelocityProperty, stateInit);

        std::vector<DynamicEffector*>::iterator dynIt;
        for (dynIt = PanelIt->dynEffectors.begin(); dynIt != PanelIt->dynEffectors.end(); dynIt++) {
            (*dynIt)->linkInProperties(states);
        }
    }

    return;
}

/*! This method allows the HRB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft
 *
 * @param[in] integTime [s] Current integration time.
 */
void NHingedRigidBodyStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]])
{
    // - Define summation variables
    double sum_Theta = 0;
    double sum_ThetaDot = 0;
    double sum_mass = 0;
    Eigen::Vector3d sum_COM;
    sum_COM.setZero();
    Eigen::Vector3d sum_COMprime;
    sum_COMprime.setZero();
    Eigen::Matrix3d sum_PanelInertia;
    sum_PanelInertia.setZero();
    Eigen::Matrix3d sum_EffInertia;
    sum_EffInertia.setZero();
    Eigen::Vector3d sum_rH;
    sum_rH.setZero();
    Eigen::Vector3d sum_rPrimeH;
    sum_rPrimeH.setZero();

    const Eigen::MatrixXd& thetaVector = this->thetaState->getStateReference();
    const Eigen::MatrixXd& thetaDotVector = this->thetaDotState->getStateReference();
    std::vector<HingedPanel>::iterator PanelIt;
    int it = 0;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){

        // - Find hinged rigid bodies' position with respect to point B
        // - First need to grab current states
        PanelIt->theta = thetaVector(it, 0);
        PanelIt->thetaDot = thetaDotVector(it, 0);
        // - Next find the sHat unit vectors
        sum_Theta += PanelIt->theta;
        PanelIt->dcm_SB = eigenM2(sum_Theta)*this->dcm_HB;
        PanelIt->sHat1_B = PanelIt->dcm_SB.row(0);
        PanelIt->sHat2_B = PanelIt->dcm_SB.row(1);
        PanelIt->sHat3_B = PanelIt->dcm_SB.row(2);

        PanelIt->r_HB_B = this->r_HB_B + sum_rH;
        PanelIt->r_SB_B = this->r_HB_B - PanelIt->d*PanelIt->sHat1_B + sum_rH;
        sum_rH += -2*PanelIt->d*PanelIt->sHat1_B;

        // - Define rTilde_SB_B
        PanelIt->rTilde_SB_B = eigenTilde(PanelIt->r_SB_B);

        // - Find rPrime_SB_B
        sum_ThetaDot += PanelIt->thetaDot;
        PanelIt->rPrime_HB_B = PanelIt->d * sum_rPrimeH;
        PanelIt->rPrime_SB_B = PanelIt->d*(sum_ThetaDot*PanelIt->sHat3_B + sum_rPrimeH);
        sum_rPrimeH += 2*PanelIt->sHat3_B*sum_ThetaDot;

        PanelIt->omega_SB_B = sum_ThetaDot*PanelIt->sHat2_B;

        // - Next find the body time derivative of the inertia about point B
        // - Define tilde matrix of rPrime_SB_B
        PanelIt->rPrimeTilde_SB_B = eigenTilde(PanelIt->rPrime_SB_B);

        // - Find body time derivative of IPntS_B
        PanelIt->ISPrimePntS_B = sum_ThetaDot*(PanelIt->IPntS_S(2,2) - PanelIt->IPntS_S(0,0))
                       *(PanelIt->sHat1_B*PanelIt->sHat3_B.transpose() + PanelIt->sHat3_B*PanelIt->sHat1_B.transpose());

        // - Mass summation
        sum_mass += PanelIt->mass;

        // - Inertia of the panels summation term
        sum_PanelInertia += PanelIt->dcm_SB.transpose()*PanelIt->IPntS_S*PanelIt->dcm_SB
            + PanelIt->mass*PanelIt->rTilde_SB_B*PanelIt->rTilde_SB_B.transpose();

        // - COM position summation terms
        sum_COM += PanelIt->mass*PanelIt->r_SB_B;

        sum_COMprime += PanelIt->mass*PanelIt->rPrime_SB_B;

        // - Inertia Prime of the effector summation terms
        sum_EffInertia += PanelIt->ISPrimePntS_B - PanelIt->mass*(PanelIt->rPrimeTilde_SB_B*PanelIt->rTilde_SB_B
                                                                  + PanelIt->rTilde_SB_B*PanelIt->rPrimeTilde_SB_B);

        it += 1;
    }

    // - update effector mass properties
    this->effProps.mEff = sum_mass;

    // - update effector COM location
    this->effProps.rEff_CB_B = 1.0/this->effProps.mEff*sum_COM;

    this->effProps.rEffPrime_CB_B = 1.0/this->effProps.mEff*sum_COMprime;

    // - Find the inertia of the hinged rigid body about point B
    this->effProps.IEffPntB_B = sum_PanelInertia;

    // - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = sum_EffInertia;

    return;
}

/*! @brief Define the Heaviside function used by the equations of motion.
 *
 * @param[in] cond Condition evaluated by the Heaviside function.
 */
double NHingedRigidBodyStateEffector::HeaviFunc(double cond)
{
    double ans;
    if (cond < 0.0) ans = 0.0;
    else ans = 1.0;
    return ans;
}

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub
 method
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] backSubContr Backsubstitution contributions.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 * @param[in] g_N [m/s^2] Gravitational acceleration expressed in inertial-frame components.
 */
void NHingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N [[maybe_unused]])
{
    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    this->sigma_BN = sigma_BN;
    sigmaLocal_BN = this->sigma_BN;
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;

    // - Define omega_BN_S
    this->omegaLoc_BN_B = omega_BN_B;
    std::vector<HingedPanel>::iterator PanelIt;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        PanelIt->omega_BN_S = PanelIt->dcm_SB*this->omegaLoc_BN_B;
    }

    if (this->hasAttachedEffectors) {
        this->computePanelInertialStates();
    }

    // Loop through to collect forces and torques from any connected dynamic effectors
    Eigen::Vector3d attBodyForce_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d attBodyTorquePntB_B = Eigen::Vector3d::Zero();
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        PanelIt->extForce_B.setZero();
        PanelIt->extTorquePntH_B.setZero();
        std::vector<DynamicEffector*>::iterator dynIt;
        for(dynIt = PanelIt->dynEffectors.begin(); dynIt != PanelIt->dynEffectors.end(); dynIt++)
        {
            // - Compute the force and torque contributions from the dynamicEffectors
            (*dynIt)->computeForceTorque(integTime, double(0.0));
            // a child's "_B" loads are already in this panel's S frame, so only "_N" is rotated
            PanelIt->extForce_B += PanelIt->dcm_SB.transpose()*(*dynIt)->forceExternal_B
                                   + dcm_BN*(*dynIt)->forceExternal_N;
            PanelIt->extTorquePntH_B += PanelIt->dcm_SB.transpose()*(*dynIt)->torqueExternalPntB_B;
        }
        attBodyForce_B += PanelIt->extForce_B;
        attBodyTorquePntB_B += PanelIt->extTorquePntH_B + PanelIt->r_HB_B.cross(PanelIt->extForce_B);
    }

    // - Define A matrix for the panel equations
    std::vector<HingedPanel>::iterator PanelIt2;
    this->matrixADHRB.resize((int) this->PanelVec.size(), (int) this->PanelVec.size());
    this->matrixADHRB.setZero();
    std::vector<HingedPanel>::iterator PanelIt3;
    int j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        int i = 1;
        for(PanelIt2=this->PanelVec.begin(); PanelIt2!=this->PanelVec.end(); PanelIt2++){
            Eigen::Vector3d sumTerm1;
            sumTerm1.setZero();
            PanelIt3 = PanelIt2;
            for(int k = i; k<= (int) this->PanelVec.size();k++){
                sumTerm1 += 2*PanelIt3->sHat3_B+4*PanelIt3->sHat3_B*((int) this->PanelVec.size() - j)
                -HeaviFunc(k-j)*4*PanelIt3->sHat3_B*(k-j);
                std::advance(PanelIt3, 1);
            }
            this->matrixADHRB(j-1,i-1) =  PanelIt->IPntS_S(1,1)*HeaviFunc(j-i) + PanelIt->mass*pow(PanelIt->d,2)
            *PanelIt->sHat3_B.transpose()*(sumTerm1-HeaviFunc(j-i)*PanelIt->sHat3_B);
            i += 1;
        }
        j += 1;
    }

    this->matrixEDHRB = this->matrixADHRB.inverse();

    // - Define F matrix for the panel equations
    this->matrixFDHRB.resize((int) this->PanelVec.size(),3);
    this->matrixFDHRB.setZero();
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        Eigen::Vector3d sumTerm1;
        Eigen::Vector3d sumTerm2;
        sumTerm2.setZero();
        if(j+1<= (int) this->PanelVec.size()){
            for(int i = j+1; i <= (int) this->PanelVec.size(); i++){
                sumTerm2 += 2*PanelIt->mass*PanelIt->d*PanelIt->sHat3_B;
            }
        }
        sumTerm1 = - PanelIt->mass*PanelIt->d*PanelIt->sHat3_B.transpose() - sumTerm2.transpose();
        this->matrixFDHRB(j-1,0) = sumTerm1[0];
        this->matrixFDHRB(j-1,1) = sumTerm1[1];
        this->matrixFDHRB(j-1,2) = sumTerm1[2];
        j += 1;
    }

    // - Define G matrix for the panel equations
    this->matrixGDHRB.resize((int) this->PanelVec.size(),3);
    matrixGDHRB.setZero();
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        Eigen::Vector3d sumTerm1;
        Eigen::Vector3d sumTerm2;
        sumTerm2.setZero();
        if(j+1<= (int) this->PanelVec.size()){
            PanelIt2 = PanelIt;
            std::advance(PanelIt2, 1);
            for(int i = j+1; i<=(int) this->PanelVec.size();i++){
                sumTerm2 += 2*PanelIt->mass*PanelIt->d*PanelIt->sHat3_B.cross(PanelIt2->r_SB_B);
                std::advance(PanelIt2, 1);
            }
        }
        sumTerm1 = -(PanelIt->IPntS_S(1,1)*PanelIt->sHat2_B - PanelIt->mass*PanelIt->d
                     *PanelIt->sHat3_B.cross(PanelIt->r_SB_B) - sumTerm2);
        this->matrixGDHRB(j-1,0) = sumTerm1[0];
        this->matrixGDHRB(j-1,1) = sumTerm1[1];
        this->matrixGDHRB(j-1,2) = sumTerm1[2];
        j += 1;
    }

    // - Define v vector for the panel equations
    this->vectorVDHRB.resize((int) this->PanelVec.size());
    this->vectorVDHRB.setZero();
    double massOfCurrentPanelAndBefore = 0; // Summation of all of prior panels masses and the current panels mass
    Eigen::Vector3d extForceCurrentPanelAndBefore = Eigen::Vector3d::Zero();
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        // Add current panel to mass
        massOfCurrentPanelAndBefore += PanelIt->mass;
        extForceCurrentPanelAndBefore += PanelIt->extForce_B;
        double sumTerm1;
        Eigen::Vector3d sumTerm2;
        sumTerm2.setZero();
        Eigen::Vector3d sumTerm3;
        sumTerm3.setZero();
        double springTerm;
        PanelIt2 = PanelIt;
        if(j+1 <= (int) this->PanelVec.size()){
            std::advance(PanelIt2, 1);
            springTerm = -PanelIt->k*(PanelIt->theta-PanelIt->theta_0)-PanelIt->c*PanelIt->thetaDot
            + PanelIt2->k*(PanelIt2->theta - PanelIt2->theta_0) + PanelIt2->c*PanelIt2->thetaDot;
        } else {
            springTerm = -PanelIt->k*(PanelIt->theta-PanelIt->theta_0)-PanelIt->c*PanelIt->thetaDot;
        }
        if(j+1<=(int) this->PanelVec.size()){
            PanelIt3 = PanelIt;
            std::advance(PanelIt3, 1);
            for(int i = j+1; i <= (int) this->PanelVec.size(); i++){
                sumTerm2 += 4*this->omegaLoc_BN_B.cross(PanelIt3->rPrime_SB_B)
                +2*this->omegaLoc_BN_B.cross(this->omegaLoc_BN_B.cross(PanelIt3->r_SB_B));
                std::advance(PanelIt3, 1);
            }
        }
        double sumThetaDot = 0;
        int i = 1;
        for(PanelIt2=this->PanelVec.begin(); PanelIt2!=this->PanelVec.end(); PanelIt2++){
            sumThetaDot += PanelIt2->thetaDot;
            sumTerm3 += pow(sumThetaDot,2)*PanelIt2->d*(2*PanelIt2->sHat1_B+4*PanelIt2->sHat1_B*((int) this->PanelVec.size() - j)-HeaviFunc(i-j)*4*PanelIt2->sHat1_B*(i-j));
            i += 1;
        }
        sumThetaDot = 0;
        PanelIt2 = this->PanelVec.begin();
        for (int n = 1; n<=j; n++){
            sumThetaDot += PanelIt2->thetaDot;
            std::advance(PanelIt2, 1);
        }
        sumTerm3 -= pow(sumThetaDot, 2)*PanelIt->d*PanelIt->sHat1_B;
        sumTerm1 = springTerm -(PanelIt->IPntS_S(0,0) - PanelIt->IPntS_S(2,2))*PanelIt->omega_BN_S(2)
        *PanelIt->omega_BN_S(0) - PanelIt->mass*PanelIt->d*PanelIt->sHat3_B.dot(2*this->omegaLoc_BN_B.cross(PanelIt->rPrime_SB_B)
             +this->omegaLoc_BN_B.cross(this->omegaLoc_BN_B.cross(PanelIt->r_SB_B))+sumTerm2+sumTerm3);
        // Add gravity torque to this sumTerm
        Eigen::Vector3d gravTorqueCurPanel;
        gravTorqueCurPanel = -PanelIt->d*PanelIt->sHat1_B.cross(PanelIt->mass*g_B);
        Eigen::Vector3d gravForceRestOfPanels;
        double remainingMass;
        remainingMass = this->totalMass - massOfCurrentPanelAndBefore;
        gravForceRestOfPanels = remainingMass*g_B;
        // an attached effector's torque on a panel outboard of j cancels between the row j and row
        // j+1 balances, so only the moment arm of its force survives here
        Eigen::Vector3d extForceRestOfPanels = attBodyForce_B - extForceCurrentPanelAndBefore;
        this->vectorVDHRB(j-1) = sumTerm1 + PanelIt->sHat2_B.dot(gravTorqueCurPanel + PanelIt->extTorquePntH_B)
        + 2.0*PanelIt->d*PanelIt->sHat3_B.dot(gravForceRestOfPanels + extForceRestOfPanels);
        j += 1;
    }

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper

    // - translational contributions
    backSubContr.matrixA.setZero();
    backSubContr.matrixB.setZero();
    backSubContr.vecTrans.setZero();
    double sumThetaDot = 0;
    Eigen::Vector3d sumTerm2;
    sumTerm2.setZero();
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        Eigen::Vector3d sumTerm1;
        sumTerm1.setZero();
        sumThetaDot += PanelIt->thetaDot;
        PanelIt2 = PanelIt;
        for(int k = j; k <= (int) this->PanelVec.size();k++){
            sumTerm1 += (2*((int) this->PanelVec.size() - k)+1)*PanelIt2->mass*PanelIt2->d*PanelIt2->sHat3_B;
            std::advance(PanelIt2, 1);
        }

        sumTerm2 = pow(sumThetaDot,2)*(2*((int) this->PanelVec.size() - j)+1)*PanelIt->mass*PanelIt->d*PanelIt->sHat1_B;
        const auto eRow = this->matrixEDHRB.row(j - 1);
        backSubContr.matrixA += sumTerm1 * (eRow * this->matrixFDHRB);
        backSubContr.matrixB += sumTerm1 * (eRow * this->matrixGDHRB);
        backSubContr.vecTrans += -sumTerm2 - sumTerm1 * (eRow * this->vectorVDHRB);
        j += 1;
    }
    backSubContr.vecTrans += attBodyForce_B;

    // - Rotational contributions
    backSubContr.matrixC.setZero();
    backSubContr.matrixD.setZero();
    backSubContr.vecRot.setZero();
    sumThetaDot = 0;
    sumTerm2.setZero();
    Eigen::Vector3d sumTerm3;
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        Eigen::Vector3d sumTerm1;
        sumTerm1.setZero();
        sumThetaDot += PanelIt->thetaDot;
        PanelIt2 = PanelIt;
        for(int k = j; k <= (int) this->PanelVec.size();k++){
            sumTerm3.setZero();
            if(k+1<=(int) this->PanelVec.size()){
                PanelIt3 = PanelIt2;
                std::advance(PanelIt3, 1);
                for(int n = k+1; n <= (int) this->PanelVec.size();n++){
                    sumTerm3 += 2*PanelIt3->r_SB_B.cross(PanelIt2->sHat3_B);
                    std::advance(PanelIt3, 1);
                }
            }
            sumTerm1 += PanelIt2->IPntS_S(1,1)*PanelIt2->sHat2_B
            + PanelIt2->mass*PanelIt2->d*(PanelIt2->r_SB_B.cross(PanelIt2->sHat3_B) + sumTerm3);
            std::advance(PanelIt2, 1);
        }
        sumTerm3.setZero();
        if(j+1<=(int) this->PanelVec.size()){
            PanelIt3 = PanelIt;
            std::advance(PanelIt3, 1);
            for(int n = j+1; n <= (int) this->PanelVec.size();n++){
                sumTerm3 += 2*PanelIt3->r_SB_B.cross(PanelIt->sHat1_B);
                std::advance(PanelIt3, 1);
            }
        }
        sumTerm2 = PanelIt->mass*this->omegaLoc_BN_B.cross(PanelIt->r_SB_B.cross(PanelIt->rPrime_SB_B))
        + pow(sumThetaDot,2)*PanelIt->mass*PanelIt->d*(PanelIt->r_SB_B.cross(PanelIt->sHat1_B) + sumTerm3)
        + PanelIt->IPntS_S(1,1)*sumThetaDot*this->omegaLoc_BN_B.cross(PanelIt->sHat2_B);
        const auto eRow = this->matrixEDHRB.row(j - 1);
        backSubContr.matrixC += sumTerm1 * (eRow * this->matrixFDHRB);
        backSubContr.matrixD += sumTerm1 * (eRow * this->matrixGDHRB);
        backSubContr.vecRot += -sumTerm2 - sumTerm1 * (eRow * this->vectorVDHRB);
        j += 1;
    }
    backSubContr.vecRot += attBodyTorquePntB_B;


    return;
}

/*! This method is used to find the derivatives for the HRB stateEffector: thetaDDot and the kinematic derivative
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] rDDot_BN_N [m/s^2] Hub translational acceleration expressed in inertial-frame components.
 * @param[in] omegaDot_BN_B [rad/s^2] Hub angular acceleration expressed in body-frame components.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 */
void NHingedRigidBodyStateEffector::computeDerivatives(double integTime [[maybe_unused]], Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN)
{
    // - Grab necessary values from manager (these have been previously computed in hubEffector)
    Eigen::Vector3d rDDotLoc_BN_N;
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Vector3d omegaDotLoc_BN_B;
    rDDotLoc_BN_N = rDDot_BN_N;
    sigmaLocal_BN = sigma_BN;
    omegaDotLoc_BN_B = omegaDot_BN_B;

    // - Find rDDotLoc_BN_B
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d rDDotLoc_BN_B;
    dcm_BN = (sigmaLocal_BN.toRotationMatrix()).transpose();
    rDDotLoc_BN_B = dcm_BN*rDDotLoc_BN_N;

    // - Compute Derivatives
    std::vector<HingedPanel>::iterator PanelIt;
    Eigen::MatrixXd thetaDDot(this->PanelVec.size(),1);
    int i = 0;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        thetaDDot(i,0) = this->matrixEDHRB.row(i).dot(this->matrixFDHRB*rDDotLoc_BN_B)
        + this->matrixEDHRB.row(i)*this->matrixGDHRB*omegaDotLoc_BN_B + this->matrixEDHRB.row(i)*this->vectorVDHRB;
        i += 1;
    }
    // - First is trivial
    this->thetaState->setDerivative(this->thetaDotState->getStateReference());
    // - Second, a little more involved
    this->thetaDotState->setDerivative(thetaDDot);

    return;
}

/*! This method is for calculating the contributions of the HRB state effector to the energy and momentum of the s/c
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] rotAngMomPntCContr_B [kg*m^2/s] Rotational angular momentum contribution.
 * @param[in,out] rotEnergyContr [J] Rotational energy contribution.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void NHingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime [[maybe_unused]], Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    this->omegaLoc_BN_B = omega_BN_B;
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = this->omegaLoc_BN_B;

    Eigen::Vector3d omega_SN_B;
    Eigen::Matrix3d IPntS_B;
    Eigen::Vector3d rDot_SB_B;
    std::vector<HingedPanel>::iterator PanelIt;
    Eigen::Vector3d rotAngMomPntCContr_B_Sum;
    rotAngMomPntCContr_B_Sum.setZero();
    double rotEnergyContr_Sum = 0;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        omega_SN_B = PanelIt->omega_SB_B + omegaLocal_BN_B;
        IPntS_B = PanelIt->dcm_SB.transpose()*PanelIt->IPntS_S*PanelIt->dcm_SB;
        rDot_SB_B = PanelIt->rPrime_SB_B + omegaLocal_BN_B.cross(PanelIt->r_SB_B);
        rotAngMomPntCContr_B_Sum += IPntS_B*omega_SN_B + PanelIt->mass*PanelIt->r_SB_B.cross(rDot_SB_B);
        rotEnergyContr_Sum += 0.5*omega_SN_B.dot(IPntS_B*omega_SN_B) + 1.0/2.0*PanelIt->mass*rDot_SB_B.dot(rDot_SB_B)
        + 1.0/2.0*PanelIt->k*(PanelIt->theta-PanelIt->theta_0)*(PanelIt->theta-PanelIt->theta_0);
    }

    // - Find rotational angular momentum contribution from hub
    rotAngMomPntCContr_B = rotAngMomPntCContr_B_Sum;

    // - Find rotational energy contribution from the hub
    rotEnergyContr = rotEnergyContr_Sum;

    return;
}
/*! This method is used so that the simulation will ask HRB to update messages.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void NHingedRigidBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->writeOutputStateMessages(CurrentSimNanos);

    return;
}
