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
#include <iostream>

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
    this->dcm_HB.Identity();
    this->nameOfThetaState ="nHingedRigidBody" + std::to_string(this->effectorID) + "Theta";
    this->nameOfThetaDotState = "nHingedRigidBody" + std::to_string(this->effectorID) + "ThetaDot";
    this->propertyNameIndex = std::to_string(NHingedRigidBodyStateEffector::effectorID); // preserves effectorID for later adding bodies

    this->effectorID++;

    return;
}

uint64_t NHingedRigidBodyStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
NHingedRigidBodyStateEffector::~NHingedRigidBodyStateEffector()
{
    this->effectorID = 1;    /* reset the panel ID*/
    return;
}


/*! This method reads necessary input messages
  */
void NHingedRigidBodyStateEffector::readInputMessages()
{
    return;
}

/*! This method takes the computed theta states and outputs them to the messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void NHingedRigidBodyStateEffector::WriteOutputMessages(uint64_t CurrentClock)
{ // error has to do with this

    std::cout << "IN WriteOutputMessages" << std::endl;

    int hingedBodyIndex = 0;
    for(auto& hingedPanel: this->PanelVec) {
        if (this->hingedBodyOutMsgs[hingedBodyIndex]->isLinked()) {
            HingedRigidBodyMsgPayload hingedBodyBuffer = this->hingedBodyOutMsgs[hingedBodyIndex]->zeroMsgPayload;

            hingedBodyBuffer.theta = hingedPanel.theta;
            hingedBodyBuffer.thetaDot = hingedPanel.thetaDot;
            this->hingedBodyOutMsgs[hingedBodyIndex]->write(&hingedBodyBuffer, this->moduleID, CurrentClock);
        }

        if (this->nHingedBodyConfigLogOutMsgs[hingedBodyIndex]->isLinked()) {
            SCStatesMsgPayload configLogMsg = this->nHingedBodyConfigLogOutMsgs[hingedBodyIndex]->zeroMsgPayload;

            // Logging the S frame is the body frame B of that object
            eigenVector3d2CArray(hingedPanel.r_SN_N, configLogMsg.r_BN_N);
            eigenVector3d2CArray(hingedPanel.v_SN_N, configLogMsg.v_BN_N);
            eigenMatrixXd2CArray(*hingedPanel.sigma_SN, configLogMsg.sigma_BN);
            eigenMatrixXd2CArray(*hingedPanel.omega_SN_S, configLogMsg.omega_BN_B);
            this->nHingedBodyConfigLogOutMsgs[hingedBodyIndex]->write(&configLogMsg, this->moduleID, CurrentClock);
        }
        hingedBodyIndex++;
    }
    return;
}

void NHingedRigidBodyStateEffector::computeHingedBodyInertialStates()
{
    for(auto& hingedPanel: this->PanelVec) {
        hingedPanel.omega_SN_B = hingedPanel.omega_SB_B + this->omega_BN_B;

        Eigen::Matrix3d dcm_SN = hingedPanel.dcm_SB * this->dcm_BN;
        *hingedPanel.sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
        *hingedPanel.omega_SN_S = hingedPanel.dcm_SB * hingedPanel.omega_SN_B;

        // COM kinematics in body-frame components.
        const Eigen::Vector3d r_HB_B = hingedPanel.r_SB_B + hingedPanel.d * hingedPanel.sHat1_B;
        const double thetaDotSum = hingedPanel.omega_SB_B.dot(hingedPanel.sHat2_B);
        const Eigen::Vector3d rPrime_HB_B = hingedPanel.rPrime_SB_B - hingedPanel.d * thetaDotSum * hingedPanel.sHat3_B;

        // Compute the translation properties.
        hingedPanel.r_SN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * hingedPanel.r_SB_B;
        hingedPanel.v_SN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
                           + this->dcm_BN.transpose() * (hingedPanel.rPrime_SB_B + this->omega_BN_B.cross(hingedPanel.r_SB_B));

        *hingedPanel.r_HN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * r_HB_B;
        *hingedPanel.v_HN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
                            + this->dcm_BN.transpose() * (rPrime_HB_B + this->omega_BN_B.cross(r_HB_B));
    }
}

/*! This method allows the HRB state effector to have access to the hub states and gravity*/
void NHingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->g_N = statesIn.getPropertyReference(this->propName_vehicleGravity); // why is propName_vehicleGravity not in class?  i think its in dynparam

    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N"); // why spiining:  this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "v_BN_N");
    // this->inertialAttitudeProperty = statesIn.getPropertyReference(this->nameOfInertialAttitudeProperty); // spinning bodies doesnt have these two and doesnt have them in the header either
    // this->inertialAngVelocityProperty = statesIn.getPropertyReference(this->nameOfInertialAngVelocityProperty);

    return;
}

void NHingedRigidBodyStateEffector::addHingedPanel(HingedPanel NewPanel) {
    PanelVec.push_back(NewPanel);
    this->numberOfDegreesOfFreedom++;

    this->nHingedBodyConfigLogOutMsgs.push_back(new Message<SCStatesMsgPayload>);
    this->hingedBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);
    this->hingedBodyRefInMsgs.push_back(ReadFunctor<HingedRigidBodyMsgPayload>());

    this->aTheta.conservativeResize(this->aTheta.rows()+1, 3);
    this->bTheta.conservativeResize(this->bTheta.rows()+1, 3);
    // this->CTheta.conservativeResize(this->CTheta.rows()+1); // no cTheta in header ???

    PanelVec[this->numberOfDegreesOfFreedom-1].nameOfInertialPositionProperty = "hingedBodyInertialPosition" + this->propertyNameIndex + "_" + std::to_string(this->numberOfDegreesOfFreedom);
    PanelVec[this->numberOfDegreesOfFreedom-1].nameOfInertialVelocityProperty = "hingedBodyInertialVelocity" + this->propertyNameIndex + "_" + std::to_string(this->numberOfDegreesOfFreedom);
    PanelVec[this->numberOfDegreesOfFreedom-1].nameOfInertialAttitudeProperty = "hingedBodyInertialAttitude" + this->propertyNameIndex + "_" + std::to_string(this->numberOfDegreesOfFreedom);
    PanelVec[this->numberOfDegreesOfFreedom-1].nameOfInertialAngVelocityProperty = "hingedBodyInertialAngVelocity" + this->propertyNameIndex + "_" + std::to_string(this->numberOfDegreesOfFreedom);
}

   void NHingedRigidBodyStateEffector::addDynamicEffector(DynamicEffector *newDynamicEffector, int segment)
{
    if (segment <= 0 || segment > this->numberOfDegreesOfFreedom) {
        bskLogger.bskLog(BSK_ERROR, "Specifying attachment to a non-existent hinged body linkage.");
    } else {
        this->PanelVec[segment-1].assignStateParamNames(newDynamicEffector);
        this->PanelVec[segment-1].dynEffectors.push_back(newDynamicEffector);
    }
}

void NHingedRigidBodyStateEffector::registerProperties(DynParamManager& states)
{
    for(auto& hingedPanel: this->PanelVec) {
        Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
        hingedPanel.r_HN_N = states.createProperty(hingedPanel.nameOfInertialPositionProperty, stateInit);
        hingedPanel.v_HN_N = states.createProperty(hingedPanel.nameOfInertialVelocityProperty, stateInit);
        hingedPanel.sigma_SN = states.createProperty(hingedPanel.nameOfInertialAttitudeProperty, stateInit); // this notation is inconsistant with s=sc
        hingedPanel.omega_SN_S = states.createProperty(hingedPanel.nameOfInertialAngVelocityProperty, stateInit);

        for(auto& dynEffector: hingedPanel.dynEffectors) {
            dynEffector->linkInProperties(states);
        }
    }
}

/*! This method allows the HRB state effector to register its states: theta and thetaDot with the dyn param manager */
void NHingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
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
    this->thetaState = states.registerState((uint32_t) this->PanelVec.size(), 1, this->nameOfThetaState);
    this->thetaState->setState(thetaInitMatrix);
    this->thetaDotState = states.registerState((uint32_t) this->PanelVec.size(), 1, this->nameOfThetaDotState);
    this->thetaDotState->setState(thetaDotInitMatrix);

    registerProperties(states);

    return;
}

/*! This method allows the HRB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void NHingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
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

    std::vector<HingedPanel>::iterator PanelIt;
    int it = 0;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){

        // - Find hinged rigid bodies' position with respect to point B
        // - First need to grab current states
        PanelIt->theta = this->thetaState->getState()(it, 0);
        PanelIt->thetaDot = this->thetaDotState->getState()(it, 0);
        // - Next find the sHat unit vectors
        sum_Theta += PanelIt->theta;
        PanelIt->dcm_SS_prev = eigenM2(PanelIt->theta);
        PanelIt->dcm_SB = eigenM2(sum_Theta)*this->dcm_HB;
        PanelIt->sHat1_B = PanelIt->dcm_SB.row(0);
        PanelIt->sHat2_B = PanelIt->dcm_SB.row(1);
        PanelIt->sHat3_B = PanelIt->dcm_SB.row(2);

        PanelIt->r_SB_B = this->r_HB_B - PanelIt->d*PanelIt->sHat1_B + sum_rH;
        sum_rH += -2*PanelIt->d*PanelIt->sHat1_B;

        // - Define rTilde_SB_B
        PanelIt->rTilde_SB_B = eigenTilde(PanelIt->r_SB_B);

        // - Find rPrime_SB_B
        sum_ThetaDot += PanelIt->thetaDot;
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

//!* Method for defining the Heaviside function for the EOMs */
double NHingedRigidBodyStateEffector::HeaviFunc(double cond)
{
    double ans;
    if (cond < 0.0) ans = 0.0;
    else ans = 1.0;
    return ans;
}

void NHingedRigidBodyStateEffector::computeDependentEffectors(
    BackSubMatrices& backSubContr, double integTime)
{
    Eigen::Vector3d force_S = Eigen::Vector3d::Zero();
    Eigen::Vector3d torquePntS_S = Eigen::Vector3d::Zero();

    for (int idx = static_cast<int>(this->PanelVec.size()) - 1; idx >= 0; --idx) {
        HingedPanel& panel = this->PanelVec[idx];

        for (auto& dynEffector : panel.dynEffectors) {
            dynEffector->computeForceTorque(integTime, 0.0);
            force_S += dynEffector->forceExternal_B;
            torquePntS_S += dynEffector->torqueExternalPntB_B;
        }

        panel.extForce_S = force_S;
        panel.extTorquePntS_S = torquePntS_S;

        // Rotate external forces/torques into the parent body's frame P (new S frame for next loop).
        const Eigen::Matrix3d dcm_PS = panel.dcm_SS_prev.transpose();
        Eigen::Vector3d r_SP_prev;
        if (idx > 0) {
            const HingedPanel& parentPanel = this->PanelVec[idx - 1];
            const Eigen::Vector3d r_HCurrent_B = panel.r_SB_B + panel.d * panel.sHat1_B;
            const Eigen::Vector3d r_HParent_B = parentPanel.r_SB_B + parentPanel.d * parentPanel.sHat1_B;
            r_SP_prev = parentPanel.dcm_SB * (r_HCurrent_B - r_HParent_B);
        } else {
            const Eigen::Vector3d r_SP_S = panel.dcm_SB * (panel.r_SB_B + panel.d * panel.sHat1_B);
            r_SP_prev = dcm_PS * r_SP_S;
        }

        Eigen::Vector3d force_prev = dcm_PS * force_S;
        Eigen::Vector3d torque_prev = dcm_PS * torquePntS_S;

        force_S = force_prev;
        torquePntS_S = torque_prev + r_SP_prev.cross(force_prev);
    }

    // Base body rotated into the hub body frame, added as cumulated force/torque here
    backSubContr.vecTrans += force_S;
    backSubContr.vecRot += torquePntS_S;
}

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub
 method */
void NHingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    backSubContr.matrixA.setZero(); // should i really be zeroing this out?
    backSubContr.matrixB.setZero();
    backSubContr.matrixC.setZero();
    backSubContr.matrixD.setZero();
    backSubContr.vecTrans.setZero();
    backSubContr.vecRot.setZero();

    // - Find dcm_BN
    Eigen::Matrix3d dcm_NB;
    this->sigma_BN = sigma_BN;
    dcm_NB = this->sigma_BN.toRotationMatrix();
    this->dcm_BN = dcm_NB.transpose();
    this->omega_BN_B = omega_BN_B;

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = this->dcm_BN*gLocal_N; // not sure about this either

    // - Define omega_BN_S
    this->omegaLoc_BN_B = this->omega_BN_B;
    std::vector<HingedPanel>::iterator PanelIt;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        PanelIt->omega_BN_S = PanelIt->dcm_SB*this->omegaLoc_BN_B;
    }
    // - Define omegaTildeLoc_BN_B
    this->omegaTildeLoc_BN_B = eigenTilde(this->omegaLoc_BN_B);

    this->computeDependentEffectors(backSubContr, integTime);

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
                sumTerm2 += (2*PanelIt->mass*PanelIt->d*PanelIt->sHat3_B.transpose()*PanelIt2->rTilde_SB_B).transpose();
                std::advance(PanelIt2, 1);
            }
        }
        sumTerm1 = -(PanelIt->IPntS_S(1,1)*PanelIt->sHat2_B.transpose()-PanelIt->mass*PanelIt->d
                     *PanelIt->sHat3_B.transpose()*PanelIt->rTilde_SB_B - sumTerm2.transpose());
        this->matrixGDHRB(j-1,0) = sumTerm1[0];
        this->matrixGDHRB(j-1,1) = sumTerm1[1];
        this->matrixGDHRB(j-1,2) = sumTerm1[2];
        j += 1;
    }

    // - Define v vector for the panel equations
    this->vectorVDHRB.resize((int) this->PanelVec.size());
    this->vectorVDHRB.setZero();
    double massOfCurrentPanelAndBefore = 0; // Summation of all of prior panels masses and the current panels mass
    j = 1;
    for(PanelIt=this->PanelVec.begin(); PanelIt!=this->PanelVec.end(); PanelIt++){
        // Add current panel to mass
        massOfCurrentPanelAndBefore += PanelIt->mass;
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
                sumTerm2 += 4*this->omegaTildeLoc_BN_B*PanelIt3->rPrime_SB_B
                +2*this->omegaTildeLoc_BN_B*this->omegaTildeLoc_BN_B*PanelIt3->r_SB_B;
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
        *PanelIt->omega_BN_S(0) - PanelIt->mass*PanelIt->d*PanelIt->sHat3_B.transpose()*(2*this->omegaTildeLoc_BN_B
             *PanelIt->rPrime_SB_B+this->omegaTildeLoc_BN_B*this->omegaTildeLoc_BN_B*PanelIt->r_SB_B+sumTerm2+sumTerm3);
        // Add gravity torque to this sumTerm
        Eigen::Vector3d gravTorqueCurPanel;
        gravTorqueCurPanel = -PanelIt->d*PanelIt->sHat1_B.cross(PanelIt->mass*g_B);
        Eigen::Vector3d gravForceRestOfPanels;
        double remainingMass;
        remainingMass = this->totalMass - massOfCurrentPanelAndBefore;
        gravForceRestOfPanels = remainingMass*g_B;
        this->vectorVDHRB(j-1) = sumTerm1
        + PanelIt->sHat2_B.dot(gravTorqueCurPanel + PanelIt->dcm_SB.transpose() * PanelIt->extTorquePntS_S)
        + 2.0*PanelIt->d*PanelIt->sHat3_B.dot(gravForceRestOfPanels);
        j += 1;
    }

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper

    // - translational contributions
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
        backSubContr.matrixA += sumTerm1*this->matrixEDHRB.row(j-1)*this->matrixFDHRB;
        backSubContr.matrixB += sumTerm1*this->matrixEDHRB.row(j-1)*this->matrixGDHRB;
        backSubContr.vecTrans += -sumTerm2 - sumTerm1*this->matrixEDHRB.row(j-1)*this->vectorVDHRB;
        j += 1;
    }
    Eigen::Vector3d aTheta;
    Eigen::Vector3d bTheta;

    aTheta = this->matrixEDHRB.row(0)*this->matrixFDHRB;
    bTheta = this->matrixEDHRB.row(0)*this->matrixGDHRB;

    // - Rotational contributions
    sumThetaDot = 0;
    sumTerm2.setZero();
    Eigen::Matrix3d sumTerm3;
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
                    sumTerm3 += 2*PanelIt3->rTilde_SB_B;
                    std::advance(PanelIt3, 1);
                }
            }
            sumTerm1 += PanelIt2->IPntS_S(1,1)*PanelIt2->sHat2_B
            +(PanelIt2->rTilde_SB_B+sumTerm3)*PanelIt2->mass*PanelIt2->d*PanelIt2->sHat3_B;
            std::advance(PanelIt2, 1);
        }
        sumTerm3.setZero();
        if(j+1<=(int) this->PanelVec.size()){
            PanelIt3 = PanelIt;
            std::advance(PanelIt3, 1);
            for(int n = j+1; n <= (int) this->PanelVec.size();n++){
                sumTerm3 += 2*PanelIt3->rTilde_SB_B;
                std::advance(PanelIt3, 1);
            }
        }
        sumTerm2 = PanelIt->mass*this->omegaTildeLoc_BN_B*PanelIt->rTilde_SB_B*PanelIt->rPrime_SB_B
        + pow(sumThetaDot,2)*(PanelIt->rTilde_SB_B+sumTerm3)*PanelIt->mass*PanelIt->d*PanelIt->sHat1_B
        + PanelIt->IPntS_S(1,1)*sumThetaDot*this->omegaTildeLoc_BN_B*PanelIt->sHat2_B;
        backSubContr.matrixC += sumTerm1*this->matrixEDHRB.row(j-1)*this->matrixFDHRB;
        backSubContr.matrixD += sumTerm1*this->matrixEDHRB.row(j-1)*this->matrixGDHRB;
        backSubContr.vecRot += -sumTerm2 - sumTerm1*this->matrixEDHRB.row(j-1)*this->vectorVDHRB;
        j += 1;
    }


    return;
}

/*! This method is used to find the derivatives for the HRB stateEffector: thetaDDot and the kinematic derivative */
void NHingedRigidBodyStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // - Grab necessarry values from manager (these have been previously computed in hubEffector)
    Eigen::Vector3d rDDotLoc_BN_N;
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Vector3d omegaDotLoc_BN_B;
    rDDotLoc_BN_N = rDDot_BN_N;
    sigmaLocal_BN = (Eigen::Vector3d )sigma_BN;
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
    this->thetaState->setDerivative(this->thetaDotState->getState());
    // - Second, a little more involved
    this->thetaDotState->setDerivative(thetaDDot);
    return;
}

/*! This method is for calculating the contributions of the HRB state effector to the energy and momentum of the s/c */
void NHingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omega_BN_B;

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
    this->computeHingedBodyInertialStates();
    WriteOutputMessages(CurrentSimNanos);

    return;
}
