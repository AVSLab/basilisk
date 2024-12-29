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
{
    return;
}

/*! This method allows the HRB state effector to have access to the hub states and gravity*/
void NHingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Get access to the hub states
    this->g_N = statesIn.getPropertyReference(this->propName_vehicleGravity);

    return;
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

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub
 method */
void NHingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    sigmaLocal_BN = (Eigen::Vector3d )sigma_BN;
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
    // - Define omegaTildeLoc_BN_B
    this->omegaTildeLoc_BN_B = eigenTilde(this->omegaLoc_BN_B);

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
        this->vectorVDHRB(j-1) = sumTerm1 + PanelIt->sHat2_B.dot(gravTorqueCurPanel)
        + 2.0*PanelIt->d*PanelIt->sHat3_B.dot(gravForceRestOfPanels);
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
    backSubContr.matrixC.setZero();
    backSubContr.matrixD.setZero();
    backSubContr.vecRot.setZero();
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
    WriteOutputMessages(CurrentSimNanos);

    return;
}
