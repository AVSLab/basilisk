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


#include "reactionWheelStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cstring>
#include <iostream>
#include <cmath>

ReactionWheelStateEffector::ReactionWheelStateEffector()
{
	CallCounts = 0;
	prevCommandTime = 0xFFFFFFFFFFFFFFFF;

    effProps.mEff = 0.0;
    effProps.IEffPntB_B.setZero();
    effProps.rEff_CB_B.setZero();
	effProps.IEffPrimePntB_B.setZero();
	effProps.rEffPrime_CB_B.setZero();

    this->nameOfReactionWheelOmegasState = "reactionWheelOmegas";
    this->nameOfReactionWheelThetasState = "reactionWheelThetas";

    return;
}


ReactionWheelStateEffector::~ReactionWheelStateEffector()
{
    for (long unsigned int c=0; c<this->rwOutMsgs.size(); c++) {
        free(this->rwOutMsgs.at(c));
    }
    return;
}

void ReactionWheelStateEffector::linkInStates(DynParamManager& statesIn)
{
	//! - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
	this->hubSigma = statesIn.getStateObject(this->stateNameOfSigma);
	this->hubOmega = statesIn.getStateObject(this->stateNameOfOmega);
	this->hubVelocity = statesIn.getStateObject(this->stateNameOfVelocity);
    this->g_N = statesIn.getPropertyReference(this->propName_vehicleGravity);

	return;
}

void ReactionWheelStateEffector::registerStates(DynParamManager& states)
{
    //! - Find number of RWs and number of RWs with jitter
    this->numRWJitter = 0;
    this->numRW = 0;
    std::vector<RWConfigMsgPayload *>::iterator RWItp;
    RWConfigMsgPayload * RWIt;
    //! zero the RW Omega and theta values (is there I should do this?)
    Eigen::MatrixXd omegasForInit(this->ReactionWheelData.size(),1);

    for(RWItp=ReactionWheelData.begin(); RWItp!=ReactionWheelData.end(); RWItp++) {
        RWIt = *RWItp;
        if (RWIt->RWModel == JitterSimple || RWIt->RWModel == JitterFullyCoupled) {
            this->numRWJitter++;
        }
        omegasForInit(RWItp - this->ReactionWheelData.begin(), 0) = RWIt->Omega;
        this->numRW++;
    }

	this->OmegasState = states.registerState((uint32_t) this->numRW, 1, this->nameOfReactionWheelOmegasState);

	if (numRWJitter > 0) {
		this->thetasState = states.registerState((uint32_t) this->numRWJitter, 1, this->nameOfReactionWheelThetasState);
	}

    this->OmegasState->setState(omegasForInit);
    if (this->numRWJitter > 0) {
        Eigen::MatrixXd thetasForZeroing(this->numRWJitter,1);
        thetasForZeroing.setZero();
        this->thetasState->setState(thetasForZeroing);
    }

    return;
}

void ReactionWheelStateEffector::updateEffectorMassProps(double integTime)
{
    // - Zero the mass props information because these will be accumulated during this call
    this->effProps.mEff = 0.;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    int thetaCount = 0;
    std::vector<RWConfigMsgPayload *>::iterator RWItp;
    RWConfigMsgPayload *RWIt;
	for(RWItp=ReactionWheelData.begin(); RWItp!=ReactionWheelData.end(); RWItp++)
	{
        RWIt = *RWItp;
		RWIt->Omega = this->OmegasState->getState()(RWItp - ReactionWheelData.begin(), 0);
		if (RWIt->RWModel == JitterFullyCoupled) {
			RWIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(RWIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = RWIt->gsHat_B;
			dcm_BW0.col(1) = RWIt->w2Hat0_B;
			dcm_BW0.col(2) = RWIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			RWIt->w2Hat_B = dcm_BW.col(1);
			RWIt->w3Hat_B = dcm_BW.col(2);

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << RWIt->Js, 0., RWIt->J13, \
								0., RWIt->Jt, 0., \
								RWIt->J13, 0., RWIt->Jg;
			RWIt->IRWPntWc_B = dcm_BW * IRWPntWc_W * dcm_BW.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W << 0., -RWIt->J13, 0., \
								-RWIt->J13, 0., 0., \
								0., 0., 0.;
			IPrimeRWPntWc_W *= RWIt->Omega;
			RWIt->IPrimeRWPntWc_B = dcm_BW * IPrimeRWPntWc_W * dcm_BW.transpose();

			//! wheel center of mass location
			RWIt->rWcB_B = RWIt->rWB_B + RWIt->d*RWIt->w2Hat_B;
			RWIt->rTildeWcB_B = eigenTilde(RWIt->rWcB_B);
			RWIt->rPrimeWcB_B = RWIt->d*RWIt->Omega*RWIt->w3Hat_B;
			Eigen::Matrix3d rPrimeTildeWcB_B = eigenTilde(RWIt->rPrimeWcB_B);

			//! - Give the mass of the reaction wheel to the effProps mass
			this->effProps.mEff += RWIt->mass;
			this->effProps.rEff_CB_B += RWIt->mass*RWIt->rWcB_B;
			this->effProps.IEffPntB_B += RWIt->IRWPntWc_B + RWIt->mass*RWIt->rTildeWcB_B*RWIt->rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += RWIt->mass*RWIt->rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += RWIt->IPrimeRWPntWc_B + RWIt->mass*rPrimeTildeWcB_B*RWIt->rTildeWcB_B.transpose() + RWIt->mass*RWIt->rTildeWcB_B*rPrimeTildeWcB_B.transpose();
            thetaCount++;
		} else if (RWIt->RWModel == JitterSimple) {
			RWIt->theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(RWIt->theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = RWIt->gsHat_B;
			dcm_BW0.col(1) = RWIt->w2Hat0_B;
			dcm_BW0.col(2) = RWIt->w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			RWIt->w2Hat_B = dcm_BW.col(1);
			RWIt->w3Hat_B = dcm_BW.col(2);
			thetaCount++;
		}
	}

    // - Need to divide out the total mass of the reaction wheels from rCB_B and rPrimeCB_B
    if (this->effProps.mEff > 0) {
        this->effProps.rEff_CB_B /= this->effProps.mEff;
        this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
    }

	return;
}

void ReactionWheelStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
	Eigen::Vector3d omegaLoc_BN_B;
	Eigen::Vector3d tempF;
	double omegas;
	double omegaw2;
	double omegaw3;
	double dSquared;
	double OmegaSquared;
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                        /*! direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                        /*! direction cosine matrix from B to N */
    Eigen::Vector3d gravityTorquePntW_B;           /*! torque of gravity on HRB about Pnt H */
    Eigen::Vector3d gLocal_N;                      /*! gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                           /*! gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    //! - Find dcm_BN
    sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;

	omegaLoc_BN_B = this->hubOmega->getState();

    std::vector<RWConfigMsgPayload *>::iterator RWItp;
    RWConfigMsgPayload * RWIt;
	for(RWItp=ReactionWheelData.begin(); RWItp!=ReactionWheelData.end(); RWItp++)
	{
        RWIt = *RWItp;
		OmegaSquared = RWIt->Omega * RWIt->Omega;

        // Determine which friction model to use (if starting from zero include stribeck)
        if (fabs(RWIt->Omega) < 0.10*RWIt->omegaLimitCycle && RWIt->betaStatic > 0) {
            RWIt->frictionStribeck = 1;
        }
        double signOfOmega = ((RWIt->Omega > 0) - (RWIt->Omega < 0));
        double omegaDot = RWIt->Omega - RWIt->omegaBefore;
        double signOfOmegaDot = ((omegaDot > 0) - (omegaDot < 0));
        if (RWIt->frictionStribeck == 1 && abs(signOfOmega - signOfOmegaDot) < 2 && RWIt->betaStatic > 0) {
            RWIt->frictionStribeck = 1;
        } else {
            RWIt->frictionStribeck = 0;
        }

        double frictionForce;
        double frictionForceAtLimitCycle;
        // Friction model which uses static, stribeck, coulomb, and viscous friction models
        if (RWIt->frictionStribeck == 1) {
            frictionForce = sqrt(2.0*exp(1.0))*(RWIt->fStatic - RWIt->fCoulomb)*exp(-(RWIt->Omega/RWIt->betaStatic)*(RWIt->Omega/RWIt->betaStatic)/2.0)*RWIt->Omega/(RWIt->betaStatic*sqrt(2.0)) + RWIt->fCoulomb*tanh(RWIt->Omega*10.0/RWIt->betaStatic) + RWIt->cViscous*RWIt->Omega;
            frictionForceAtLimitCycle = sqrt(2.0*exp(1.0))*(RWIt->fStatic - RWIt->fCoulomb)*exp(-(RWIt->omegaLimitCycle/RWIt->betaStatic)*(RWIt->omegaLimitCycle/RWIt->betaStatic)/2.0)*RWIt->omegaLimitCycle/(RWIt->betaStatic*sqrt(2.0)) + RWIt->fCoulomb*tanh(RWIt->omegaLimitCycle*10.0/RWIt->betaStatic) + RWIt->cViscous*RWIt->omegaLimitCycle;
        } else {
            frictionForce = signOfOmega*RWIt->fCoulomb + RWIt->cViscous*RWIt->Omega;
            frictionForceAtLimitCycle = RWIt->fCoulomb + RWIt->cViscous*RWIt->omegaLimitCycle;
        }

        // This line avoids the limit cycle that can occur with friction
        if (fabs(RWIt->Omega) < RWIt->omegaLimitCycle) {
            frictionForce = frictionForceAtLimitCycle/RWIt->omegaLimitCycle*RWIt->Omega;
        }

        // Set friction force
        RWIt->frictionTorque = -frictionForce;

		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			backSubContr.matrixD -= RWIt->Js * RWIt->gsHat_B * RWIt->gsHat_B.transpose();
			backSubContr.vecRot -= RWIt->gsHat_B * (RWIt->u_current + RWIt->frictionTorque) + RWIt->Js*RWIt->Omega*omegaLoc_BN_B.cross(RWIt->gsHat_B);

			//! imbalance torque (simplified external)
			if (RWIt->RWModel == JitterSimple) {
				/* Fs = Us * Omega^2 */ // static imbalance force
				tempF = RWIt->U_s * OmegaSquared * RWIt->w2Hat_B;
				backSubContr.vecTrans += tempF;

				//! add in dynamic imbalance torque
				/* tau_s = cross(r_B,Fs) */ // static imbalance torque
				/* tau_d = Ud * Omega^2 */ // dynamic imbalance torque
				backSubContr.vecRot += ( RWIt->rWB_B.cross(tempF) ) + ( RWIt->U_d*OmegaSquared * RWIt->w2Hat_B );
			}
        } else if (RWIt->RWModel == JitterFullyCoupled) {

			omegas = RWIt->gsHat_B.transpose()*omegaLoc_BN_B;
			omegaw2 = RWIt->w2Hat_B.transpose()*omegaLoc_BN_B;
			omegaw3 = RWIt->w3Hat_B.transpose()*omegaLoc_BN_B;

            gravityTorquePntW_B = RWIt->d*RWIt->w2Hat_B.cross(RWIt->mass*g_B);

			dSquared = RWIt->d * RWIt->d;

			RWIt->aOmega = -RWIt->mass*RWIt->d/(RWIt->Js + RWIt->mass*dSquared) * RWIt->w3Hat_B;
			RWIt->bOmega = -1.0/(RWIt->Js + RWIt->mass*dSquared)*((RWIt->Js+RWIt->mass*dSquared)*RWIt->gsHat_B + RWIt->J13*RWIt->w3Hat_B + RWIt->mass*RWIt->d*RWIt->rWB_B.cross(RWIt->w3Hat_B));
			RWIt->cOmega = 1.0/(RWIt->Js + RWIt->mass*dSquared)*(omegaw2*omegaw3*(-RWIt->mass*dSquared)-RWIt->J13*omegaw2*omegas-RWIt->mass*RWIt->d*RWIt->w3Hat_B.transpose()*omegaLoc_BN_B.cross(omegaLoc_BN_B.cross(RWIt->rWB_B))+(RWIt->u_current + RWIt->frictionTorque) + RWIt->gsHat_B.dot(gravityTorquePntW_B));

			backSubContr.matrixA += RWIt->mass * RWIt->d * RWIt->w3Hat_B * RWIt->aOmega.transpose();
			backSubContr.matrixB += RWIt->mass * RWIt->d * RWIt->w3Hat_B * RWIt->bOmega.transpose();
			backSubContr.matrixC += (RWIt->IRWPntWc_B*RWIt->gsHat_B + RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->aOmega.transpose();
			backSubContr.matrixD += (RWIt->IRWPntWc_B*RWIt->gsHat_B + RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->bOmega.transpose();
			backSubContr.vecTrans += RWIt->mass*RWIt->d*(OmegaSquared*RWIt->w2Hat_B - RWIt->cOmega*RWIt->w3Hat_B);
			backSubContr.vecRot += RWIt->mass*RWIt->d*OmegaSquared*RWIt->rWcB_B.cross(RWIt->w2Hat_B) - RWIt->IPrimeRWPntWc_B*RWIt->Omega*RWIt->gsHat_B - omegaLoc_BN_B.cross(RWIt->IRWPntWc_B*RWIt->Omega*RWIt->gsHat_B+RWIt->mass*RWIt->rWcB_B.cross(RWIt->rPrimeWcB_B)) - (RWIt->IRWPntWc_B*RWIt->gsHat_B+RWIt->mass*RWIt->d*RWIt->rWcB_B.cross(RWIt->w3Hat_B))*RWIt->cOmega;
		}
	}
	return;
}

void ReactionWheelStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
	Eigen::MatrixXd OmegasDot(this->numRW,1);
    Eigen::MatrixXd thetasDot(this->numRWJitter,1);
	Eigen::Vector3d omegaDotBNLoc_B;
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /*! direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /*! direction cosine matrix from B to N */
	Eigen::Vector3d rDDotBNLoc_N;                  /*! second time derivative of rBN in N frame */
	Eigen::Vector3d rDDotBNLoc_B;                  /*! second time derivative of rBN in B frame */
	int RWi = 0;
    int thetaCount = 0;
	std::vector<RWConfigMsgPayload *>::iterator RWItp;
    RWConfigMsgPayload *RWIt;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = this->hubOmega->getStateDeriv();
	rDDotBNLoc_N = this->hubVelocity->getStateDeriv();
	sigmaBNLocal = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
	for(RWItp=ReactionWheelData.begin(); RWItp!=ReactionWheelData.end(); RWItp++)
	{
        RWIt = *RWItp;
        if(RWIt->RWModel == JitterFullyCoupled || RWIt->RWModel == JitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = RWIt->Omega;
            thetaCount++;
        }
		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			OmegasDot(RWi,0) = (RWIt->u_current + RWIt->frictionTorque)/RWIt->Js - RWIt->gsHat_B.transpose()*omegaDotBNLoc_B;
        } else if(RWIt->RWModel == JitterFullyCoupled) {
			OmegasDot(RWi,0) = RWIt->aOmega.dot(rDDotBNLoc_B) + RWIt->bOmega.dot(omegaDotBNLoc_B) + RWIt->cOmega;
		}
		RWi++;
	}

	OmegasState->setDerivative(OmegasDot);
    if (this->numRWJitter > 0) {
        thetasState->setDerivative(thetasDot);
    }
}

void ReactionWheelStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                              double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /*! direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /*! direction cosine matrix from B to N */
	Eigen::Vector3d omegaLoc_BN_B = hubOmega->getState();

    //! - Compute energy and momentum contribution of each wheel
    rotAngMomPntCContr_B.setZero();
    std::vector<RWConfigMsgPayload *>::iterator RWItp;
    RWConfigMsgPayload *RWIt;
    for(RWItp=ReactionWheelData.begin(); RWItp!=ReactionWheelData.end(); RWItp++)
    {
        RWIt = *RWItp;
		if (RWIt->RWModel == BalancedWheels || RWIt->RWModel == JitterSimple) {
			rotAngMomPntCContr_B += RWIt->Js*RWIt->gsHat_B*RWIt->Omega;
            rotEnergyContr += 1.0/2.0*RWIt->Js*RWIt->Omega*RWIt->Omega + RWIt->Js*RWIt->Omega*RWIt->gsHat_B.dot(omegaLoc_BN_B);
		} else if (RWIt->RWModel == JitterFullyCoupled) {
			Eigen::Vector3d omega_WN_B = omegaLoc_BN_B + RWIt->Omega*RWIt->gsHat_B;
			Eigen::Vector3d r_WcB_B = RWIt->rWcB_B;
			Eigen::Vector3d rDot_WcB_B = RWIt->d*RWIt->Omega*RWIt->w3Hat_B + omegaLoc_BN_B.cross(RWIt->rWcB_B);

			rotEnergyContr += 0.5*omega_WN_B.transpose()*RWIt->IRWPntWc_B*omega_WN_B + 0.5*RWIt->mass*rDot_WcB_B.dot(rDot_WcB_B);
			rotAngMomPntCContr_B += RWIt->IRWPntWc_B*omega_WN_B + RWIt->mass*r_WcB_B.cross(rDot_WcB_B);
		}
    }

    return;
}

/*! add a RW data object to the reactionWheelStateEffector @return void
 */
void ReactionWheelStateEffector::addReactionWheel(RWConfigMsgPayload *NewRW)
{
    /* store the RW information */
    this->ReactionWheelData.push_back(NewRW);

    /* add a RW state log output message for this wheel */
    Message<RWConfigLogMsgPayload> *msg;
    msg = new Message<RWConfigLogMsgPayload>;
    this->rwOutMsgs.push_back(msg);
}


/*! Reset the module to origina configuration values.
 @return void
 */
void ReactionWheelStateEffector::Reset(uint64_t CurrenSimNanos)
{
    RWCmdMsgPayload RWCmdInitializer;
    RWCmdInitializer.u_cmd = 0.0;

    //! - Clear out any currently firing RWs and re-init cmd array
    this->NewRWCmds.clear();
    for (long unsigned int i=0; i<this->ReactionWheelData.size(); i++) {
        this->NewRWCmds.push_back(RWCmdInitializer);
    }

    std::vector<RWConfigMsgPayload *>::iterator itp;
    RWConfigMsgPayload *it;
    for (itp = ReactionWheelData.begin(); itp != ReactionWheelData.end(); itp++)
    {
        it = *itp;
        if (it->betaStatic == 0.0)
        {
            bskLogger.bskLog(BSK_WARNING, "Stribeck coefficent currently zero and should be positive to active this friction model, or negative to turn it off!");
        }
        //! Define CoM offset d and off-diagonal inertia J13 if using fully coupled model
        if (it->RWModel == JitterFullyCoupled) {
            it->d = it->U_s/it->mass; //!< determine CoM offset from static imbalance parameter
            it->J13 = it->U_d; //!< off-diagonal inertia is equal to dynamic imbalance parameter
        }
    }

    /* zero the RW wheel output message buffer */
    this->rwSpeedMsgBuffer = this->rwSpeedOutMsg.zeroMsgPayload;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ReactionWheelStateEffector::WriteOutputMessages(uint64_t CurrentClock)
{
    RWConfigMsgPayload test;
	RWConfigLogMsgPayload tmpRW;
	std::vector<RWConfigMsgPayload *>::iterator itp;
    RWConfigMsgPayload *it;
    int c = 0;
    for (itp = ReactionWheelData.begin(); itp != ReactionWheelData.end(); itp++)
	{
        it = *itp;
        if (numRWJitter > 0) {
            it->theta = this->thetasState->getState()(itp - ReactionWheelData.begin(), 0);
        }
        it->Omega = this->OmegasState->getState()(itp - ReactionWheelData.begin(), 0);

        tmpRW = this->rwOutMsgs[c]->zeroMsgPayload;
		tmpRW.theta = it->theta;
		tmpRW.u_current = it->u_current;
        tmpRW.frictionTorque = it->frictionTorque;
		tmpRW.u_max = it->u_max;
		tmpRW.u_min = it->u_min;
		tmpRW.u_f = it->fCoulomb;
		tmpRW.Omega = it->Omega;
		tmpRW.Omega_max = it->Omega_max;
		tmpRW.Js = it->Js;
		tmpRW.U_s = it->U_s;
		tmpRW.U_d = it->U_d;
		tmpRW.RWModel = it->RWModel;
        tmpRW.P_max = it->P_max;
        eigenVector3d2CArray(it->gsHat_B, tmpRW.gsHat_B);
        eigenVector3d2CArray(it->rWB_B, tmpRW.rWB_B);
		// Write out config data for eachreaction wheel
        this->rwOutMsgs[c]->write(&tmpRW, this->moduleID, CurrentClock);
        c++;
	}

    return;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param integTimeNanos The current time used for time-stamping the message
 @return void
 */
void ReactionWheelStateEffector::writeOutputStateMessages(uint64_t integTimeNanos)
{
    std::vector<RWConfigMsgPayload *>::iterator itp;
    RWConfigMsgPayload *it;
    for (itp = ReactionWheelData.begin(); itp != ReactionWheelData.end(); itp++)
    {
        it = *itp;
        if (numRWJitter > 0) {
            it->theta = this->thetasState->getState()(itp - ReactionWheelData.begin(), 0);
            this->rwSpeedMsgBuffer.wheelThetas[itp - ReactionWheelData.begin()] = it->theta;
        }
        it->Omega = this->OmegasState->getState()(itp - ReactionWheelData.begin(), 0);
        this->rwSpeedMsgBuffer.wheelSpeeds[itp - ReactionWheelData.begin()] = it->Omega;
    }

    // Write this message once for all reaction wheels
    this->rwSpeedOutMsg.write(&this->rwSpeedMsgBuffer, this->moduleID, integTimeNanos);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the RWs.
 @return void
 */
void ReactionWheelStateEffector::ReadInputs()
{

	//! read the incoming command array, or zero if not connected
    if (this->rwMotorCmdInMsg.isLinked()) {
        this->incomingCmdBuffer = this->rwMotorCmdInMsg();
        this->prevCommandTime = this->rwMotorCmdInMsg.timeWritten();
    } else {
        this->incomingCmdBuffer = this->rwMotorCmdInMsg.zeroMsgPayload;
    }

	//! - Set the NewRWCmds vector.  Using the data() method for raw speed
	RWCmdMsgPayload *CmdPtr;
    uint64_t i;
	for(i=0, CmdPtr = NewRWCmds.data(); i<ReactionWheelData.size(); CmdPtr++, i++)
	{
		CmdPtr->u_cmd = this->incomingCmdBuffer.motorTorque[i];
	}
}

/*! This method is used to read the new commands vector and set the RW
 firings appropriately.  It assumes that the ReadInputs method has already been
 run successfully.
 @return void
 @param CurrentTime The current simulation time converted to a double
 */
void ReactionWheelStateEffector::ConfigureRWRequests(double CurrentTime)
{
	std::vector<RWCmdMsgPayload>::iterator CmdIt;
	size_t RWIter = 0;

	// loop through commands
	for(CmdIt = NewRWCmds.begin(); CmdIt != NewRWCmds.end(); CmdIt++)
	{
		// Torque saturation
		if (this->ReactionWheelData[RWIter]->u_max > 0) {
			if(CmdIt->u_cmd > this->ReactionWheelData[RWIter]->u_max) {
				CmdIt->u_cmd = this->ReactionWheelData[RWIter]->u_max;
			} else if(CmdIt->u_cmd < -this->ReactionWheelData[RWIter]->u_max) {
				CmdIt->u_cmd = -this->ReactionWheelData[RWIter]->u_max;
			}
		}

		// minimum torque
		if (std::abs(CmdIt->u_cmd) < this->ReactionWheelData[RWIter]->u_min) {
			CmdIt->u_cmd = 0.0;
		}

        // Power saturation
        if (this->ReactionWheelData[RWIter]->P_max > 0) {
            if (std::abs(CmdIt->u_cmd * this->ReactionWheelData[RWIter]->Omega) >= this->ReactionWheelData[RWIter]->P_max) {
                CmdIt->u_cmd = std::copysign(this->ReactionWheelData[RWIter]->P_max / this->ReactionWheelData[RWIter]->Omega, CmdIt->u_cmd);
            }
        }

        // Speed saturation
        if (std::abs(this->ReactionWheelData[RWIter]->Omega) >= this->ReactionWheelData[RWIter]->Omega_max
            && this->ReactionWheelData[RWIter]->Omega_max > 0.0 /* negative Omega_max turns of wheel saturation modeling */
            && this->ReactionWheelData[RWIter]->Omega * CmdIt->u_cmd >= 0.0 // check if torque would accelerate wheel speed beyond Omega_max
            ) {
            CmdIt->u_cmd = 0.0;
        }

		this->ReactionWheelData[RWIter]->u_current = CmdIt->u_cmd; // save actual torque for reaction wheel motor

        // Save the previous omega for next time
        this->ReactionWheelData[RWIter]->omegaBefore = this->ReactionWheelData[RWIter]->Omega;

		RWIter++;

	}
}

/*! This method is the main cyclical call for the scheduled part of the RW
 dynamics model.  It reads the current commands array and sets the RW
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ReactionWheelStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
	//! - Read the inputs and then call ConfigureRWRequests to set up dynamics
	ReadInputs();
    ConfigureRWRequests(CurrentSimNanos*NANO2SEC);
    WriteOutputMessages(CurrentSimNanos);
//
}
