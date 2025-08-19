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
}


ReactionWheelStateEffector::~ReactionWheelStateEffector()
{
    // Clear output messages vector
    for (unsigned int i = 0; i < this->rwOutMsgs.size(); i++) {
        if (this->rwOutMsgs[i]) {
            delete this->rwOutMsgs[i];
            this->rwOutMsgs[i] = nullptr;
        }
    }
    rwOutMsgs.clear();

    // Clear reaction wheel data vector - these are owned by SWIG
    ReactionWheelData.clear();
}

void ReactionWheelStateEffector::linkInStates(DynParamManager& statesIn)
{
	//! - Get access to the hub states
    this->g_N = statesIn.getPropertyReference(this->propName_vehicleGravity);
}

void ReactionWheelStateEffector::registerStates(DynParamManager& states)
{
    //! - Find number of RWs and number of RWs with jitter
    this->numRWJitter = 0;
    this->numRW = 0;
    //! zero the RW Omega and theta values (is there I should do this?)
    Eigen::MatrixXd omegasForInit(this->ReactionWheelData.size(),1);

    for (std::size_t i = 0; i < ReactionWheelData.size(); ++i)
    {
        const auto& rw = *ReactionWheelData[i];
        if (rw.RWModel == JitterSimple || rw.RWModel == JitterFullyCoupled) {
            this->numRWJitter++;
        }
        omegasForInit(i, 0) = rw.Omega;
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
    for (std::size_t i = 0; i < ReactionWheelData.size(); ++i)
    {
        auto& rw = *ReactionWheelData[i];
		rw.Omega = this->OmegasState->getState()(i, 0);

		if (rw.RWModel == JitterFullyCoupled) {
			rw.theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(rw.theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = rw.gsHat_B;
			dcm_BW0.col(1) = rw.w2Hat0_B;
			dcm_BW0.col(2) = rw.w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			rw.w2Hat_B = dcm_BW.col(1);
			rw.w3Hat_B = dcm_BW.col(2);

			//! wheel inertia tensor about wheel center of mass represented in B frame
			Eigen::Matrix3d IRWPntWc_W;
			IRWPntWc_W << rw.Js, 0., rw.J13, \
								0., rw.Jt, 0., \
								rw.J13, 0., rw.Jg;
			rw.IRWPntWc_B = dcm_BW * IRWPntWc_W * dcm_BW.transpose();

			//! wheel inertia tensor body frame derivative about wheel center of mass represented in B frame
			Eigen::Matrix3d IPrimeRWPntWc_W;
			IPrimeRWPntWc_W << 0., -rw.J13, 0., \
								-rw.J13, 0., 0., \
								0., 0., 0.;
			IPrimeRWPntWc_W *= rw.Omega;
			rw.IPrimeRWPntWc_B = dcm_BW * IPrimeRWPntWc_W * dcm_BW.transpose();

			//! wheel center of mass location
			rw.rWcB_B = rw.rWB_B + rw.d*rw.w2Hat_B;
			rw.rTildeWcB_B = eigenTilde(rw.rWcB_B);
			rw.rPrimeWcB_B = rw.d*rw.Omega*rw.w3Hat_B;
			Eigen::Matrix3d rPrimeTildeWcB_B = eigenTilde(rw.rPrimeWcB_B);

			//! - Give the mass of the reaction wheel to the effProps mass
			this->effProps.mEff += rw.mass;
			this->effProps.rEff_CB_B += rw.mass*rw.rWcB_B;
			this->effProps.IEffPntB_B += rw.IRWPntWc_B + rw.mass*rw.rTildeWcB_B*rw.rTildeWcB_B.transpose();
			this->effProps.rEffPrime_CB_B += rw.mass*rw.rPrimeWcB_B;
			this->effProps.IEffPrimePntB_B += rw.IPrimeRWPntWc_B + rw.mass*rPrimeTildeWcB_B*rw.rTildeWcB_B.transpose() + rw.mass*rw.rTildeWcB_B*rPrimeTildeWcB_B.transpose();
            thetaCount++;
		} else if (rw.RWModel == JitterSimple) {
			rw.theta = this->thetasState->getState()(thetaCount, 0);
			Eigen::Matrix3d dcm_WW0 = eigenM1(rw.theta);
			Eigen::Matrix3d dcm_BW0;
			dcm_BW0.col(0) = rw.gsHat_B;
			dcm_BW0.col(1) = rw.w2Hat0_B;
			dcm_BW0.col(2) = rw.w3Hat0_B;
			Eigen::Matrix3d dcm_BW = dcm_BW0 * dcm_WW0.transpose();
			rw.w2Hat_B = dcm_BW.col(1);
			rw.w3Hat_B = dcm_BW.col(2);
			thetaCount++;
		}
	}

    // - Need to divide out the total mass of the reaction wheels from rCB_B and rPrimeCB_B
    if (this->effProps.mEff > 0) {
        this->effProps.rEff_CB_B /= this->effProps.mEff;
        this->effProps.rEffPrime_CB_B /= this->effProps.mEff;
    }
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
    sigmaBNLocal = (Eigen::Vector3d ) sigma_BN;
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;

	omegaLoc_BN_B = omega_BN_B;

    for (const auto & rwPtr : ReactionWheelData)
    {
        auto & rw = *rwPtr;
		OmegaSquared = rw.Omega * rw.Omega;

        // Determine which friction model to use (if starting from zero include stribeck)
        if (fabs(rw.Omega) < 0.10*rw.omegaLimitCycle && rw.betaStatic > 0) {
            rw.frictionStribeck = 1;
        }
        double signOfOmega = ((rw.Omega > 0) - (rw.Omega < 0));
        double omegaDot = rw.Omega - rw.omegaBefore;
        double signOfOmegaDot = ((omegaDot > 0) - (omegaDot < 0));
        if (rw.frictionStribeck == 1 && abs(signOfOmega - signOfOmegaDot) < 2 && rw.betaStatic > 0) {
            rw.frictionStribeck = 1;
        } else {
            rw.frictionStribeck = 0;
        }

        double frictionForce;
        double frictionForceAtLimitCycle;
        // Friction model which uses static, stribeck, coulomb, and viscous friction models
        if (rw.frictionStribeck == 1) {
            frictionForce = sqrt(2.0*exp(1.0))*(rw.fStatic - rw.fCoulomb)*exp(-(rw.Omega/rw.betaStatic)*(rw.Omega/rw.betaStatic)/2.0)*rw.Omega/(rw.betaStatic*sqrt(2.0)) + rw.fCoulomb*tanh(rw.Omega*10.0/rw.betaStatic) + rw.cViscous*rw.Omega;
            frictionForceAtLimitCycle = sqrt(2.0*exp(1.0))*(rw.fStatic - rw.fCoulomb)*exp(-(rw.omegaLimitCycle/rw.betaStatic)*(rw.omegaLimitCycle/rw.betaStatic)/2.0)*rw.omegaLimitCycle/(rw.betaStatic*sqrt(2.0)) + rw.fCoulomb*tanh(rw.omegaLimitCycle*10.0/rw.betaStatic) + rw.cViscous*rw.omegaLimitCycle;
        } else {
            frictionForce = signOfOmega*rw.fCoulomb + rw.cViscous*rw.Omega;
            frictionForceAtLimitCycle = rw.fCoulomb + rw.cViscous*rw.omegaLimitCycle;
        }

        // This line avoids the limit cycle that can occur with friction
        if (fabs(rw.Omega) < rw.omegaLimitCycle) {
            frictionForce = frictionForceAtLimitCycle/rw.omegaLimitCycle*rw.Omega;
        }

        // Set friction force
        rw.frictionTorque = -frictionForce;

		if (rw.RWModel == BalancedWheels || rw.RWModel == JitterSimple) {
			backSubContr.matrixD -= rw.Js * rw.gsHat_B * rw.gsHat_B.transpose();
			backSubContr.vecRot -= rw.gsHat_B * (rw.u_current + rw.frictionTorque) + rw.Js*rw.Omega*omegaLoc_BN_B.cross(rw.gsHat_B);

			//! imbalance torque (simplified external)
			if (rw.RWModel == JitterSimple) {
				/* Fs = Us * Omega^2 */ // static imbalance force
				tempF = rw.U_s * OmegaSquared * rw.w2Hat_B;
				backSubContr.vecTrans += tempF;

				//! add in dynamic imbalance torque
				/* tau_s = cross(r_B,Fs) */ // static imbalance torque
				/* tau_d = Ud * Omega^2 */ // dynamic imbalance torque
				backSubContr.vecRot += ( rw.rWB_B.cross(tempF) ) + ( rw.U_d*OmegaSquared * rw.w2Hat_B );
			}
        } else if (rw.RWModel == JitterFullyCoupled) {

			omegas = rw.gsHat_B.transpose()*omegaLoc_BN_B;
			omegaw2 = rw.w2Hat_B.transpose()*omegaLoc_BN_B;
			omegaw3 = rw.w3Hat_B.transpose()*omegaLoc_BN_B;

            gravityTorquePntW_B = rw.d*rw.w2Hat_B.cross(rw.mass*g_B);

			dSquared = rw.d * rw.d;

			rw.aOmega = -rw.mass*rw.d/(rw.Js + rw.mass*dSquared) * rw.w3Hat_B;
			rw.bOmega = -1.0/(rw.Js + rw.mass*dSquared)*((rw.Js+rw.mass*dSquared)*rw.gsHat_B + rw.J13*rw.w3Hat_B + rw.mass*rw.d*rw.rWB_B.cross(rw.w3Hat_B));
			rw.cOmega = 1.0/(rw.Js + rw.mass*dSquared)*(omegaw2*omegaw3*(-rw.mass*dSquared)-rw.J13*omegaw2*omegas-rw.mass*rw.d*rw.w3Hat_B.transpose()*omegaLoc_BN_B.cross(omegaLoc_BN_B.cross(rw.rWB_B))+(rw.u_current + rw.frictionTorque) + rw.gsHat_B.dot(gravityTorquePntW_B));

			backSubContr.matrixA += rw.mass * rw.d * rw.w3Hat_B * rw.aOmega.transpose();
			backSubContr.matrixB += rw.mass * rw.d * rw.w3Hat_B * rw.bOmega.transpose();
			backSubContr.matrixC += (rw.IRWPntWc_B*rw.gsHat_B + rw.mass*rw.d*rw.rWcB_B.cross(rw.w3Hat_B))*rw.aOmega.transpose();
			backSubContr.matrixD += (rw.IRWPntWc_B*rw.gsHat_B + rw.mass*rw.d*rw.rWcB_B.cross(rw.w3Hat_B))*rw.bOmega.transpose();
			backSubContr.vecTrans += rw.mass*rw.d*(OmegaSquared*rw.w2Hat_B - rw.cOmega*rw.w3Hat_B);
			backSubContr.vecRot += rw.mass*rw.d*OmegaSquared*rw.rWcB_B.cross(rw.w2Hat_B) - rw.IPrimeRWPntWc_B*rw.Omega*rw.gsHat_B - omegaLoc_BN_B.cross(rw.IRWPntWc_B*rw.Omega*rw.gsHat_B+rw.mass*rw.rWcB_B.cross(rw.rPrimeWcB_B)) - (rw.IRWPntWc_B*rw.gsHat_B+rw.mass*rw.d*rw.rWcB_B.cross(rw.w3Hat_B))*rw.cOmega;
		}
	}
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
    int thetaCount = 0;

	//! Grab necessarry values from manager
	omegaDotBNLoc_B = omegaDot_BN_B;
	rDDotBNLoc_N = rDDot_BN_N;
	sigmaBNLocal = (Eigen::Vector3d ) sigma_BN;
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	rDDotBNLoc_B = dcm_BN*rDDotBNLoc_N;

	//! - Compute Derivatives
    for (std::size_t i = 0; i < ReactionWheelData.size(); ++i)
    {
        auto& rw = *ReactionWheelData[i];
        if(rw.RWModel == JitterFullyCoupled || rw.RWModel == JitterSimple) {
            // - Set trivial kinemetic derivative
            thetasDot(thetaCount,0) = rw.Omega;
            thetaCount++;
        }
		if (rw.RWModel == BalancedWheels || rw.RWModel == JitterSimple) {
			OmegasDot(i,0) = (rw.u_current + rw.frictionTorque)/rw.Js - rw.gsHat_B.transpose()*omegaDotBNLoc_B;

            // Check for numerical instability due to excessive wheel acceleration
            if (std::abs(OmegasDot(i,0)) > this->maxWheelAcceleration) {
                bskLogger.bskLog(BSK_WARNING, "Excessive reaction wheel acceleration detected (%.2e rad/s^2). This may be caused by using unlimited torque (useMaxTorque=False) with a small spacecraft inertia. Consider using torque limits or increasing spacecraft inertia.", std::abs(OmegasDot(i,0)));

                // Safety mechanism: limit the wheel acceleration to prevent numerical instability
                OmegasDot(i,0) = std::copysign(this->maxWheelAcceleration, OmegasDot(i,0));

                // Recalculate the effective torque for consistency
                double effectiveTorque = OmegasDot(i,0) * rw.Js + rw.gsHat_B.transpose()*omegaDotBNLoc_B;
                rw.u_current = effectiveTorque - rw.frictionTorque;
            }
        } else if(rw.RWModel == JitterFullyCoupled) {
			OmegasDot(i,0) = rw.aOmega.dot(rDDotBNLoc_B) + rw.bOmega.dot(omegaDotBNLoc_B) + rw.cOmega;

            // Check for numerical instability in fully coupled model as well
            if (std::abs(OmegasDot(i,0)) > this->maxWheelAcceleration) {
                bskLogger.bskLog(BSK_WARNING, "Excessive reaction wheel acceleration detected (%.2e rad/s^2). This may be caused by using unlimited torque (useMaxTorque=False) with a small spacecraft inertia. Consider using torque limits or increasing spacecraft inertia.", std::abs(OmegasDot(i,0)));

                // Safety mechanism: limit the wheel acceleration to prevent numerical instability
                OmegasDot(i,0) = std::copysign(this->maxWheelAcceleration, OmegasDot(i,0));
            }
		}
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
	Eigen::Vector3d omegaLoc_BN_B = omega_BN_B;

    //! - Compute energy and momentum contribution of each wheel
    rotAngMomPntCContr_B.setZero();
    for (const auto & rwPtr : ReactionWheelData)
    {
        auto const& rw = *rwPtr;
		if (rw.RWModel == BalancedWheels || rw.RWModel == JitterSimple) {
			rotAngMomPntCContr_B += rw.Js*rw.gsHat_B*rw.Omega;
            rotEnergyContr += 1.0/2.0*rw.Js*rw.Omega*rw.Omega + rw.Js*rw.Omega*rw.gsHat_B.dot(omegaLoc_BN_B);
		} else if (rw.RWModel == JitterFullyCoupled) {
			Eigen::Vector3d omega_WN_B = omegaLoc_BN_B + rw.Omega*rw.gsHat_B;
			Eigen::Vector3d r_WcB_B = rw.rWcB_B;
			Eigen::Vector3d rDot_WcB_B = rw.d*rw.Omega*rw.w3Hat_B + omegaLoc_BN_B.cross(rw.rWcB_B);

			rotEnergyContr += 0.5*omega_WN_B.transpose()*rw.IRWPntWc_B*omega_WN_B + 0.5*rw.mass*rDot_WcB_B.dot(rDot_WcB_B);
			rotAngMomPntCContr_B += rw.IRWPntWc_B*omega_WN_B + rw.mass*r_WcB_B.cross(rDot_WcB_B);
		}
    }
}

/*! add a RW data object to the reactionWheelStateEffector
 */
void ReactionWheelStateEffector::addReactionWheel(std::shared_ptr<RWConfigPayload> NewRW)
{
    /* store the RW information */
    this->ReactionWheelData.push_back(NewRW);

    /* add a RW state log output message for this wheel */
    Message<RWConfigLogMsgPayload> *msg;
    msg = new Message<RWConfigLogMsgPayload>;
    this->rwOutMsgs.push_back(msg);
}


/*! Reset the module to origina configuration values.

 */
void ReactionWheelStateEffector::Reset(uint64_t CurrenSimNanos)
{
    RWCmdMsgPayload RWCmdInitializer;
    RWCmdInitializer.u_cmd = 0.0;

    //! - Clear out any currently firing RWs and re-init cmd array
    this->NewRWCmds.clear();

    for (const auto & rwPtr : ReactionWheelData)
    {
        this->NewRWCmds.push_back(RWCmdInitializer);

        auto & rw = *rwPtr;
        if (rw.betaStatic == 0.0)
        {
            bskLogger.bskLog(BSK_WARNING, "Stribeck coefficent currently zero and should be positive to active this friction model, or negative to turn it off!");
        }
        //! Define CoM offset d and off-diagonal inertia J13 if using fully coupled model
        if (rw.RWModel == JitterFullyCoupled) {
            rw.d = rw.U_s/rw.mass; //!< determine CoM offset from static imbalance parameter
            rw.J13 = rw.U_d; //!< off-diagonal inertia is equal to dynamic imbalance parameter
        }
    }

    /* zero the RW wheel output message buffer */
    this->rwSpeedMsgBuffer = this->rwSpeedOutMsg.zeroMsgPayload;
}

/*! This method is here to write the output message structure into the specified
 message.
 @param CurrentClock The current time used for time-stamping the message

 */
void ReactionWheelStateEffector::WriteOutputMessages(uint64_t CurrentClock)
{
    for (std::size_t i = 0; i < ReactionWheelData.size(); ++i)
    {
        auto& rw = *ReactionWheelData[i];
        if (numRWJitter > 0) {
            rw.theta = this->thetasState->getState()(i, 0);
        }
        rw.Omega = this->OmegasState->getState()(i, 0);

        RWConfigLogMsgPayload tmpRW = this->rwOutMsgs[i]->zeroMsgPayload;
		tmpRW.theta = rw.theta;
		tmpRW.u_current = rw.u_current;
        tmpRW.frictionTorque = rw.frictionTorque;
		tmpRW.u_max = rw.u_max;
		tmpRW.u_min = rw.u_min;
		tmpRW.u_f = rw.fCoulomb;
		tmpRW.Omega = rw.Omega;
		tmpRW.Omega_max = rw.Omega_max;
		tmpRW.Js = rw.Js;
		tmpRW.U_s = rw.U_s;
		tmpRW.U_d = rw.U_d;
		tmpRW.RWModel = rw.RWModel;
        tmpRW.P_max = rw.P_max;
        eigenVector3d2CArray(rw.gsHat_B, tmpRW.gsHat_B);
        eigenVector3d2CArray(rw.rWB_B, tmpRW.rWB_B);
		// Write out config data for each reaction wheel
        this->rwOutMsgs[i]->write(&tmpRW, this->moduleID, CurrentClock);
	}
}

/*! This method is here to write the output message structure into the specified
 message.
 @param integTimeNanos The current time used for time-stamping the message

 */
void ReactionWheelStateEffector::writeOutputStateMessages(uint64_t integTimeNanos)
{
    for (std::size_t i = 0; i < ReactionWheelData.size(); ++i)
    {
        auto& rw = *ReactionWheelData[i];
        if (numRWJitter > 0) {
            rw.theta = this->thetasState->getState()(i, 0);
            this->rwSpeedMsgBuffer.wheelThetas[i] = rw.theta;
        }
        rw.Omega = this->OmegasState->getState()(i, 0);
        this->rwSpeedMsgBuffer.wheelSpeeds[i] = rw.Omega;
    }

    // Write this message once for all reaction wheels
    this->rwSpeedOutMsg.write(&this->rwSpeedMsgBuffer, this->moduleID, integTimeNanos);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the RWs.

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

 @param CurrentTime The current simulation time converted to a double
 */
void ReactionWheelStateEffector::ConfigureRWRequests(double CurrentTime)
{
    for (std::size_t i = 0; i < NewRWCmds.size(); ++i)
	{
        auto& cmd = NewRWCmds[i];
		// Torque saturation
		if (this->ReactionWheelData[i]->u_max > 0) {
			if(cmd.u_cmd > this->ReactionWheelData[i]->u_max) {
				cmd.u_cmd = this->ReactionWheelData[i]->u_max;
			} else if(cmd.u_cmd < -this->ReactionWheelData[i]->u_max) {
				cmd.u_cmd = -this->ReactionWheelData[i]->u_max;
			}
		} else {
            // Warning for unlimited torque with potentially small spacecraft
            static bool warningIssued = false;
            if (!warningIssued && std::abs(cmd.u_cmd) > this->largeTorqueThreshold) {  // Threshold for "large" torque
                bskLogger.bskLog(BSK_WARNING, "Using unlimited reaction wheel torque (u_max <= 0). This can cause numerical instability with small spacecraft inertia. Consider setting useMaxTorque=True or increasing spacecraft inertia.");
                warningIssued = true;
            }
        }

		// minimum torque
		if (std::abs(cmd.u_cmd) < this->ReactionWheelData[i]->u_min) {
			cmd.u_cmd = 0.0;
		}

        // Power saturation
        if (this->ReactionWheelData[i]->P_max > 0) {
            if (std::abs(cmd.u_cmd * this->ReactionWheelData[i]->Omega) >= this->ReactionWheelData[i]->P_max) {
                cmd.u_cmd = std::copysign(this->ReactionWheelData[i]->P_max / this->ReactionWheelData[i]->Omega, cmd.u_cmd);
            }
        }

        // Speed saturation
        if (std::abs(this->ReactionWheelData[i]->Omega) >= this->ReactionWheelData[i]->Omega_max
            && this->ReactionWheelData[i]->Omega_max > 0.0 /* negative Omega_max turns of wheel saturation modeling */
            && this->ReactionWheelData[i]->Omega * cmd.u_cmd >= 0.0 // check if torque would accelerate wheel speed beyond Omega_max
            ) {
            cmd.u_cmd = 0.0;
        }

		this->ReactionWheelData[i]->u_current = cmd.u_cmd; // save actual torque for reaction wheel motor

        // Save the previous omega for next time
        this->ReactionWheelData[i]->omegaBefore = this->ReactionWheelData[i]->Omega;
	}
}

/*! This method is the main cyclical call for the scheduled part of the RW
 dynamics model.  It reads the current commands array and sets the RW
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system

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
