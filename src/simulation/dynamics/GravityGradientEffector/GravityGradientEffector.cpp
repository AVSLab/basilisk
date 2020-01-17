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

#include <iostream>
#include "GravityGradientEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"

GravityGradientEffector::GravityGradientEffector()
{
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
    this->forceExternal_N.fill(0.0);
    
	this->gravityGradientOutMsgId = -1;
    this->OutputBufferCount = 2;

	return;
}

/*! The destructor.*/
GravityGradientEffector::~GravityGradientEffector()
{
	return;
}

/*! Create the outgoing gravity gradient torque message.
 @return void
 */
void GravityGradientEffector::SelfInit()
{
    std::string outMsgName;         /* output msg name */

    if (this->gravityGradientOutMsgName.length() > 0) {
        /* use the user specified msg name */
        outMsgName = this->gravityGradientOutMsgName;
    } else {
        /* auto-generate a default output msg name */
        outMsgName = this->ModelTag + "_gravityGradient";
    }
    this->gravityGradientOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(outMsgName,
                                                                sizeof(GravityGradientSimMsg),
                                                                this->OutputBufferCount,
                                                                "GravityGradientSimMsg",
                                                                moduleID);
    
    return;
}

/*! This method is used to connect to incoming message.  For this module there are none.
 @return void
 */
void GravityGradientEffector::CrossInit()
{
    return;
}

/*! Write the gravity gradient torque output message.
@return void
 */
void GravityGradientEffector::WriteOutputMessages(uint64_t CurrentClock)
{
    GravityGradientSimMsg outMsg;
    eigenVector3d2CArray(this->torqueExternalPntB_B, outMsg.gravityGradientTorque_B);
    SystemMessaging::GetInstance()->WriteMessage(this->gravityGradientOutMsgId, CurrentClock,
                                                 sizeof(GravityGradientSimMsg), reinterpret_cast<uint8_t*>(&outMsg), this->moduleID);

	return;
}

/*! This method is used to link the gravity gradient effector to the hub position, inertia tensor and center of mass vector.
 @return void
 */

void GravityGradientEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
    this->r_BN_N = states.getStateObject("hubPosition");
	this->ISCPntB_B = states.getPropertyReference("inertiaSC");
    this->c_B = states.getPropertyReference("centerOfMassSC");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
//void GravityGradientEffector::updateDragDir(){
//    Eigen::MRPd sigmaBN;
//    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
//    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();
//
//	this->v_B = dcm_BN*this->hubVelocity->getState(); // [m/s] sc velocity
//	this->v_hat_B = this->v_B / this->v_B.norm();
//
//	return;
//}


/*! This method computes the body forces and torques for the gravity gradient effector.
*/
void GravityGradientEffector::computeForceTorque(double integTime){
	// Zero out the force/torque values to begin with
    this->forceExternal_B.setZero();
    this->forceExternal_N.setZero();
    this->torqueExternalPntB_B.setZero();

    double mu = MU_EARTH * pow(10,9);  /* in m^3/s^2 */

    /* find orbit radius */
    double rMag = this->r_BN_N->getState().norm();
//    std::cout << "r_BN_N \n" << this->r_BN_N->getState() << "\n" << std::endl;

    /* compute DCN [BN] */
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();
//    std::cout << "dcm_BN\n" << dcm_BN << "\n" << std::endl;

    /* compute normalized position vector in B frame */
    Eigen::Vector3d rHat_B = dcm_BN * this->r_BN_N->getState().normalized();
//    std::cout << "rHat_B\n" << rHat_B << "\n" << std::endl;

    /* evaluate inertia tensor about center of mass */
    Eigen::MatrixXd ISCPntC_B;
    ISCPntC_B = *this->ISCPntB_B;
//    std::cout << "ISCPntC_B\n" << ISCPntC_B << "\n" << std::endl;

    /* compute gravity gradient torque */
    this->torqueExternalPntB_B = 3.0*mu/rMag/rMag/rMag * (ISCPntC_B * rHat_B);
    this->torqueExternalPntB_B = rHat_B.cross(this->torqueExternalPntB_B);
//    std::cout << "torqueExternalPntB_B\n" << this->torqueExternalPntB_B << "\n" << std::endl;

    /* */

  	return;
}

/*! This method is called once per BSK update cycle.  It writes out a msg of the
    evaluated gravity gradient torque.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void GravityGradientEffector::UpdateState(uint64_t CurrentSimNanos)
{
	this->WriteOutputMessages(CurrentSimNanos);
	return;
}
