/*
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "architecture/utilities/linearAlgebra.h"

GravityGradientEffector::GravityGradientEffector()
{
	return;
}

/*! The destructor.*/
GravityGradientEffector::~GravityGradientEffector()
{
	return;
}


/*! This method is used to set the effector, and check same module variables

*/
void GravityGradientEffector::Reset(uint64_t CurrentSimNanos)
{
    /* zero the effector output forces and torques */
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->forceExternal_N.fill(0.0);

    if (this->planetPropertyNames.size()==0) {
        bskLogger.bskLog(BSK_ERROR, "planetPropertyNames array is empty, you must specify at least one planet using addPlanetName().");
    }

    /* empty the vector of planet state pointers */
    static_cast<void>(this->r_PN_N.empty());
    static_cast<void>(this->muPlanet.empty());

    return;
}

/*! This method adds planet names to a vector.
 @param planetName The planet name

 */
void GravityGradientEffector::addPlanetName(std::string planetName)
{
    this->planetPropertyNames.push_back(planetName);

    return;
}


/*! Write the gravity gradient torque output message.

 */
void GravityGradientEffector::WriteOutputMessages(uint64_t CurrentClock)
{
    GravityGradientMsgPayload outMsg;
    eigenVector3d2CArray(this->torqueExternalPntB_B, outMsg.gravityGradientTorque_B);
    this->gravityGradientOutMsg.write(&outMsg, this->moduleID, CurrentClock);

	return;
}

/*! This method is used to link the gravity gradient effector to the hub position, inertia tensor and center of mass vector.

 */

void GravityGradientEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->r_BN_N = states.getStateObject(this->stateNameOfPosition);
    this->ISCPntB_B = states.getPropertyReference(this->propName_inertiaSC);
    this->c_B = states.getPropertyReference(this->propName_centerOfMassSC);
    this->m_SC = states.getPropertyReference(this->propName_m_SC);

    std::vector<std::string>::iterator name;
    for(name = this->planetPropertyNames.begin(); name != this->planetPropertyNames.end(); name++) {
        this->r_PN_N.push_back(states.getPropertyReference(*name + ".r_PN_N"));
        this->muPlanet.push_back(states.getPropertyReference(*name + ".mu"));
    }
}

/*! This method computes the body forces and torques for the gravity gradient effector.
*/
void GravityGradientEffector::computeForceTorque(double integTime, double timeStep){
	// Zero out the force/torque values to begin with
    this->torqueExternalPntB_B.setZero();

    /* compute DCN [BN] */
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

    /* evaluate inertia tensor about center of mass */
    Eigen::MatrixXd ISCPntC_B;
    Eigen::Matrix3d cTilde;
    cTilde = eigenTilde(*this->c_B);
    ISCPntC_B = *this->ISCPntB_B - (*this->m_SC)(0,0)*cTilde*cTilde.transpose();

    std::vector<std::string>::iterator it;
    int c = 0;
    for(it = this->planetPropertyNames.begin(); it != this->planetPropertyNames.end(); it++) {
        double mu = (*this->muPlanet[c])(0,0);  /* in m^3/s^2 */

        /* determine spacecraft CM position relative to planet */
        Eigen::Vector3d r_CP_N = this->r_BN_N->getState() + dcm_BN.transpose()*(*this->c_B) - *(this->r_PN_N[c]);

        /* find orbit radius */
        double rMag = r_CP_N.norm();

        /* compute normalized position vector in B frame */
        Eigen::Vector3d rHat_B = dcm_BN * r_CP_N.normalized();

        /* compute gravity gradient torque */
        Eigen::Vector3d ggTorque;
        ggTorque = 3.0*mu/rMag/rMag/rMag * (ISCPntC_B * rHat_B);
        ggTorque = rHat_B.cross(ggTorque);

        /* sum up all gravity gradient contributions */
        this->torqueExternalPntB_B += ggTorque;
        c++;
    }

  	return;
}

/*! This method is called once per BSK update cycle.  It writes out a msg of the
    evaluated gravity gradient torque.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void GravityGradientEffector::UpdateState(uint64_t CurrentSimNanos)
{
	this->WriteOutputMessages(CurrentSimNanos);
	return;
}
