/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "prescribedMotionStateEffector.h"

/*! This is the constructor, setting variables to default values */
PrescribedMotionStateEffector::PrescribedMotionStateEffector()
{
    return;
}

/*! This is the destructor, nothing to report here */
PrescribedMotionStateEffector::~PrescribedMotionStateEffector()
{
    return;
}

/*! This method is used to reset the module. */
void PrescribedMotionStateEffector::Reset(uint64_t CurrentClock)
{
    return;
}


/*! This method takes the computed states and outputs them to the messaging system. */
void PrescribedMotionStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void PrescribedMotionStateEffector::prependSpacecraftNameToStates()
{
    return;
}

/*! This method allows the effector to have access to the hub states */
void PrescribedMotionStateEffector::linkInStates(DynParamManager& statesIn)
{
    return;
}

/*! This method allows the state effector to register its states with the dynamic parameter manager (unused) */
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
    return;
}

/*! This method allows the state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void PrescribedMotionStateEffector::updateEffectorMassProps(double integTime)
{
    return;
}

/*! This method allows the state effector to give its contributions to the matrices needed for the back-sub
 method */
void PrescribedMotionStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    return;
}

/*! This method is empty because the equations of motion of the effector are prescribed */
void PrescribedMotionStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    return;
}

/*! This method is for calculating the contributions of the effector to the energy and momentum of the spacecraft */
void PrescribedMotionStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    return;
}

/*! This method computes the effector states relative to the inertial frame */
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{
    return;
}

/*! This method updates the effector state at the dynamics frequency */
void PrescribedMotionStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    return;
}
