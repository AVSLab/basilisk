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

#ifndef _PRESCRIBEDROTATION1DOF_
#define _PRESCRIBEDROTATION1DOF_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/PrescribedRotationMsgPayload.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed 1 DOF Rotation Profiler Class */
class PrescribedRotation1DOF: public SysModel{
public:
    PrescribedRotation1DOF() = default;                                    //!< Constructor
    ~PrescribedRotation1DOF() = default;                                   //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                         //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                   //!< Update member function
    void setCoastOptionRampDuration(double rampDuration);                  //!< Setter for the coast option ramp duration
    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                     //!< Setter for the spinning body rotation axis
    void setThetaDDotMax(double thetaDDotMax);                             //!< Setter for the ramp segment scalar angular acceleration
    void setThetaInit(double thetaInit);                                   //!< Setter for the initial spinning body angle
    double getCoastOptionRampDuration() const;                             //!< Getter for the coast option ramp duration
    const Eigen::Vector3d &getRotHat_M() const;                            //!< Getter for the spinning body rotation axis
    double getThetaDDotMax() const;                                        //!< Getter for the ramp segment scalar angular acceleration
    double getThetaInit() const;                                           //!< Getter for the initial spinning body angle

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyInMsg;              //!< Input msg for the spinning body reference angle and angle rate
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;                 //!< Output msg for the spinning body angle and angle rate
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;        //!< Output msg for the spinning body prescribed rotational states

    BSKLogger *bskLogger;                                                  //!< BSK Logging

private:

    /* Coast option member functions */
    bool isInFirstRampSegment(double time) const;               //!< Method for determining if the current time is within the first ramp segment for the coast option
    bool isInCoastSegment(double time) const;                   //!< Method for determining if the current time is within the coast segment for the coast option
    bool isInSecondRampSegment(double time) const;              //!< Method for determining if the current time is within the second ramp segment for the coast option
    void computeCoastParameters();                              //!< Method for computing the required parameters for the rotation with a coast period
    void computeCoastSegment(double time);                      //!< Method for computing the scalar rotational states for the coast option coast period

    /* Non-coast option member functions */
    bool isInFirstRampSegmentNoCoast(double time) const;        //!< Method for determining if the current time is within the first ramp segment for the no coast option
    bool isInSecondRampSegmentNoCoast(double time) const;       //!< Method for determining if the current time is within the second ramp segment for the no coast option
    void computeParametersNoCoast();                            //!< Method for computing the required parameters for the rotation with no coast period

    /* Shared member functions */
    void computeFirstRampSegment(double time);                  //!< Method for computing the scalar rotational states for the first ramp segment
    void computeSecondRampSegment(double time);                 //!< Method for computing the scalar rotational states for the second ramp segment
    void computeRotationComplete();                             //!< Method for computing the scalar rotational states when the rotation is complete
    Eigen::Vector3d computeSigma_FM();                          //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_FM

    /* User-configurable variables */
    double coastOptionRampDuration;                             //!< [s] Ramp time used for the coast option
    double thetaDDotMax;                                        //!< [rad/s^2] Maximum angular acceleration of spinning body used in the ramp segments
    Eigen::Vector3d rotHat_M;                                   //!< Spinning body rotation axis in M frame components

    /* Coast option variables */
    double theta_tr;                                            //!< [rad] Angle at the end of the first ramp segment
    double theta_tc;                                            //!< [rad] Angle at the end of the coast segment
    double thetaDot_tr;                                         //!< [rad/s] Angle rate at the end of the first ramp segment
    double tr;                                                  //!< [s] The simulation time at the end of the first ramp segment
    double tc;                                                  //!< [s] The simulation time at the end of the coast period

    /* Non-coast option variables */
    double ts;                                                  //!< [s] The simulation time halfway through the rotation

    /* Shared module variables */
    double theta;                                               //!< [rad] Current angle
    double thetaDot;                                            //!< [rad/s] Current angle rate
    double thetaDDot;                                           //!< [rad/s^2] Current angular acceleration
    bool convergence;                                           //!< Boolean variable is true when the rotation is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the rotation
    double thetaInit;                                           //!< [rad] Initial spinning body angle from frame M to frame F about rotHat_M
    double thetaDotInit;                                        //!< [rad/s] Initial spinning body angle rate between frame M to frame F
    double thetaRef;                                            //!< [rad] Spinning body reference angle from frame M to frame F about rotHat_M
    double tf;                                                  //!< [s] Simulation time when the rotation is complete
    double a;                                                   //!< Parabolic constant for the first acceleration segment
    double b;                                                   //!< Parabolic constant for the second acceleration segment

};

#endif
