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
    void setRotAxis_M(const Eigen::Vector3d &rotAxis_M);                   //!< Setter for the spinning body rotation axis
    void setThetaDDotMax(double thetaDDotMax);                             //!< Setter for the ramp segment scalar angular acceleration
    void setThetaInit(double thetaInit);                                   //!< Setter for the initial spinning body angle
    const Eigen::Vector3d &getRotAxis_M() const;                           //!< Getter for the spinning body rotation axis
    double getThetaDDotMax() const;                                        //!< Getter for the ramp segment scalar angular acceleration
    double getThetaInit() const;                                           //!< Getter for the initial spinning body angle

    double theta;                                                          //!< [rad] Current angle
    double thetaDot;                                                       //!< [rad/s] Current angle rate
    double thetaDDot;                                                      //!< [rad/s^2] Current angular acceleration

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyInMsg;              //!< Input msg for the spinning body reference angle and angle rate
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;                 //!< Output msg for the spinning body angle and angle rate
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;        //!< Output msg for the spinning body prescribed rotational states

    BSKLogger *bskLogger;                                                  //!< BSK Logging

private:
    double thetaDDotMax;                                        //!< [rad/s^2] Maximum angular acceleration of spinning body
    Eigen::Vector3d rotAxis_M;                                  //!< Rotation axis for the maneuver in M frame components
    Eigen::Vector3d omega_FM_F;                                 //!< [rad/s] Angular velocity of frame F wrt frame M in F frame components
    Eigen::Vector3d omegaPrime_FM_F;                            //!< [rad/s^2] B frame time derivative of omega_FM_F in F frame components
    Eigen::Vector3d sigma_FM;                                   //!< MRP attitude of frame F with respect to frame M

    bool convergence;                                           //!< Boolean variable is true when the maneuver is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the maneuver
    double thetaInit;                                           //!< [rad] Initial spinning body angle from frame M to frame F about rotAxis_M
    double thetaDotInit;                                        //!< [rad/s] Initial spinning body angle rate between frame M to frame F
    double thetaRef;                                            //!< [rad] Reference angle from frame M to frame F about rotAxis_M
    double ts;                                                  //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf;                                                  //!< [s] Simulation time when the maneuver is finished
    double a;                                                   //!< Parabolic constant for the first half of the maneuver
    double b;                                                   //!< Parabolic constant for the second half of the maneuver

};

#endif
