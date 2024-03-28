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
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/PrescribedRotationMsg_C.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed 1 DOF Rotation Profiler Class */
class PrescribedRotation1DOF: public SysModel{
public:
    PrescribedRotation1DOF() = default;                                    //!< Constructor
    ~PrescribedRotation1DOF() = default;                                   //!< Destructor

    void SelfInit() override;                                              //!< Member function to initialize the C-wrapped output message
    void Reset(uint64_t CurrentSimNanos) override;                         //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                   //!< Update member function
    void setCoastOptionBangDuration(double bangDuration);                  //!< Setter for the coast option bang duration
    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                     //!< Setter for the spinning body rotation axis
    void setThetaDDotMax(double thetaDDotMax);                             //!< Setter for the bang segment scalar angular acceleration
    void setThetaInit(double thetaInit);                                   //!< Setter for the initial spinning body angle
    double getCoastOptionBangDuration() const;                             //!< Getter for the coast option bang duration
    const Eigen::Vector3d &getRotHat_M() const;                            //!< Getter for the spinning body rotation axis
    double getThetaDDotMax() const;                                        //!< Getter for the bang segment scalar angular acceleration
    double getThetaInit() const;                                           //!< Getter for the initial spinning body angle

    ReadFunctor<HingedRigidBodyMsgPayload> spinningBodyInMsg;              //!< Input msg for the spinning body reference angle and angle rate
    Message<HingedRigidBodyMsgPayload> spinningBodyOutMsg;                 //!< Output msg for the spinning body angle and angle rate
    Message<PrescribedRotationMsgPayload> prescribedRotationOutMsg;        //!< Output msg for the spinning body prescribed rotational states
    HingedRigidBodyMsg_C spinningBodyOutMsgC = {};                         //!< C-wrapped output msg for the spinning body angle and angle rate
    PrescribedRotationMsg_C prescribedRotationOutMsgC = {};                //!< C-wrapped output msg for the spinning body prescribed rotational states

    BSKLogger *bskLogger;                                                  //!< BSK Logging

private:
    /* Methods for computing the required rotational parameters */
    void computeBangBangParametersNoSmoothing();                           //!< Method for computing the required parameters for the non-smoothed bang-bang profiler option
    void computeBangCoastBangParametersNoSmoothing();                      //!< Method for computing the required parameters for the non-smoothed bang-coast-bang profiler option

    /* Methods for computing the current rotational states */
    void computeCurrentState(double time);                                 //!< Intermediate method used to group the calculation of the current rotational states into a single method
    bool isInFirstBangSegmentNoCoast(double time) const;                   //!< Method for determining if the current time is within the first bang segment for the no coast option
    bool isInFirstBangSegment(double time) const;                          //!< Method for determining if the current time is within the first bang segment for the coast option
    bool isInSecondBangSegmentNoCoast(double time) const;                  //!< Method for determining if the current time is within the second bang segment for the no coast option
    bool isInSecondBangSegment(double time) const;                         //!< Method for determining if the current time is within the second bang segment for the coast option
    bool isInCoastSegment(double time) const;                              //!< Method for determining if the current time is within the coast segment for the coast option
    void computeFirstBangSegment(double time);                             //!< Method for computing the scalar rotational states for the first bang segment
    void computeSecondBangSegment(double time);                            //!< Method for computing the scalar rotational states for the second bang segment
    void computeCoastSegment(double time);                                 //!< Method for computing the scalar rotational states for the coast option coast period
    void computeRotationComplete();                                        //!< Method for computing the scalar rotational states when the rotation is complete

    void writeOutputMessages(uint64_t CurrentSimNanos);                    //!< Method for writing the module output messages and computing the output message data
    Eigen::Vector3d computeSigma_FM();                                     //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_FM

    /* User-configurable variables */
    double coastOptionBangDuration;                                        //!< [s] Time used for the coast option bang segment
    double thetaDDotMax;                                                   //!< [rad/s^2] Maximum angular acceleration of spinning body used in the bang segments
    Eigen::Vector3d rotHat_M;                                              //!< Spinning body rotation axis in M frame components

    /* Scalar rotational states */
    double thetaInit;                                                      //!< [rad] Initial spinning body angle from frame M to frame F about rotHat_M
    double thetaRef;                                                       //!< [rad] Spinning body reference angle from frame M to frame F about rotHat_M
    double theta;                                                          //!< [rad] Current angle
    double thetaDot;                                                       //!< [rad/s] Current angle rate
    double thetaDDot;                                                      //!< [rad/s^2] Current angular acceleration
    double theta_tr;                                                       //!< [rad] Angle at the end of the first bang segment
    double theta_tc;                                                       //!< [rad] Angle at the end of the coast segment
    double thetaDot_tr;                                                    //!< [rad/s] Angle rate at the end of the first bang segment

    /* Temporal parameters */
    double tInit;                                                          //!< [s] Simulation time at the beginning of the rotation
    double t_r;                                                            //!< [s] The simulation time at the end of the first bang segment
    double t_s;                                                            //!< [s] The simulation time halfway through the rotation
    double t_c;                                                            //!< [s] The simulation time at the end of the coast period
    double t_f;                                                            //!< [s] The simulation time when the rotation is complete

    bool convergence;                                                      //!< Boolean variable is true when the rotation is complete
    double a;                                                              //!< Parabolic constant for the first half of the bang-bang non-smoothed rotation
    double b;                                                              //!< Parabolic constant for the second half of the bang-bang non-smoothed rotation
};

#endif
