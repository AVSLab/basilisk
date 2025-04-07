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
    void setCoastOptionBangDuration(const double bangDuration);            //!< Setter for the coast option bang duration
    void setRotHat_M(const Eigen::Vector3d &rotHat_M);                     //!< Setter for the spinning body rotation axis
    void setSmoothingDuration(const double smoothingDuration);             //!< Setter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value
    void setThetaDDotMax(const double thetaDDotMax);                       //!< Setter for the bang segment scalar angular acceleration
    void setThetaInit(const double thetaInit);                             //!< Setter for the initial spinning body angle
    double getCoastOptionBangDuration() const;                             //!< Getter for the coast option bang duration
    const Eigen::Vector3d &getRotHat_M() const;                            //!< Getter for the spinning body rotation axis
    double getSmoothingDuration() const;                                   //!< Getter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value
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
    void computeRotationParameters();                                      //!< Intermediate method to group the calculation of rotation parameters into a single method
    void computeBangBangParametersNoSmoothing();                           //!< Method for computing the required parameters for the non-smoothed bang-bang profiler option
    void computeBangCoastBangParametersNoSmoothing();                      //!< Method for computing the required parameters for the non-smoothed bang-coast-bang profiler option
    void computeSmoothedBangBangParameters();                              //!< Method for computing the required parameters for the smoothed bang-bang profiler option
    void computeSmoothedBangCoastBangParameters();                         //!< Method for computing the required parameters for the smoothed bang-coast-bang profiler option

    /* Methods for computing the current rotational states */
    void computeCurrentState(double time);                                 //!< Intermediate method used to group the calculation of the current rotational states into a single method
    bool isInFirstBangSegment(double time) const;                          //!< Method for determining if the current time is within the first bang segment
    bool isInSecondBangSegment(double time) const;                         //!< Method for determining if the current time is within the second bang segment
    bool isInFirstSmoothedSegment(double time) const;                      //!< Method for determining if the current time is within the first smoothing segment for the smoothed profiler options
    bool isInSecondSmoothedSegment(double time) const;                     //!< Method for determining if the current time is within the second smoothing segment for the smoothed profiler options
    bool isInThirdSmoothedSegment(double time) const;                      //!< Method for determining if the current time is within the third smoothing segment for the smoothed profiler options
    bool isInFourthSmoothedSegment(double time) const;                     //!< Method for determining if the current time is within the fourth smoothing segment for the smoothed bang-coast-bang option
    bool isInCoastSegment(double time) const;                              //!< Method for determining if the current time is within the coast segment
    void computeFirstBangSegment(double time);                             //!< Method for computing the first bang segment scalar rotational states
    void computeSecondBangSegment(double time);                            //!< Method for computing the second bang segment scalar rotational states
    void computeFirstSmoothedSegment(double time);                         //!< Method for computing the first smoothing segment scalar rotational states for the smoothed profiler options
    void computeSecondSmoothedSegment(double time);                        //!< Method for computing the second smoothing segment scalar rotational states for the smoothed profiler options
    void computeThirdSmoothedSegment(double time);                         //!< Method for computing the third smoothing segment scalar rotational states for the smoothed profiler options
    void computeFourthSmoothedSegment(double time);                        //!< Method for computing the fourth smoothing segment scalar rotational states for the smoothed bang-coast-bang option
    void computeCoastSegment(double time);                                 //!< Method for computing the coast segment scalar rotational states
    void computeRotationComplete();                                        //!< Method for computing the scalar rotational states when the rotation is complete

    void writeOutputMessages(uint64_t CurrentSimNanos);                    //!< Method for writing the module output messages and computing the output message data
    Eigen::Vector3d computeSigma_PM();                                     //!< Method for computing the current spinning body MRP attitude relative to the mount frame: sigma_PM

    /* User-configurable variables */
    double coastOptionBangDuration;                                        //!< [s] Time used for the coast option bang segment
    double smoothingDuration;                                              //!< [s] Time the acceleration is smoothed to the given maximum acceleration value
    double thetaDDotMax;                                                   //!< [rad/s^2] Maximum angular acceleration of spinning body used in the bang segments
    Eigen::Vector3d rotHat_M;                                              //!< Spinning body rotation axis in M frame components

    /* Scalar rotational states */
    double thetaInit;                                                      //!< [rad] Initial spinning body angle from frame M to frame P about rotHat_M
    double thetaRef;                                                       //!< [rad] Spinning body reference angle from frame M to frame P about rotHat_M
    double theta;                                                          //!< [rad] Current angle
    double thetaDot;                                                       //!< [rad/s] Current angle rate
    double thetaDDot;                                                      //!< [rad/s^2] Current angular acceleration
    double theta_tb1;                                                      //!< [rad] Angle at the end of the first bang segment
    double thetaDot_tb1;                                                   //!< [rad/s] Angle rate at the end of the first bang segment
    double theta_tb2;                                                      //!< [rad] Angle at the end of the second bang segment
    double thetaDot_tb2;                                                   //!< [rad/s] Angle rate at the end of the second bang segment
    double theta_ts1;                                                      //!< [rad] Angle at the end of the first smoothed segment
    double thetaDot_ts1;                                                   //!< [rad/s] Angle rate at the end of the first smoothed segment
    double theta_ts2;                                                      //!< [rad] Angle at the end of the second smoothed segment
    double thetaDot_ts2;                                                   //!< [rad/s] Angle rate at the end of the second smoothed segment
    double theta_ts3;                                                      //!< [rad] Angle at the end of the third smoothed segment
    double thetaDot_ts3;                                                   //!< [rad/s] Angle rate at the end of the third smoothed segment
    double theta_tc;                                                       //!< [rad] Angle at the end of the coast segment
    double thetaDot_tc;                                                    //!< [rad/s] Angle rate at the end of the coast segment

    /* Temporal parameters */
    double tInit;                                                          //!< [s] Simulation time at the beginning of the rotation
    double t_b1;                                                           //!< [s] Simulation time at the end of the first bang segment
    double t_b2;                                                           //!< [s] Simulation time at the end of the second bang segment
    double t_s1;                                                           //!< [s] Simulation time at the end of the first smoothed segment
    double t_s2;                                                           //!< [s] Simulation time at the end of the second smoothed segment
    double t_s3;                                                           //!< [s] Simulation time at the end of the third smoothed segment
    double t_c;                                                            //!< [s] Simulation time at the end of the coast segment
    double t_f;                                                            //!< [s] Simulation time when the rotation is complete

    bool convergence;                                                      //!< Boolean variable is true when the rotation is complete
    double a;                                                              //!< Parabolic constant for the first half of the bang-bang non-smoothed rotation
    double b;                                                              //!< Parabolic constant for the second half of the bang-bang non-smoothed rotation
};

#endif
