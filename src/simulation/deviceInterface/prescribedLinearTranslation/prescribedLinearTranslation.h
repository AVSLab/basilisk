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

#ifndef _PRESCRIBEDTRANS_
#define _PRESCRIBEDTRANS_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/PrescribedTranslationMsg_C.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed Linear Translation Profiler Class */
class PrescribedLinearTranslation: public SysModel {
public:
    PrescribedLinearTranslation() = default;                                    //!< Constructor
    ~PrescribedLinearTranslation() = default;                                   //!< Destructor

    void SelfInit() override;                                                   //!< Member function to initialize the C-wrapped output message
    void Reset(uint64_t CurrentSimNanos) override;                              //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                        //!< Update member function
    void setCoastOptionBangDuration(const double bangDuration);                 //!< Setter method for the coast option bang duration
    void setSmoothingDuration(const double smoothingDuration);                  //!< Setter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value
    void setTransAccelMax(const double transAccelMax);                          //!< Setter method for the bang segment scalar linear acceleration
    void setTransHat_M(const Eigen::Vector3d &transHat_M);                      //!< Setter method for the translating body axis of translation
    void setTransPosInit(const double transPosInit);                            //!< Setter method for the initial translating body hub-relative position
    double getCoastOptionBangDuration() const;                                  //!< Getter method for the coast option bang duration
    double getSmoothingDuration() const;                                        //!< Getter method for the duration the acceleration is smoothed until reaching the given maximum acceleration value
    double getTransAccelMax() const;                                            //!< Getter method for the bang segment scalar linear acceleration
    const Eigen::Vector3d &getTransHat_M() const;                               //!< Getter method for the translating body axis of translation
    double getTransPosInit() const;                                             //!< Getter method for the initial translating body position

    ReadFunctor<LinearTranslationRigidBodyMsgPayload> linearTranslationRigidBodyInMsg;    //!< Input msg for the translational reference position and velocity
    Message<LinearTranslationRigidBodyMsgPayload> linearTranslationRigidBodyOutMsg;       //!< Output msg for the translational reference position and velocity
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;                 //!< Output msg for the translational body prescribed states
    PrescribedTranslationMsg_C prescribedTranslationOutMsgC = {};                         //!< C-wrapped Output msg for the translational body prescribed states

    BSKLogger *bskLogger;                                                       //!< BSK Logging

private:
    /* Methods for computing the required translation parameters */
    void computeTranslationParameters();                                        //!< Intermediate method to group the calculation of translation parameters into a single method
    void computeBangBangParametersNoSmoothing();                                //!< Method for computing the required parameters for the non-smoothed bang-bang profiler option
    void computeBangCoastBangParametersNoSmoothing();                           //!< Method for computing the required parameters for the non-smoothed bang-coast-bang profiler option
    void computeSmoothedBangBangParameters();                                   //!< Method for computing the required parameters for the translation with a smoothed bang-bang acceleration profile
    void computeSmoothedBangCoastBangParameters();                              //!< Method for computing the required parameters for the smoothed bang-coast-bang option

    /* Methods for computing the current translational states */
    void computeCurrentState(double time);                                      //!< Intermediate method used to group the calculation of the current translational states into a single method
    bool isInFirstBangSegment(double time) const;                               //!< Method for determining if the current time is within the first bang segment
    bool isInSecondBangSegment(double time) const;                              //!< Method for determining if the current time is within the second bang segment
    bool isInFirstSmoothedSegment(double time) const;                           //!< Method for determining if the current time is within the first smoothing segment for the smoothed profiler options
    bool isInSecondSmoothedSegment(double time) const;                          //!< Method for determining if the current time is within the second smoothing segment for the smoothed profiler options
    bool isInThirdSmoothedSegment(double time) const;                           //!< Method for determining if the current time is within the third smoothing segment for the smoothed profiler options
    bool isInFourthSmoothedSegment(double time) const;                          //!< Method for determining if the current time is within the fourth smoothing segment for the smoothed bang-coast-bang option
    bool isInCoastSegment(double time) const;                                   //!< Method for determining if the current time is within the coast segment
    void computeFirstBangSegment(double time);                                  //!< Method for computing the first bang segment scalar translational states
    void computeSecondBangSegment(double time);                                 //!< Method for computing the second bang segment scalar translational states
    void computeFirstSmoothedSegment(double time);                              //!< Method for computing the first smoothing segment scalar translational states for the smoothed profiler options
    void computeSecondSmoothedSegment(double time);                             //!< Method for computing the second smoothing segment scalar translational states for the smoothed profiler options
    void computeThirdSmoothedSegment(double time);                              //!< Method for computing the third smoothing segment scalar translational states for the smoothed profiler options
    void computeFourthSmoothedSegment(double time);                             //!< Method for computing the fourth smoothing segment scalar translational states for the smoothed bang-coast-bang option
    void computeCoastSegment(double time);                                      //!< Method for computing the coast segment scalar translational states
    void computeTranslationComplete();                                          //!< Method for computing the scalar translational states when the translation is complete

    void writeOutputMessages(uint64_t CurrentSimNanos);                         //!< Method for writing the module output messages and computing the output message data

    /* User-configurable variables */
    double coastOptionBangDuration;                                             //!< [s] Time used for the coast option bang segment
    double smoothingDuration;                                                   //!< [s] Time the acceleration is smoothed to the given maximum acceleration value
    double transAccelMax;                                                       //!< [m/s^2] Maximum acceleration magnitude
    Eigen::Vector3d transHat_M;                                                 //!< Axis along the direction of translation expressed in M frame components

    /* Scalar translational states */
    double transPosInit;                                                        //!< [m] Initial translational body position from M to P frame origin along transHat_M
    double transPosRef;                                                         //!< [m] Reference translational body position from M to P frame origin along transHat_M
    double transPos;                                                            //!< [m] Current translational body position along transHat_M
    double transVel;                                                            //!< [m] Current translational body velocity along transHat_M
    double transAccel;                                                          //!< [m] Current translational body acceleration along transHat_M
    double transPos_tb1;                                                        //!< [m] Position at the end of the first bang segment
    double transVel_tb1;                                                        //!< [m/s] Velocity at the end of the first bang segment
    double transPos_tb2;                                                        //!< [m] Position at the end of the second bang segment
    double transVel_tb2;                                                        //!< [m/s] Velocity at the end of the second bang segment
    double transPos_ts1;                                                        //!< [m] Position at the end of the first smoothed segment
    double transVel_ts1;                                                        //!< [m/s] Velocity at the end of the first smoothed segment
    double transPos_ts2;                                                        //!< [m] Position at the end of the second smoothed segment
    double transVel_ts2;                                                        //!< [m/s] Velocity at the end of the second smoothed segment
    double transPos_ts3;                                                        //!< [m] Position at the end of the third smoothed segment
    double transVel_ts3;                                                        //!< [m/s] Velocity at the end of the third smoothed segment
    double transPos_tc;                                                         //!< [m] Position at the end of the coast segment
    double transVel_tc;                                                         //!< [m/s] Velocity at the end of the coast segment

    /* Temporal parameters */
    double tInit;                                                               //!< [s] Simulation time at the beginning of the translation
    double t_b1;                                                                //!< [s] Simulation time at the end of the first bang segment
    double t_b2;                                                                //!< [s] Simulation time at the end of the second bang segment
    double t_s1;                                                                //!< [s] Simulation time at the end of the first smoothed segment
    double t_s2;                                                                //!< [s] Simulation time at the end of the second smoothed segment
    double t_s3;                                                                //!< [s] Simulation time at the end of the third smoothed segment
    double t_c;                                                                 //!< [s] Simulation time at the end of the coast segment
    double t_f;                                                                 //!< [s] Simulation time when the translation is complete

    bool convergence;                                                           //!< Boolean variable is true when the translation is complete
    double a;                                                                   //!< Parabolic constant for the first half of the bang-bang non-smoothed translation
    double b;                                                                   //!< Parabolic constant for the second half of the bang-bang non-smoothed translation
};

#endif
