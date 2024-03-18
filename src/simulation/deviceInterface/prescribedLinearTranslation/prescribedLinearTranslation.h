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
    void setCoastOptionBangDuration(double bangDuration);                       //!< Setter method for the coast option bang duration
    void setTransAccelMax(double transAccelMax);                                //!< Setter method for the bang segment scalar linear acceleration
    void setTransHat_M(const Eigen::Vector3d &transHat_M);                      //!< Setter method for the translating body axis of translation
    void setTransPosInit(double transPosInit);                                  //!< Setter method for the initial translating body hub-relative position
    double getCoastOptionBangDuration() const;                                  //!< Getter method for the coast option bang duration
    double getTransAccelMax() const;                                            //!< Getter method for the bang segment scalar linear acceleration
    const Eigen::Vector3d &getTransHat_M() const;                               //!< Getter method for the translating body axis of translation
    double getTransPosInit() const;                                             //!< Getter method for the initial translating body position

    ReadFunctor<LinearTranslationRigidBodyMsgPayload> linearTranslationRigidBodyInMsg;    //!< Input msg for the translational reference position and velocity
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;                 //!< Output msg for the translational body prescribed states
    PrescribedTranslationMsg_C prescribedTranslationOutMsgC = {};                         //!< C-wrapped Output msg for the translational body prescribed states

    BSKLogger *bskLogger;                                                       //!< BSK Logging

private:
    /* Methods for computing the required translation parameters */
    void computeParametersNoCoast();                                            //!< Method for computing the required parameters for the translation with no coast period
    void computeCoastParameters();                                              //!< Method for computing the required parameters for the translation with a coast period

    /* Methods for computing the current translational states */
    void computeCurrentState(double time);                                      //!< Intermediate method used to group the calculation of the current translational states into a single method
    bool isInFirstBangSegmentNoCoast(double time) const;                        //!< Method for determining if the current time is within the first bang segment for the no coast option
    bool isInFirstBangSegment(double time) const;                               //!< Method for determining if the current time is within the first bang segment for the coast option
    bool isInSecondBangSegmentNoCoast(double time) const;                       //!< Method for determining if the current time is within the second bang segment for the no coast option
    bool isInSecondBangSegment(double time) const;                              //!< Method for determining if the current time is within the second bang segment for the coast option
    bool isInCoastSegment(double time) const;                                   //!< Method for determining if the current time is within the coast segment for the coast option
    void computeFirstBangSegment(double time);                                  //!< Method for computing the scalar translational states for the first bang segment
    void computeSecondBangSegment(double time);                                 //!< Method for computing the scalar translational states for the second bang segment
    void computeCoastSegment(double time);                                      //!< Method for computing the scalar translational states for the coast option coast period
    void computeTranslationComplete();                                          //!< Method for computing the scalar translational states when the translation is complete

    void writeOutputMessages(uint64_t CurrentSimNanos);                         //!< Method for writing the module output messages and computing the output message data

    /* User-configurable variables */
    double coastOptionBangDuration;                                             //!< [s] Time used for the coast option bang segment
    double transAccelMax;                                                       //!< [m/s^2] Maximum acceleration magnitude
    Eigen::Vector3d transHat_M;                                                 //!< Axis along the direction of translation expressed in M frame components

    /* Scalar translational states */
    double transPosInit;                                                        //!< [m] Initial translational body position from M to F frame origin along transHat_M
    double transPosRef;                                                         //!< [m] Reference translational body position from M to F frame origin along transHat_M
    double transPos;                                                            //!< [m] Current translational body position along transHat_M
    double transVel;                                                            //!< [m] Current translational body velocity along transHat_M
    double transAccel;                                                          //!< [m] Current translational body acceleration along transHat_M
    double transPos_tr;                                                         //!< [m] Position at the end of the first bang segment
    double transVel_tr;                                                         //!< [m/s] Velocity at the end of the first bang segment

    /* Temporal parameters */
    double tInit;                                                               //!< [s] Simulation time at the beginning of the translation
    double t_r;                                                                 //!< [s] The simulation time at the end of the first bang segment
    double t_s;                                                                 //!< [s] The simulation time halfway through the translation
    double t_c;                                                                 //!< [s] Simulation time at the end of the coast segment
    double t_f;                                                                 //!< [s] Simulation time when the translation is complete

    bool convergence;                                                           //!< Boolean variable is true when the translation is complete
    double a;                                                                   //!< Parabolic constant for the first half of the bang-bang non-smoothed translation
    double b;                                                                   //!< Parabolic constant for the second half of the bang-bang non-smoothed translation
};

#endif
