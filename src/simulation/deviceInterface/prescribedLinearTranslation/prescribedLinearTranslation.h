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

    PrescribedLinearTranslation() = default;                                //!< Constructor
    ~PrescribedLinearTranslation() = default;                               //!< Destructor

    void SelfInit() override;                                               //!< Member function to initialize the C-wrapped output message
    void Reset(uint64_t CurrentSimNanos) override;                          //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                    //!< Update member function
    void setCoastOptionRampDuration(double rampDuration);                   //!< Setter method for the coast option ramp duration
    void setTransAccelMax(double transAccelMax);                            //!< Setter method for the ramp segment scalar linear acceleration
    void setTransHat_M(const Eigen::Vector3d &transHat_M);                  //!< Setter method for the translating body axis of translation
    void setTransPosInit(double transPosInit);                              //!< Setter method for the initial translating body hub-relative position
    double getCoastOptionRampDuration() const;                              //!< Getter method for the coast option ramp duration
    double getTransAccelMax() const;                                        //!< Getter method for the ramp segment scalar linear acceleration
    const Eigen::Vector3d &getTransHat_M() const;                           //!< Getter method for the translating body axis of translation
    double getTransPosInit() const;                                         //!< Getter method for the initial translating body position
    
    ReadFunctor<LinearTranslationRigidBodyMsgPayload> linearTranslationRigidBodyInMsg;    //!< Input msg for the translational reference position and velocity
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;                 //!< Output msg for the translational body prescribed states
    PrescribedTranslationMsg_C prescribedTranslationOutMsgC = {};                         //!< C-wrapped Output msg for the translational body prescribed states

    BSKLogger *bskLogger;                                                   //!< BSK Logging

private:

    /* Coast option member functions */
    bool isInFirstRampSegment(double time) const;               //!< Method for determining if the current time is within the first ramp segment for the coast option
    bool isInCoastSegment(double time) const;                   //!< Method for determining if the current time is within the coast segment for the coast option
    bool isInSecondRampSegment(double time) const;              //!< Method for determining if the current time is within the second ramp segment for the coast option
    void computeCoastParameters();                              //!< Method for computing the required parameters for the translation with a coast period
    void computeCoastSegment(double time);                      //!< Method for computing the scalar translational states for the coast option coast period

    /* Non-coast option member functions */
    bool isInFirstRampSegmentNoCoast(double time) const;        //!< Method for determining if the current time is within the first ramp segment for the no coast option
    bool isInSecondRampSegmentNoCoast(double time) const;       //!< Method for determining if the current time is within the second ramp segment for the no coast option
    void computeParametersNoCoast();                            //!< Method for computing the required parameters for the translation with no coast period

    /* Shared member functions */
    void computeFirstRampSegment(double time);                  //!< Method for computing the scalar translational states for the first ramp segment
    void computeSecondRampSegment(double time);                 //!< Method for computing the scalar translational states for the second ramp segment
    void computeTranslationComplete();                          //!< Method for computing the scalar translational states when the translation is complete

    /* User-configurable variables */
    double coastOptionRampDuration;                             //!< [s] Ramp time used for the coast option
    double transAccelMax;                                       //!< [m/s^2] Maximum acceleration magnitude
    Eigen::Vector3d transHat_M;                                 //!< Axis along the direction of translation expressed in M frame components

    /* Coast option variables */
    double transPos_tr;                                         //!< [m] Position at the end of the first ramp segment
    double transVel_tr;                                         //!< [m/s] Velocity at the end of the first ramp segment
    double tr;                                                  //!< [s] The simulation time at the end of the first ramp segment
    double tc;                                                  //!< [s] The simulation time at the end of the coast period

    /* Non-coast option variables */
    double ts;                                                  //!< [s] The simulation time halfway through the translation

    /* Shared module variables */
    double transPos;                                            //!< [m] Current translational body position along transHat_M
    double transVel;                                            //!< [m] Current translational body velocity along transHat_M
    double transAccel;                                          //!< [m] Current translational body acceleration along transHat_M
    bool convergence;                                           //!< Boolean variable is true when the translation is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the translation
    double transPosInit;                                        //!< [m] Initial translational body position from M to F frame origin along transHat_M
    double transPosRef;                                         //!< [m] Reference translational body position from M to F frame origin along transHat_M
    double tf;                                                  //!< [s] The simulation time when the translation is complete
    double a;                                                   //!< Parabolic constant for the first half of the translation
    double b;                                                   //!< Parabolic constant for the second half of the translation
};

#endif
