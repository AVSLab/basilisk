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

#ifndef _PRESCRIBEDTRANS_
#define _PRESCRIBEDTRANS_

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/msgPayloadDefC/PrescribedTranslationMsgPayload.h"
#include "architecture/msgPayloadDefC/LinearTranslationRigidBodyMsgPayload.h"
#include <Eigen/Dense>
#include <cstdint>

/*! @brief Prescribed Linear Translation Profiler Class */
class PrescribedTranslation: public SysModel {
public:
    PrescribedTranslation() = default;                                      //!< Constructor
    ~PrescribedTranslation() = default;                                     //!< Destructor

    void Reset(uint64_t CurrentSimNanos) override;                          //!< Reset member function
    void UpdateState(uint64_t CurrentSimNanos) override;                    //!< Update member function

    void setCoastOptionRampDuration(double rampDuration);                   //!< Setter method for the coast option ramp duration
    void setR_FM_M(const Eigen::Vector3d &r_FM_M);                          //!< Setter method for the translating body hub-relative position vector
    void setRPrime_FM_M(const Eigen::Vector3d &rPrime_FM_M);                //!< Setter method for the translating body hub-relative velocity vector
    void setRPrimePrime_FM_M(const Eigen::Vector3d &rPrimePrime_FM_M);      //!< Setter method for the translating body hub-relative acceleration vector
    void setTransAccelMax(double transAccelMax);                            //!< Setter method for the ramp segment scalar linear acceleration
    void setTransAxis_M(const Eigen::Vector3d &transAxis_M);                //!< Setter method for the translating body axis of translation
    void setTransPosInit(double transPosInit);                              //!< Setter method for the initial translating body hub-relative position
    double getCoastOptionRampDuration() const;                              //!< Getter method for the coast option ramp duration
    const Eigen::Vector3d &getR_FM_M() const;                               //!< Getter method for the translating body's hub-relative position vector
    const Eigen::Vector3d &getRPrime_FM_M() const;                          //!< Getter method for the translating body's hub-relative linear velocity vector
    const Eigen::Vector3d &getRPrimePrime_FM_M() const;                     //!< Getter method for the translating body's hub-relative linear acceleration vector
    double getTransAccelMax() const;                                        //!< Getter method for the ramp segment scalar linear acceleration
    const Eigen::Vector3d &getTransAxis_M() const;                          //!< Getter method for the translating body axis of translation
    double getTransPosInit() const;                                         //!< Getter method for the initial translating body position

    double transPos;                                                        //!< [m] Current translational body position along transAxis_M
    double transVel;                                                        //!< [m] Current translational body velocity along transAxis_M
    double transAccel;                                                      //!< [m] Current translational body acceleration along transAxis_M

    ReadFunctor<LinearTranslationRigidBodyMsgPayload> linearTranslationRigidBodyInMsg;    //!< Input msg for the translational reference position and velocity
    Message<PrescribedTranslationMsgPayload> prescribedTranslationOutMsg;                 //!< Output msg for the translational body prescribed states

    BSKLogger *bskLogger;                                           //!< BSK Logging

private:

    /* User-configurable variables */
    double coastOptionRampDuration;                             //!< [s] Ramp time used for the coast option
    double transAccelMax;                                       //!< [m/s^2] Maximum acceleration magnitude
    Eigen::Vector3d transAxis_M;                                //!< Axis along the direction of translation expressed in M frame components
    Eigen::Vector3d r_FM_M;                                     //!< [m] Translational body position relative to the Mount frame expressed in M frame components
    Eigen::Vector3d rPrime_FM_M;                                //!< [m/s] B frame time derivative of r_FM_M expressed in M frame components
    Eigen::Vector3d rPrimePrime_FM_M;                           //!< [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components

    /* Coast option variables */
    double transPos_tr;                                         //!< [m] Position at the end of the first ramp segment
    double transPos_tc;                                         //!< [m] Position at the end of the coast segment
    double transVel_tr;                                         //!< [m/s] Velocity at the end of the first ramp segment
    double transVel_tc;                                         //!< [m/s] Velocity at the end of the coast segment
    double tr;                                                  //!< [s] The simulation time at the end of the first ramp segment
    double tc;                                                  //!< [s] The simulation time at the end of the coast period

    /* Non-coast option variables */
    double ts;                                                  //!< [s] The simulation time halfway through the translation

    /* Shared module variables */
    bool convergence;                                           //!< Boolean variable is true when the translation is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the translation
    double transPosInit;                                        //!< [m] Initial translational body position from M to F frame origin along transAxis_M
    double transVelInit;                                        //!< [m/s] Initial translational body velocity
    double transPosRef;                                         //!< [m] Reference translational body position from M to F frame origin along transAxis_M
    double tf;                                                  //!< [s] The simulation time when the translation is complete
    double a;                                                   //!< Parabolic constant for the first half of the translation
    double b;                                                   //!< Parabolic constant for the second half of the translation
};

#endif
