/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef HILL_FRAME_RELATIVE_CONTROL_H
#define HILL_FRAME_RELATIVE_CONTROL_H

#include <array>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CmdForceInertialMsgPayload.h"
#include "architecture/msgPayloadDefC/HillRelStateMsgPayload.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "cMsgCInterface/CmdForceInertialMsg_C.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Computes an inertial control force command from Hill-frame relative position and velocity errors. */
class HillFrameRelativeControl : public SysModel {
public:
    /*! @brief Constructor */
    HillFrameRelativeControl();
    /*! @brief Destructor */
    ~HillFrameRelativeControl();

    /*! @brief Reset module and validate required input message wiring.
     *
     * @param currentSimNanos [ns] Current simulation time
     */
    void Reset(uint64_t currentSimNanos) override;

    /*! @brief Read messages, compute command, and write output.
     *
     * @param currentSimNanos [ns] Current simulation time
     */
    void UpdateState(uint64_t currentSimNanos) override;
    /*! @brief Initialize C-wrapped output messages. */
    void SelfInit() override;

    ReadFunctor<NavTransMsgPayload> chiefTransInMsg;                 //!< chief translational nav input message
    ReadFunctor<HillRelStateMsgPayload> hillStateInMsg;              //!< optional deputy-relative-to-chief Hill-frame state input message
    ReadFunctor<NavTransMsgPayload> deputyTransInMsg;                //!< optional deputy translational nav input message (exclusive with hillStateInMsg)
    ReadFunctor<VehicleConfigMsgPayload> deputyVehicleConfigInMsg;   //!< deputy mass configuration input message
    Message<CmdForceInertialMsgPayload> forceOutMsg;                 //!< inertial force command output message
    CmdForceInertialMsg_C forceOutMsgC = {};                         //!< C-wrapped inertial force command output message

    BSKLogger bskLogger;                                             //!< BSK logging object

    /*! @brief Set Hill-frame position feedback gain matrix.
     *
     * @param gain [1/s^2] Position-error gain matrix in row-major order (length 9)
     */
    void setK(const std::vector<double>& gain);

    /*! @brief Set Hill-frame velocity feedback gain matrix.
     *
     * @param gain [1/s] Velocity-error gain matrix in row-major order (length 9)
     */
    void setP(const std::vector<double>& gain);

    /*! @brief Set Hill-frame relative position reference.
     *
     * @param rRef_H [m] Relative-position reference vector in Hill frame
     */
    void setReferencePosition(const std::vector<double>& rRef_H);

    /*! @brief Set Hill-frame relative velocity reference.
     *
     * @param vRef_H [m/s] Relative-velocity reference vector in Hill frame
     */
    void setReferenceVelocity(const std::vector<double>& vRef_H);

    /*! @brief Set central-body gravitational parameter.
     *
     * @param mu [m^3/s^2] Gravitational parameter used in A1 feedforward dynamics
     */
    void setMu(double mu);

    /*! @brief Get Hill-frame position feedback gain matrix.
     *
     * @return [1/s^2] Position-error gain matrix in row-major order
     */
    std::vector<double> getK() const;

    /*! @brief Get Hill-frame velocity feedback gain matrix.
     *
     * @return [1/s] Velocity-error gain matrix in row-major order
     */
    std::vector<double> getP() const;

    /*! @brief Get Hill-frame relative position reference.
     *
     * @return [m] Relative-position reference vector in Hill frame
     */
    std::vector<double> getReferencePosition() const;

    /*! @brief Get Hill-frame relative velocity reference.
     *
     * @return [m/s] Relative-velocity reference vector in Hill frame
     */
    std::vector<double> getReferenceVelocity() const;

    /*! @brief Get central-body gravitational parameter.
     *
     * @return [m^3/s^2] Gravitational parameter
     */
    double getMu() const;

private:
    /*! @brief Read all subscribed input messages into local buffers. */
    void readMessages();

    /*! @brief Compute inertial force command from Hill-frame tracking errors. */
    void computeForceCommand();

    /*! @brief Write inertial force command output message.
     *
     * @param currentSimNanos [ns] Current simulation time
     */
    void writeMessages(uint64_t currentSimNanos);

    NavTransMsgPayload chiefNavBuffer;               //!< chief nav input message buffer
    HillRelStateMsgPayload hillStateBuffer;          //!< Hill relative state input message buffer
    NavTransMsgPayload deputyNavBuffer;              //!< deputy nav input message buffer
    VehicleConfigMsgPayload deputyVehicleBuffer;     //!< deputy vehicle config buffer

    double K[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}; //!< Hill-frame position feedback gain matrix [1/s^2]
    double P[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}; //!< Hill-frame velocity feedback gain matrix [1/s]
    bool setKFlag = false; //!< flag to check if K has been set
    bool setPFlag = false; //!< flag to check if P has been set
    std::array<double, 3> rRef_H = {0.0, 0.0, 0.0}; //!< reference Hill-frame relative position [m]
    std::array<double, 3> vRef_H = {0.0, 0.0, 0.0}; //!< reference Hill-frame relative velocity [m/s]
    double mu = 0.0; //!< [m^3/s^2] gravitational parameter used in feedforward dynamics
    double forceCmd_N[3] = {0.0, 0.0, 0.0}; //!< commanded force in inertial frame [N]
};

#endif
