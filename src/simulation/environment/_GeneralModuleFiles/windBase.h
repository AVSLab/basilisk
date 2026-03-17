/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#ifndef WIND_BASE_H
#define WIND_BASE_H

#include <Eigen/Dense>
#include <time.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/WindMsgPayload.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/astroConstants.h"

/*! @brief Abstract base class for atmospheric wind models.
 *
 * WindBase reads spacecraft state input messages and writes wind velocity
 * output messages.  Concrete subclasses implement evaluateWindModel() to fill
 * the WindMsgPayload for each tracked spacecraft. The module supports multiple
 * spacecraft through the addSpacecraftToModel() method.
 */
class WindBase : public SysModel {
public:
    /*! This method initializes some basic parameters for the module.

    */
    WindBase();
    /*! Destructor.

    */
    ~WindBase();

    /*! Reset: resolves the epoch and calls customReset() for subclass-specific initialization.
     *  @param CurrentSimNanos  Current simulation time (ns).
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /*! Main update method: reads messages, evaluates wind model, writes output.
     *  @param CurrentSimNanos  Current simulation time (ns).
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /*! Adds a spacecraft to the model by subscribing to its state message and creating an output message.
     *  @param tmpScMsg  Spacecraft state message to subscribe to.
     */
    void addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg);

protected:
    /*! Writes the wind output messages for each spacecraft.
     *  @param CurrentClock  Current simulation time (ns).
     */
    void writeMessages(uint64_t CurrentClock);

    /*! Reads the spacecraft and planet state messages.
     *  @return true if all required messages were successfully read.
     */
    bool readMessages();

    /*! Computes `r_BP_N` from spacecraft and planet positions.
     *
     *  @param planetState  Planet state message.
     *  @param scState      Spacecraft state message.
     *  @return Spacecraft position relative to planet in inertial frame [m].
     */
    Eigen::Vector3d computeRelativePos(const SpicePlanetStateMsgPayload& planetState,
                                      const SCStatesMsgPayload& scState) const;

    /*! Computes `r_BP_N`, then calls evaluateWindModel() for each spacecraft.
     *  @param currentTime  Current simulation time (s).
     */
    void updateLocalWind(double currentTime);

    /*! Pure-virtual method that each concrete wind model must implement.
     *  @param msg                 Output wind message to fill.
     *  @param r_BP_N            Spacecraft position relative to planet in inertial frame [m].
     *  @param v_corotatingAir_N  Co-rotating atmosphere velocity [m/s] in N frame.
     *  @param currentTime         Current simulation time (s).
     */
    virtual void evaluateWindModel(WindMsgPayload *msg, const Eigen::Vector3d& r_BP_N,
                                  const Eigen::Vector3d& v_corotatingAir_N, double currentTime) = 0;

    /*! Optional subclass reset hook.
     *  @param CurrentClock  Current simulation time (ns).
     */
    virtual void customReset(uint64_t CurrentClock);

    /*! Optional subclass write hook.
     *  @param CurrentClock  Current simulation time (ns).
     */
    virtual void customWriteMessages(uint64_t CurrentClock);

    /*! Optional subclass read hook.
     *  @return true if successful.
     */
    virtual bool customReadMessages();

    /*! Optional hook called from Reset() when epochInMsg is not linked.
     *  Subclasses can override this to set epochDateTime from a module variable.
     */
    virtual void customSetEpochFromVariable();

public:
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs{};  //!< Spacecraft state input messages
    std::vector<Message<WindMsgPayload>*> envOutMsgs{};            //!< Wind velocity output messages
    ReadFunctor<SpicePlanetStateMsgPayload> planetPosInMsg{};      //!< Planet SPICE state input message
    ReadFunctor<EpochMsgPayload> epochInMsg{};                     //!< (optional) epoch date/time input message
    BSKLogger bskLogger{};                                         //!< -- BSK Logging

    /*! Returns the planet angular velocity vector in the inertial frame.
     *  @return  Planet angular velocity [rad/s] in N frame.
     */
    Eigen::Vector3d getPlanetOmega_N() const;

    /*! Sets the planet angular velocity vector in the inertial frame.
     *
     * This method sets the manual planet angular velocity and disables SPICE-derived omega.
     * To re-enable SPICE-derived omega, call setUseSpiceOmegaFlag(true).
     *
     * @param omega  Planet angular velocity [rad/s] in N frame.
     * @throws BSK_ERROR if input contains NaN or infinite values.
     */
    void setPlanetOmega_N(const Eigen::Vector3d &omega);

    /*! Gets the current useSpiceOmega flag.
     *
     * @return true if SPICE-derived omega is being used when available.
     */
    bool getUseSpiceOmegaFlag() const;

    /*! Sets the useSpiceOmega flag.
     *
     * @param useSpice  If true, use SPICE-derived omega when available, fallback to manual otherwise.
     */
    void setUseSpiceOmegaFlag(bool useSpice);

protected:
    struct tm epochDateTime{};                   //!< Epoch date/time (Gregorian) for time-dependent models
    std::vector<WindMsgPayload> envOutBuffer{};  //!< Message write buffer for each spacecraft
    std::vector<SCStatesMsgPayload> scStates{};  //!< Cached spacecraft state messages
    SpicePlanetStateMsgPayload planetState{};    //!< Cached planet state message

    /*! Updates `spiceOmega_N` from `J20002Pfix_dot` when SPICE provides it.
     *
     *  With ```C = J20002Pfix``` (maps inertial N to planet-fixed P), the kinematic equation
     *  gives ```skew_PN_P = -C_dot * C^T```.  spiceOmega_N is extracted from that skew-symmetric
     *  matrix and rotated to the inertial frame: ```spiceOmega_N = C^T * omega_PN_P```.
     *  The update is skipped when `J20002Pfix_dot` is zero (SPICE did not compute orientation
     *  derivatives), preserving the manually set value.
     */
    void updatePlanetOmegaFromSpice();

    /*! Returns the selected planet angular velocity to use in wind calculations.
     *  Either `planetOmega_N` set by the user or `spiceOmega_N` computed from SPICE.
     *
     * @return Selected planet angular velocity [rad/s] in N frame.
     */
    Eigen::Vector3d getSelectedOmega();

private:
    bool useSpiceOmega = true;                                //!< [bool] If true, use SPICE-derived omega; if false, use manually set value
    Eigen::Vector3d planetOmega_N = {0.0, 0.0, OMEGA_EARTH};  //!< [rad/s] Planet angular velocity in inertial frame (default: Earth rotation rate)
    Eigen::Vector3d spiceOmega_N = {0.0, 0.0, 0.0};           //!< [rad/s] Planet angular velocity derived from SPICE DCM derivatives

};

#endif /* WIND_BASE_H */
