/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJSITE_H
#define MJSITE_H

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "MJObject.h"

#include <Eigen/Dense>

class MJBody;

/// @cond
/**
 * @brief Specialization of getObjectTypeName for mjsSite.
 *
 * Provides a human-readable name for the MuJoCo site type.
 *
 * @return A string view representing the site type.
 */
template <>
constexpr std::string_view MJBasilisk::detail::getObjectTypeName<mjsSite>()
{
    return "site";
}
/// @endcond

/**
 * @brief Represents a MuJoCo site, which is a reference point attached to a body.
 *
 * The `MJSite` class provides functionality to manage a 'site' in MuJoCo. A 'site'
 * represents a point in MuJoCo where one may query dynamic information (such as
 * position, orientation, velocity...) and apply forces and torques.
 */
class MJSite : public MJObject<mjsSite>
{
public:
    /**
     * @brief Constructs an `MJSite` object with a given MuJoCo site object
     * and the body where it's attached.
     *
     * @param site Pointer to the MuJoCo site.
     * @param body Reference to the body the site is attached to.
     */
    MJSite(mjsSite* site, MJBody& body);

    // Delete copy and move constructors and assignment operators
    MJSite(const MJSite&) = delete;
    MJSite(MJSite&&) = delete;
    MJSite& operator=(const MJSite&) = delete;
    MJSite& operator=(MJSite&&) = delete;

    /**
     * @brief Retrieves the body the site is attached to.
     *
     * @return A reference to the attached `MJBody` object.
     */
    MJBody& getBody() { return body; }

    /**
     * @brief Sets the position of the site relative to the body it's attached to.
     *
     * @param position The 3D position vector relative to the body, in the body reference frame.
     */
    void setPositionRelativeToBody(const Eigen::Vector3d& position);

    /**
     * @brief Sets the attitude (orientation) of the site relative to the body it's attached to.
     *
     * @param attitude The orientation represented as Modified Rodrigues Parameters (MRP) from the body reference frame.
     */
    void setAttitudeRelativeToBody(const Eigen::MRPd& attitude);

    /**
     * @brief Writes the frame state message (`stateOutMsg`) based on the
     * state of the site.
     *
     * Sets the position, velocity, orientation, and angular rate of the site
     * in the inertial frame, as parsed from the given MuJoCo model and data.
     *
     * @param model Pointer to the MuJoCo model.
     * @param data Pointer to the MuJoCo simulation data.
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     */
    void writeFwdKinematicsMessage(mjModel* model, mjData* data, uint64_t CurrentSimNanos);

public:
    Message<SCStatesMsgPayload> stateOutMsg; ///< Message to output site frame state.

protected:
    MJBody& body; ///< Reference to the body to which the site is attached.
};

#endif
